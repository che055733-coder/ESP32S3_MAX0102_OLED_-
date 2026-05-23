
/*硬件连接：
OLED：SDA-8
      SCL-9
MAX: SDA-47
     SCL-21*/

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"      // 新增：心率算法库
#include "SSD1306Wire.h"

// ========== OLED ==========
#define OLED_SDA 8
#define OLED_SCL 9

// ========== MAX30102 ==========
#define MAX_SDA 47
#define MAX_SCL 21

// ========== 优化参数 ==========
#define BUFFER_SIZE 100      // 缓冲区大小
#define STEP_SIZE 25         // 滑动窗口步长
#define SMOOTH_WINDOW 5      // 移动平均窗口大小

// ========== 心率算法参数 ==========
#define RATE_SIZE 4          // 心率平均窗口大小
byte heartRates[RATE_SIZE];  // 心率数组
byte rateSpot = 0;
long lastBeat = 0;           // 上次心跳时间
float beatsPerMinute = 0;
int beatAvg = 0;

// ========== OLED用 Wire ==========
SSD1306Wire display(0x3C, OLED_SDA, OLED_SCL);

// ========== MAX30102用 Wire1 ==========
TwoWire I2C_MAX = TwoWire(1);
MAX30105 particleSensor;

// ========== 数据缓存 ==========
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
uint32_t irRaw[BUFFER_SIZE];
uint32_t redRaw[BUFFER_SIZE];

int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

// 高精度血氧值
float preciseSpO2 = 0;
float filteredSpO2 = 0;

// 新增：融合后的心率和血氧
int finalHeartRate = 0;
float finalSpO2Value = 0;

// ========== 移动平均滤波 ==========
void movingAverageFilter(uint32_t *input, uint32_t *output, int len, int windowSize) {
    if (len < windowSize) return;
    
    for (int i = 0; i < len; i++) {
        uint32_t sum = 0;
        int count = 0;
        for (int j = -windowSize/2; j <= windowSize/2; j++) {
            int idx = i + j;
            if (idx >= 0 && idx < len) {
                sum += input[idx];
                count++;
            }
        }
        output[i] = sum / count;
    }
}

// ========== 优化的血氧计算（二次拟合公式）==========
float calculateSpO2_Optimized(uint32_t *red, uint32_t *ir, int len) {
    // 计算 DC 分量（平均值）
    uint64_t red_sum = 0, ir_sum = 0;
    for (int i = 0; i < len; i++) {
        red_sum += red[i];
        ir_sum += ir[i];
    }
    
    if (ir_sum == 0) return 0;
    
    float red_dc = (float)red_sum / len;
    float ir_dc = (float)ir_sum / len;
    
    // 计算 AC 分量（使用标准差，比峰峰值更稳定）
    float red_ac_var = 0, ir_ac_var = 0;
    for (int i = 0; i < len; i++) {
        red_ac_var += pow((float)red[i] - red_dc, 2);
        ir_ac_var += pow((float)ir[i] - ir_dc, 2);
    }
    float red_ac = sqrt(red_ac_var / len);
    float ir_ac = sqrt(ir_ac_var / len);
    
    // 避免除零
    if (ir_ac == 0 || ir_dc == 0) return 0;
    
    // 计算 R 值
    float R = (red_ac / red_dc) / (ir_ac / ir_dc);
    
    // 二次拟合公式（Maxim 官方推荐）
    float spo2 = -45.060f * R * R + 30.354f * R + 94.845f;
    
    // 限幅到合理范围
    if (spo2 > 100) spo2 = 100;
    if (spo2 < 70) spo2 = 0;
    
    return spo2;
}

// ========== 温度补偿 ==========
float temperatureCompensation(float spo2) {
    float temp = particleSensor.readTemperature();
    
    // 温度补偿公式：以25°C为基准，每偏离1°C补偿0.12%
    float compensation = (25.0f - temp) * 0.12f;
    float compensated = spo2 + compensation;

    // 限幅
    if (compensated > 100) compensated = 100;
    if (compensated < 70) compensated = 0;
    
    return compensated;
}

// ========== 指数移动平均（平滑血氧输出）==========
float exponentialMovingAverage(float newValue, float oldValue, float alpha) {
    return alpha * newValue + (1 - alpha) * oldValue;
}

// ========== 检测手指是否放置 ==========
bool isFingerPlaced() {
    // 计算红外信号的平均值（手指放置时信号会明显增强）
    uint64_t sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum += irRaw[i];
    }
    uint32_t avg = sum / BUFFER_SIZE;
    
    // 阈值根据实际信号强度调整（通常手指放置时 > 50000）
    return (avg > 30000);
}

// ========== 新增：PBA心率计算 ==========
void updateHeartRatePBA(long irValue) {
    if (checkForBeat(irValue) == true) {
        // 检测到心跳
        long delta = millis() - lastBeat;
        lastBeat = millis();
        
        beatsPerMinute = 60 / (delta / 1000.0);
        
        // 过滤异常值
        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
            heartRates[rateSpot++] = (byte)beatsPerMinute;
            rateSpot %= RATE_SIZE;
            
            // 计算平均值
            beatAvg = 0;
            for (byte x = 0; x < RATE_SIZE; x++) {
                beatAvg += heartRates[x];
            }
            beatAvg /= RATE_SIZE;
        }
    }
}

// ========== 心率融合算法 ==========
int getFusedHeartRate() {
    // 策略：优先使用PBA算法的平均心率（响应快）
    // 当PBA无效时使用Maxim官方心率
    // 两者都无效时返回0
    
    bool pbaValid = (beatAvg > 30 && beatAvg < 200 && beatAvg > 0);
    bool maximValid = (validHeartRate && heartRate > 30 && heartRate < 200);
    
    if (pbaValid && maximValid) {
        // 两者都有效：取平均
        return (beatAvg + (int)heartRate) / 2;
    } else if (pbaValid) {
        // 仅PBA有效
        return beatAvg;
    } else if (maximValid) {
        // 仅Maxim有效
        return heartRate;
    } else {
        // 都无效
        return 0;
    }
}

// ========== 血氧融合算法 ==========
float getFusedSpO2() {
    // 计算优化算法的血氧
    float optimizedSpO2 = calculateSpO2_Optimized(redBuffer, irBuffer, BUFFER_SIZE);
    
    float finalSpO2 = 0;
    
    // 优先使用优化算法
    if (optimizedSpO2 > 0 && optimizedSpO2 < 100) {
        finalSpO2 = optimizedSpO2;
    } else if (validSPO2 && spo2 > 0 && spo2 < 100) {
        // 备用：官方算法
        finalSpO2 = spo2;
    } else {
        return 0;
    }
    
    // 温度补偿
    finalSpO2 = temperatureCompensation(finalSpO2);
    
    // 指数移动平均平滑
    filteredSpO2 = exponentialMovingAverage(finalSpO2, filteredSpO2, 0.3f);
    
    return filteredSpO2;
}

// ========== OLED 显示 ==========
void updateDisplay() {
    display.clear();
    
    // 标题
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "MAX30102");
    
    // 手指检测状态
    display.setFont(ArialMT_Plain_10);
    if (!isFingerPlaced()) {
        display.drawString(0, 16, "Place finger...");
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 32, "HR: ---");
        display.drawString(0, 48, "SpO2: ---");
    } else {
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 16, "Measuring...");
        
        // 心率显示（使用融合后的值）
        display.setFont(ArialMT_Plain_16);
        if (finalHeartRate > 30 && finalHeartRate < 200) {
            display.drawString(0, 32, "HR: " + String(finalHeartRate));
        } else {
            display.drawString(0, 32, "HR: ---");
        }
        
        // 血氧显示
        if (finalSpO2Value > 85 && finalSpO2Value <= 100) {
            display.drawString(0, 48, "SpO2: " + String((int)finalSpO2Value) + "%");
        } else {
            display.drawString(0, 48, "SpO2: ---");
        }
    }
    
    display.display();
}

// ========== 串口调试 ==========
void printDebugInfo() {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        Serial.print("PBA HR: ");
        Serial.print(beatAvg);
        Serial.print(" | Maxim HR: ");
        Serial.print(heartRate);
        Serial.print(" | Final HR: ");
        Serial.print(finalHeartRate);
        Serial.print(" bpm  |  SpO2: ");
        Serial.print(finalSpO2Value);
        Serial.print("%  |  Valid: ");
        Serial.print(validHeartRate);
        Serial.print("/");
        Serial.print(validSPO2);
        Serial.print("  |  Finger: ");
        Serial.println(isFingerPlaced() ? "Yes" : "No");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("MAX30102 Optimized System Starting...");

    // ===== OLED I2C =====
    Wire.begin(OLED_SDA, OLED_SCL);
    delay(100);

    // ===== MAX I2C =====
    I2C_MAX.begin(MAX_SDA, MAX_SCL);
    delay(100);

    // ===== OLED 初始化 =====
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_16);
    display.clear();
    display.drawString(0, 0, "Init...");
    display.display();
    delay(1000);

    // ===== MAX30102 初始化 =====
    if (!particleSensor.begin(I2C_MAX, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 ERROR - Check wiring!");
        display.clear();
        display.drawString(0, 0, "MAX30102");
        display.drawString(0, 20, "ERROR!");
        display.drawString(0, 40, "Check wiring");
        display.display();
        while (1);
    }

    // ===== 复位传感器 =====
    particleSensor.softReset();
    delay(100);
    
    // ===== 优化的 MAX30102 配置 =====
    particleSensor.setup(
        400,    // 采样率 400Hz
        4,      // 样本平均 4
        2,      // LED模式: 血氧模式
        400,    // 采样率(重复)
        411,    // 脉冲宽度 411μs
        4096    // ADC量程 4096nA
    );

    // ===== LED 电流设置 =====
    particleSensor.setPulseAmplitudeRed(0x2F);   // 红光 47mA
    particleSensor.setPulseAmplitudeIR(0x25);    // 红外 37mA
    particleSensor.setPulseAmplitudeGreen(0);    // 关闭绿灯
    
    // 启用 FIFO Rollover
    particleSensor.enableFIFORollover();
    
    Serial.println("MAX30102 Initialized Successfully!");
    Serial.println("Configuration: 400Hz, 411us, 18bit");
    Serial.println("LED Current: Red=47mA, IR=37mA");
    
    // 显示就绪信息
    display.clear();
    display.drawString(0, 0, "Ready!");
    display.drawString(0, 20, "Place finger");
    display.drawString(0, 40, "on sensor");
    display.display();
    delay(2000);
    
    // 初始化滤波值
    filteredSpO2 = 96.0f;
}

void loop() {
    // ===== 首次填充缓冲区 =====
    for (int i = 0; i < BUFFER_SIZE; i++) {
        while (!particleSensor.available())
            particleSensor.check();
        
        redRaw[i] = particleSensor.getRed();
        irRaw[i] = particleSensor.getIR();
        
        // 新增：实时PBA心率计算（使用原始IR值）
        updateHeartRatePBA(irRaw[i]);
        
        particleSensor.nextSample();
    }
    
    // ===== 应用滤波 =====
    movingAverageFilter(redRaw, redBuffer, BUFFER_SIZE, SMOOTH_WINDOW);
    movingAverageFilter(irRaw, irBuffer, BUFFER_SIZE, SMOOTH_WINDOW);
    
    // ===== 使用官方算法 =====
    maxim_heart_rate_and_oxygen_saturation(
        irBuffer, BUFFER_SIZE,
        redBuffer,
        &spo2, &validSPO2,
        &heartRate, &validHeartRate
    );
    
    // ===== 获取融合后的血氧 =====
    finalSpO2Value = getFusedSpO2();
    
    // ===== 获取融合后的心率 =====
    finalHeartRate = getFusedHeartRate();
    
    // ===== 更新显示 =====
    updateDisplay();
    printDebugInfo();
    
    // ===== 滑动窗口持续更新 =====
    while (true) {
        // 滑动窗口：丢弃最旧的数据
        for (int i = STEP_SIZE; i < BUFFER_SIZE; i++) {
            redRaw[i - STEP_SIZE] = redRaw[i];
            irRaw[i - STEP_SIZE] = irRaw[i];
        }
        
        // 采集新样本
        for (int i = BUFFER_SIZE - STEP_SIZE; i < BUFFER_SIZE; i++) {
            while (!particleSensor.available())
                particleSensor.check();
            
            redRaw[i] = particleSensor.getRed();
            irRaw[i] = particleSensor.getIR();
            
            // 实时PBA心率计算
            updateHeartRatePBA(irRaw[i]);
            
            particleSensor.nextSample();
        }
        
        // 应用滤波
        movingAverageFilter(redRaw, redBuffer, BUFFER_SIZE, SMOOTH_WINDOW);
        movingAverageFilter(irRaw, irBuffer, BUFFER_SIZE, SMOOTH_WINDOW);
        
        // 重新计算
        maxim_heart_rate_and_oxygen_saturation(
            irBuffer, BUFFER_SIZE,
            redBuffer,
            &spo2, &validSPO2,
            &heartRate, &validHeartRate
        );
        
        // 更新融合值
        finalSpO2Value = getFusedSpO2();
        finalHeartRate = getFusedHeartRate();
        
        // 更新显示
        updateDisplay();
        printDebugInfo();
        
        delay(100);
    }
}