# MAX30102 Heart Rate & SpO2 Monitor

基于ESP32和MAX30102的高精度心率血氧监测系统，采用PBA+Maxim双算法融合。
利用esp32s3读取传感器数据，并显示在OLED上面

## 特性
- 双心率算法融合（PBA实时检测 + Maxim精确计算）
- 多级信号滤波（IIR/移动平均 + 指数平滑）
- 动态LED增益调节（适配不同肤色）
- 温度补偿算法
- OLED实时显示（128x64）
- 完整的串口调试输出


## 精度说明
与华为gt3手表数据对比，基本一致

## 硬件连接（直连，也可选用面包板）
| 模块 | SDA | SCL |
| OLED | GPIO8 | GPIO9 |
| MAX30102 | GPIO47 | GPIO21 |

## 依赖库
- [SparkFun MAX3010x Sensor Library](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library)
- [ESP8266_SSD1306](https://github.com/ThingPulse/esp8266-oled-ssd1306)

## 快速开始
1. 安装Arduino IDE
2. 通过库管理器安装依赖库
3. 打开`src/MAX30102_Monitor.ino`
4. 选择ESP32S3开发板，编译上传

## 文件test4.0为详细代码

## 效果展示

## 许可证
MIT License
