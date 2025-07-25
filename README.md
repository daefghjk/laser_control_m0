## 基于mspm0g3507的运动目标控制与自动追踪系统

### 引脚分配
#### UART都是单片机端
|外设|功能|引脚|
|---|---|---|
|步进电机|ST1|A21|
|       |ST2|A22|
|       |DIR1|B22|
|       |DIR2|B23|
|       |EN1|B21|
|       |EN2|B19|
|       |ADC|B17|
|摇杆|VRX|A27|
|   |VRY|A26|
|   |SW|B13|
|蜂鸣器|EN|B24|
|K230|RXD|A1|
|    |TXD|A0|
|UART1|RXD|B5|
|     |TXD|B4|
|UART2|RXD|B18|
|     |TXD|A23|
|UART3|RXD|A25|
|     |TXD|B12|
|OLED|SDA|A10|
|    |SCL|A11|
|I2C1|SDA|A16|
|    |SCL|A15|
|SPI0|SCLK|A12|
|    |PICO(MOSI)|A14|
|    |POCI(MISO)|A13|
|    |CS0|A2|
|SPI1|SCLK|B16|
|    |PICO|B15|
|    |POCI|B14|
|    |CS0|B20|
|PWM1|C0|A28|
|    |C1|A24|
|PWM2|C0|B10|
|    |C1|B11|
|剩余引脚|-|A7 A8 A9 A17 A18 A29 A30 A31<br> B0 B1 B2 B3 B6 B7 B8 B9 B25 B26 B27|

