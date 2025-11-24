# mlx-cst8160-drivers
Use CST8160 touch IC on ESP-IDF without INT# or RST#.

## 功能说明

支持ESP-IDF 5.1.4，使用硬件I2C；

特点如下：

- 用硬件 I2C 通信，简化了代码；
- 移除中断依赖，支持轮询方式检测触摸；
- 支持 RST 和 INT 引脚的可选配置；
- 增加屏幕旋转和坐标映射功能；

常规配置，同时使用SDA、SCL、RST#和INT#引脚，这些引脚都配置为上拉模式，其中RST#配置为（推挽）输出；

最简配置，仅使用SDA和SCL引脚，推荐将RST#连接到MCU的RST引脚，INT#悬空/10k电阻上拉；

极简配置，仅使用SDA和SCL引脚，将RST#和INT#均通过10k电阻上拉；

## 示例代码

见example，适配深圳秦唐盛世科技的2.0寸屏幕（ST7789V+CST8160，240x320 IPS），在屏幕设置了加/减号区域，可在串口输出触摸点数和触摸坐标，增加/减少计数；

## tft驱动

tft驱动详见：

- LCDWIKI驱动IDF适配版[LCDWIKI-drivers-idf](https://github.com/sprlightning/LCDWIKI-drivers-idf)，支持ESP-IDF 5.1.4环境；不过有个问题，屏幕刷新很慢；推荐使用类似TFT_eSPI这样的高效tft库。
