/*****************************************************************************
* | File      	:   mlx_cst8160.h
* | Author      :   Adapted from MLX's FT6336 driver
* | Function    :   CST8160 Touch Functions
* | Info        :   参考了CST816T驱动和FT6336驱动，进行了独立整合；
* |             :   用硬件 I2C 通信，简化了代码；
* |             :   移除中断依赖，支持轮询方式检测触摸；
* |             :   支持 RST 和 INT 引脚的可选配置；
* |             :   增加屏幕旋转和坐标映射功能；
*----------------
* |	This version:   V1.0
* | Date        :   2025-11-22
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#ifndef MLX_CST8160_H
#define MLX_CST8160_H

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define CST8160_ADDR                 0x15        // I2C地址（7位）
#define CST8160_ADDR_8BIT            (CST8160_ADDR << 1)  // 8位地址

// 旋转方向定义
#define CST8160_ROTATION_NORMAL      0
#define CST8160_ROTATION_RIGHT       1
#define CST8160_ROTATION_INVERTED    2
#define CST8160_ROTATION_LEFT        3

#define CST8160_ROTATION_0           (CST8160_ROTATION_NORMAL)
#define CST8160_ROTATION_90          (CST8160_ROTATION_LEFT)
#define CST8160_ROTATION_180         (CST8160_ROTATION_INVERTED)
#define CST8160_ROTATION_270         (CST8160_ROTATION_RIGHT)

// 寄存器定义
#define CST8160_REG_CHIP_ID          0xA7        // 芯片ID寄存器
#define CST8160_REG_FIRMWARE_VERSION 0xA9        // 固件版本寄存器
#define CST8160_REG_FACTORY_ID       0xAA        // 工厂ID寄存器
#define CST8160_REG_TD_STATUS        0x02        // 触摸状态寄存器
#define CST8160_REG_TOUCH_X_HIGH     0x03        // X坐标高位寄存器
#define CST8160_REG_TOUCH_X_LOW      0x04        // X坐标低位寄存器
#define CST8160_REG_TOUCH_Y_HIGH     0x05        // Y坐标高位寄存器
#define CST8160_REG_TOUCH_Y_LOW      0x06        // Y坐标低位寄存器

// 触摸点结构体
typedef struct {
    uint8_t id;         // 触摸点ID
    uint16_t x;         // X坐标
    uint16_t y;         // Y坐标
    bool valid;         // 有效标志
    bool pressed;       // 按下状态
} cst8160_touch_point_t;

// CST8160设备结构体
typedef struct {
    i2c_port_t i2c_port;        // I2C端口号
    uint8_t addr;               // I2C地址（7位）
    gpio_num_t rst_pin;         // 复位引脚，-1表示不使用
    gpio_num_t int_pin;         // 中断引脚，-1表示不使用
    uint16_t screen_width;      // 屏幕宽度
    uint16_t screen_height;     // 屏幕高度
    uint8_t rotation;           // 旋转方向
    cst8160_touch_point_t points[1];  // 触摸点数据(最多支持1点触摸)
    uint8_t touch_count;        // 触摸点数量
} cst8160_dev_t;

/**
 * @brief 初始化CST8160触摸屏
 * 
 * @param dev CST8160设备结构体指针
 * @param i2c_port I2C端口号
 * @param sda_pin SDA引脚
 * @param scl_pin SCL引脚
 * @param rst_pin 复位引脚，-1表示不使用
 * @param int_pin 中断引脚，-1表示不使用
 * @param width 屏幕宽度
 * @param height 屏幕高度
 * @return esp_err_t 成功返回ESP_OK，否则返回错误码
 */
esp_err_t cst8160_init(cst8160_dev_t *dev, i2c_port_t i2c_port, 
                     gpio_num_t sda_pin, gpio_num_t scl_pin,
                     gpio_num_t rst_pin, gpio_num_t int_pin,
                     uint16_t width, uint16_t height);

/**
 * @brief 设置屏幕旋转方向
 * 
 * @param dev CST8160设备结构体指针
 * @param rotation 旋转方向，取值为CST8160_ROTATION_*
 */
void cst8160_set_rotation(cst8160_dev_t *dev, uint8_t rotation);

/**
 * @brief 读取触摸数据
 * 
 * @param dev CST8160设备结构体指针
 * @return esp_err_t 成功返回ESP_OK，否则返回错误码
 */
esp_err_t cst8160_read_touch(cst8160_dev_t *dev);

/**
 * @brief 检查是否有触摸
 * 
 * @param dev CST8160设备结构体指针
 * @return bool 有触摸返回true，否则返回false
 */
bool cst8160_is_touched(cst8160_dev_t *dev);

/**
 * @brief 复位触摸屏
 * 
 * @param dev CST8160设备结构体指针
 * @return esp_err_t 成功返回ESP_OK，否则返回错误码
 */
esp_err_t cst8160_reset(cst8160_dev_t *dev);

/**
 * @brief 写入寄存器
 * 
 * @param dev CST8160设备结构体指针
 * @param reg 寄存器地址
 * @param data 要写入的数据
 * @return esp_err_t 成功返回ESP_OK，否则返回错误码
 */
esp_err_t cst8160_write_reg(cst8160_dev_t *dev, uint8_t reg, uint8_t data);

/**
 * @brief 读取寄存器
 * 
 * @param dev CST8160设备结构体指针
 * @param reg 寄存器地址
 * @param data 读取到的数据
 * @return esp_err_t 成功返回ESP_OK，否则返回错误码
 */
esp_err_t cst8160_read_reg(cst8160_dev_t *dev, uint8_t reg, uint8_t *data);

#endif // MLX_CST8160_H
