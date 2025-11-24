/*****************************************************************************
* | File      	:   mlx_cst8160.cpp
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
#include "mlx_cst8160.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "string.h" // memset()

static const char *TAG = "CST8160";

/**
 * @brief 初始化CST8160触摸屏
 */
esp_err_t cst8160_init(cst8160_dev_t *dev, i2c_port_t i2c_port, 
                     gpio_num_t sda_pin, gpio_num_t scl_pin,
                     gpio_num_t rst_pin, gpio_num_t int_pin,
                     uint16_t width, uint16_t height) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 初始化设备结构体
    dev->i2c_port = i2c_port;
    dev->addr = CST8160_ADDR;
    dev->rst_pin = rst_pin;
    dev->int_pin = int_pin;
    dev->screen_width = width;
    dev->screen_height = height;
    dev->rotation = CST8160_ROTATION_NORMAL;
    dev->touch_count = 0;
    memset(dev->points, 0, sizeof(dev->points));

    // 配置RST引脚
    if (dev->rst_pin != GPIO_NUM_NC) {
        gpio_config_t gpio_conf = {
            .pin_bit_mask = (1ULL << dev->rst_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&gpio_conf);
    }

    // 配置INT引脚（如果使用）
    if (dev->int_pin != GPIO_NUM_NC) {
        gpio_config_t gpio_conf = {
            .pin_bit_mask = (1ULL << dev->int_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&gpio_conf);
    }

    // 复位触摸屏
    esp_err_t ret = cst8160_reset(dev);
    if (ret != ESP_OK) {
        return ret;
    }

    // 检查芯片ID
    uint8_t chip_id;
    ret = cst8160_read_reg(dev, CST8160_REG_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }

    // CST8160的芯片ID通常为0xB4，这里做简单检查
    if (chip_id != 0xB4) {
        vTaskDelay(pdMS_TO_TICKS(5)); // 等待5ms，确保芯片已复位
        // 再读一次
        ret = cst8160_read_reg(dev, CST8160_REG_CHIP_ID, &chip_id);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read chip ID");
            return ret;
        }
        if (chip_id != 0xB4) {
            ESP_LOGW(TAG, "Invalid chip ID: 0x%02X (expected 0xB4)", chip_id);
        }
    }

    // 读取固件版本
    uint8_t firmware_version;
    ret = cst8160_read_reg(dev, CST8160_REG_FIRMWARE_VERSION, &firmware_version);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Firmware version: 0x%02X", firmware_version);
    }

    // 读取工厂ID
    uint8_t factory_id;
    ret = cst8160_read_reg(dev, CST8160_REG_FACTORY_ID, &factory_id);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Factory ID: 0x%02X", factory_id);
    }

    ESP_LOGI(TAG, "CST8160 initialized successfully");
    return ESP_OK;
}

/**
 * @brief 设置屏幕旋转方向
 */
void cst8160_set_rotation(cst8160_dev_t *dev, uint8_t rotation) {
    if (dev == NULL) return;
    if (rotation >= 4) rotation = 0;
    dev->rotation = rotation;
}

/**
 * @brief 读取触摸数据
 */
esp_err_t cst8160_read_touch(cst8160_dev_t *dev) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 读取触摸状态
    uint8_t status;
    esp_err_t ret = cst8160_read_reg(dev, CST8160_REG_TD_STATUS, &status);
    if (ret != ESP_OK) {
        return ret;
    }

    // 触摸点数量（0或1）
    dev->touch_count = status & 0x0F;
    if (dev->touch_count > 1) {
        dev->touch_count = 0;
    }

    // 清除之前的触摸点数据
    for (int i = 0; i < 1; i++) {
        dev->points[i].valid = false;
        dev->points[i].pressed = false;
    }

    // 读取触摸点数据
    if (dev->touch_count > 0) {
        uint8_t data[4]; // X高位, X低位, Y高位, Y低位
        
        // 读取X坐标
        ret = cst8160_read_reg(dev, CST8160_REG_TOUCH_X_HIGH, &data[0]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read X high byte");
            return ret;
        }
        
        ret = cst8160_read_reg(dev, CST8160_REG_TOUCH_X_LOW, &data[1]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read X low byte");
            return ret;
        }
        
        // 读取Y坐标
        ret = cst8160_read_reg(dev, CST8160_REG_TOUCH_Y_HIGH, &data[2]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read Y high byte");
            return ret;
        }
        
        ret = cst8160_read_reg(dev, CST8160_REG_TOUCH_Y_LOW, &data[3]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read Y low byte");
            return ret;
        }

        // 解析触摸数据
        uint16_t x = ((data[0] & 0x0F) << 8) | data[1];
        uint16_t y = ((data[2] & 0x0F) << 8) | data[3];
        uint8_t id = 0; // CST8160只支持单点触摸
        
        // 检查坐标有效性
        if (x < dev->screen_width && y < dev->screen_height) {
            // 根据旋转方向调整坐标
            uint16_t temp;
            switch (dev->rotation) {
                case CST8160_ROTATION_NORMAL:
                    // 正常方向，不调整
                    break;
                case CST8160_ROTATION_RIGHT:
                    temp = x;
                    x = y;
                    y = dev->screen_width - temp;
                    break;
                case CST8160_ROTATION_INVERTED:
                    x = dev->screen_width - x;
                    y = dev->screen_height - y;
                    break;
                case CST8160_ROTATION_LEFT:
                    temp = x;
                    x = dev->screen_height - y;
                    y = temp;
                    break;
            }

            // 存储触摸点数据
            dev->points[0].id = id;
            dev->points[0].x = x;
            dev->points[0].y = y;
            dev->points[0].valid = true;
            dev->points[0].pressed = true;
        }
    }

    return ESP_OK;
}

/**
 * @brief 检查是否有触摸
 */
bool cst8160_is_touched(cst8160_dev_t *dev) {
    if (dev == NULL) return false;

    // 如果使用了INT引脚，先检查INT引脚状态
    if (dev->int_pin != GPIO_NUM_NC) {
        if (gpio_get_level(dev->int_pin) != 0) {
            return false;
        }
    }

    // 读取触摸状态寄存器确认
    uint8_t status;
    if (cst8160_read_reg(dev, CST8160_REG_TD_STATUS, &status) != ESP_OK) {
        return false;
    }

    return (status & 0x0F) > 0;
}

/**
 * @brief 复位触摸屏
 */
esp_err_t cst8160_reset(cst8160_dev_t *dev) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 如果没有复位引脚，直接返回成功
    if (dev->rst_pin == GPIO_NUM_NC) {
        vTaskDelay(pdMS_TO_TICKS(100));
        return ESP_OK;
    }

    // 执行复位序列
    gpio_set_level(dev->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(dev->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

/**
 * @brief 写入寄存器
 */
esp_err_t cst8160_write_reg(cst8160_dev_t *dev, uint8_t reg, uint8_t data) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CST8160_ADDR_8BIT | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 * @brief 读取寄存器
 */
esp_err_t cst8160_read_reg(cst8160_dev_t *dev, uint8_t reg, uint8_t *data) {
    if (dev == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CST8160_ADDR_8BIT | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CST8160_ADDR_8BIT | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}
