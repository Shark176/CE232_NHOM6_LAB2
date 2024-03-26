/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <ssd1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "font8x8_basic.h"
static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           4      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           5      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU9250_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT                   7

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void ssd1306_init() {
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
	i2c_master_write_byte(cmd, 0x14, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping

	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGI(TAG, "OLED configured successfully");
	} else {
		ESP_LOGE(TAG, "OLED configuration failed. code: 0x%.2X", espRc);
	}
	i2c_cmd_link_delete(cmd);
}

void task_ssd1306_display_text(const void *arg_text, const int *page) {
	char *text = (char*)arg_text;
	uint8_t text_len = strlen(text);

	i2c_cmd_handle_t cmd;

	uint8_t cur_page = page;

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, 0x00, true); // reset column - choose column --> 0
	i2c_master_write_byte(cmd, 0x10, true); // reset line - choose line --> 0
	i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset page

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	for (uint8_t i = 0; i < text_len; i++) {
		if (text[i] == '\n') {
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
			i2c_master_write_byte(cmd, 0x00, true); // reset column
			i2c_master_write_byte(cmd, 0x10, true);
			i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
		} else {
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
			i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
		}
	}

	//vTaskDelete(NULL);
}

void task_ssd1306_display_clear(void *ignore) {
	i2c_cmd_handle_t cmd;

	uint8_t clear[128];
	for (uint8_t i = 0; i < 128; i++) {
		clear[i] = 0;
	}
	for (uint8_t i = 0; i < 8; i++) {
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
		i2c_master_write_byte(cmd, 0xB0 | i, true);

		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
		i2c_master_write(cmd, clear, 128, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
	}

	//vTaskDelete(NULL);
}

void task_ssd1306_display_scrol(void *ignore) {
	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
		
		i2c_master_write_byte(cmd, 0x2E, true);

		i2c_master_write_byte(cmd, 0x29, true);
		i2c_master_write_byte(cmd, 0x00, true);
		i2c_master_write_byte(cmd, 0x00, true);
		i2c_master_write_byte(cmd, 0x00, true);
		i2c_master_write_byte(cmd, 0x03, true);
		i2c_master_write_byte(cmd, 0x00, true);
		i2c_master_write_byte(cmd, 0x00, true);
		i2c_master_write_byte(cmd, 0x00, true);

		i2c_master_write_byte(cmd, 0x2F, true);

		i2c_master_stop(cmd);
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
}

static const uint8_t logo_UIT[]= {
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f,
    0x3f, 0x1f, 0x0f, 0x0f, 0x07, 0x07, 0x83, 0x43, 0x41, 0xa1, 0x11, 0x51, 0x29, 0xa9, 0x95, 0xd3,
    0xeb, 0xe9, 0x75, 0x77, 0x33, 0x3f, 0x3f, 0x1f, 0x1f, 0x0f, 0x8f, 0x0f, 0x8f, 0x8f, 0xc7, 0xc7,
    0xc7, 0xc7, 0x87, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x1f, 0x3f, 0x3f, 0x7f, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x0f, 0x83, 0x81, 0x40, 0xa0,
    0x50, 0x28, 0xb4, 0xd2, 0xe8, 0xf5, 0x72, 0x3a, 0x1d, 0x1c, 0x0e, 0x87, 0x87, 0xc3, 0xe1, 0xe1,
    0xf0, 0xf0, 0xf8, 0xfc, 0xfc, 0xfc, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xf9, 0xf3, 0xcf,
    0x3f, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xfc, 0xfe, 0xfe, 0xff, 0xff, 0xfe, 0xfc, 0xf0, 0x81, 0x0f,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x97, 0x68, 0x94, 0xca, 0xe5, 0x72, 0x39, 0x1c,
    0x0e, 0x07, 0x83, 0xc1, 0xe1, 0xf0, 0xf8, 0x7c, 0x3c, 0x3e, 0x3f, 0x7f, 0xff, 0xff, 0xff, 0x7f,
    0x3f, 0x3f, 0x1f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xf8, 0x83, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xe0,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x79, 0x3c, 0x0f, 0x07, 0x03, 0x80, 0xc0, 0xf0, 0xf8,
    0xfc, 0xfe, 0xff, 0xff, 0x1f, 0x03, 0x80, 0xc0, 0x30, 0x18, 0x04, 0x02, 0x01, 0x01, 0x01, 0x02,
    0x04, 0x18, 0x30, 0xc0, 0x80, 0x03, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xbf, 0xcf, 0xf1, 0xfe, 0xff,
    0xff, 0xff, 0xff, 0x3f, 0x0f, 0x03, 0x00, 0x00, 0xc0, 0xf0, 0xfc, 0xfe, 0xff, 0xff, 0x81, 0x01,
    0x01, 0x01, 0xff, 0xe3, 0x71, 0x0f, 0x03, 0x02, 0x82, 0xc4, 0xe4, 0xec, 0xf8, 0xf8, 0xf8, 0xec,
    0xc4, 0xc4, 0x82, 0x02, 0x03, 0x0f, 0x79, 0xff, 0xff, 0x01, 0x01, 0x01, 0x81, 0xff, 0xff, 0xff,
    0xff, 0x7f, 0x83, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x3f, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xcf, 0x8f, 0x0f, 0x0f, 0x0f,
    0x1f, 0x3d, 0x79, 0xf1, 0xe1, 0xc1, 0x81, 0x03, 0x07, 0x1f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x7f, 0x1f, 0x07, 0x03, 0x81, 0xc1, 0xe1, 0xf1, 0x79, 0x3d, 0x1f, 0x0f, 0x0f, 0x0f, 0x8f, 0xc7,
    0xf1, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x3f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xf8,
    0xf0, 0xf0, 0xe0, 0xc0, 0xc0, 0x81, 0x83, 0x03, 0x02, 0x04, 0x00, 0x00, 0x03, 0x07, 0x03, 0x00,
    0x00, 0x04, 0x02, 0x03, 0x03, 0x81, 0x81, 0xc0, 0xe0, 0xf0, 0xf0, 0xf8, 0xfc, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xfc, 0xf0, 0xe0, 0xc0, 0xc0, 0x80, 0x80, 0x80, 0x00, 0x01, 0x01, 0x03, 0x03, 0x07, 0x07,
    0x07, 0x07, 0x07, 0x87, 0x87, 0x87, 0x87, 0xc7, 0xc7, 0xc7, 0xe3, 0xe2, 0xf2, 0xf2, 0xf0, 0xf8,
    0xfc, 0xfc, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
void ssd1306_draw_bitmap(const uint8_t *bitmap, uint8_t x, uint8_t y, uint8_t height, uint8_t width)
{
	i2c_cmd_handle_t cmd;
	uint8_t page_start, page_end;
	uint8_t column_start, column_end;
	uint8_t page, column;

	// Calculate the start and end pages and columns for the bitmap
	page_start = y / 8;
	page_end = (y + height - 1) / 8;
	column_start = x;
	column_end = x + width - 1;

	// Loop through each page and column, writing the bitmap data to the display
	for (page = page_start; page <= page_end; page++)
	{
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
		i2c_master_write_byte(cmd, 0X00 | 0, true); // reset column
		i2c_master_write_byte(cmd, 0X10, true);
		i2c_master_write_byte(cmd, 0xB0 | page, true); // set page

		i2c_master_stop(cmd);
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);

		for (column = column_start; column <= column_end; column++)
		{
			uint8_t data = logo_UIT[(page - page_start) * width + (column - column_start)];
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
			i2c_master_write_byte(cmd, data, true);

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
		}
		
	}
}
void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    ssd1306_init();
	//BAI TAP 2
	task_ssd1306_display_clear(0);
    task_ssd1306_display_text("21522760",0);
 	vTaskDelay(50);
   	task_ssd1306_display_text("21522002",1);
	vTaskDelay(50);
   	task_ssd1306_display_text("21521036",2);
	vTaskDelay(50);
   	task_ssd1306_display_text("21522570",3);
	vTaskDelay(50);
	//BAI TAP 3
	task_ssd1306_display_clear(0);
	vTaskDelay(50);
	ssd1306_draw_bitmap(logo_UIT, 32, 0, 64, 64);
    vTaskDelay(500);
}
