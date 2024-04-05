#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <float.h>

#include "esp_event.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "xi2c.h"
#include "fonts.h"
#include "ssd1306.h"
#include "iot_button.h"

const static char *TAG1 = "FALL_DETECTION:";
extern QueueHandle_t data_queue;

//Button parameters
#define BUTTON_IO_NUM				0
#define BUTTON_ACTIVE_LEVEL         BUTTON_ACTIVE_LOW


//I2C parameters 
#define I2C_MASTER_SCL_IO           26
#define I2C_MASTER_SDA_IO           25
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000


//ACCELEROMETER registers and values
#define MPU6050_SENSOR_ADDR                 0x68
#define MPU6050_ACCEL_XOUT           		0x3B
#define MPU6050_ACCEL_YOUT           		0x3D
#define MPU6050_ACCEL_ZOUT           		0x3F
#define MPU6050_TEMP_OUT           			0x41


//Magnetometer registers and values
#define MAG3110_SENSOR_ADDR					0x0E  //the problem is with the address of this sensor or more in general with the sensor itself
#define MAG3110_OUT_X_MSB_ADDR              0x01
#define MAG3110_OUT_X_LSB_ADDR              0x02
#define MAG3110_OUT_Y_MSB_ADDR              0x03
#define MAG3110_OUT_Y_LSB_ADDR              0x04
#define MAG3110_OUT_Z_MSB_ADDR              0x05
#define MAG3110_OUT_Z_LSB_ADDR              0x06
#define MAG3110_OFF_X_MSB_REG				0x09
#define MAG3110_OFF_X_LSB_REG				0x0A
#define MAG3110_OFF_Y_MSB_REG				0x0B
#define MAG3110_OFF_Y_LSB_REG				0x0C
#define MAG3110_OFF_Z_MSB_REG				0x0D
#define MAG3110_OFF_Z_LSB_REG				0x0E

//calibration param
#define CAL_TIMEOUT 20	//in s 

static uint8_t reset_accel = 0;

static button_handle_t btn_handle = NULL; 

/////////////////////////////////////////////////
//I2C CONFIG FUCNCTIONS
//
///////////////////////////////////////////////////
static i2c_config_t i2c_conf;

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t* conf = &i2c_conf;

    conf->mode = I2C_MODE_MASTER;
	conf->sda_io_num = I2C_MASTER_SDA_IO;
	conf->scl_io_num = I2C_MASTER_SCL_IO;
	conf->sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf->scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf->master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(i2c_master_port, conf);

    return i2c_driver_install(i2c_master_port, conf->mode, 0, 0, 0);
}

/////////////////////////////////////////////////
//ACCELEROMETER FUNCTIONS 
//
///////////////////////////////////////////////////

static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

//ACCELERATION reading function
void mpu6050_accel_read(float *accelx, float *accely, float *accelz) {
	
	uint8_t data[2];
    data[0] = 0;
    data[1] = 0;
	int16_t accel_aux;
	uint8_t reset = 0;
	if(!uxQueueSpacesAvailable(data_queue)){
		xQueuePeek(data_queue,&(reset),0);
	}

	if(!reset){
		ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_ACCEL_XOUT, data, 2));
		accel_aux = (((data[0] <<8)) | data[1]);
		*accelx = accel_aux;
		*accelx = *accelx/16384;  //to get the acceleration in gs 
		ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_ACCEL_YOUT, data, 2));
		accel_aux = (((data[0] <<8)) | data[1]);
		*accely = accel_aux;
		*accely = *accely/16384;
		ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_ACCEL_ZOUT, data, 2));
		accel_aux = (((data[0] <<8)) | data[1]);
		*accelz = accel_aux; 
		*accelz = *accelz/16384; 
		
	} else{
		*accelx = 0; 
		*accely = 0;
		*accelz = 0;
	}
	
	return;
}

/////////////////////////////////////////////////
//MAGNETOMETER FUNCTIONS 
//
///////////////////////////////////////////////////

static esp_err_t mag3110_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MAG3110_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t mag3110_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MAG3110_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ret;
}

void mag3110_mag_read(float *magfx, float *magfy, float *magfz){

	uint8_t data[2];
    data[0] = 0;
    data[1] = 0;
	int16_t magf_aux;

	ESP_ERROR_CHECK(mag3110_register_read(MAG3110_OUT_X_MSB_ADDR, data, 2));
    magf_aux = (((data[0] <<8)) | data[1]);
    *magfx = magf_aux;
    *magfx = *magfx;  //to get the field in uT
    ESP_ERROR_CHECK(mag3110_register_read(MAG3110_OUT_Y_MSB_ADDR, data, 2));
    magf_aux = (((data[0] <<8)) | data[1]);
    *magfy = magf_aux;
    *magfy = *magfy;
    ESP_ERROR_CHECK(mag3110_register_read(MAG3110_OUT_Z_MSB_ADDR, data, 2));
    magf_aux = (((data[0] <<8)) | data[1]);
    *magfz = magf_aux;
    *magfz = *magfz;
	return; 
}


void mag3110_calibrating(){
	bool changed = false;
	float magfx;
	float magfy;
	float magfz;

	float max_magx = -FLT_MAX;
	float max_magy = -FLT_MAX;
	float max_magz = -FLT_MAX;
	float min_magx = FLT_MAX;
	float min_magy = FLT_MAX;
	float min_magz = FLT_MAX;

	float mag_off_x;
	float mag_off_y;
	float mag_off_z;
	
	int64_t start_time = esp_timer_get_time();  //time in us
	int64_t elapsed_time;
	do
	{
		changed = false;
		mag3110_mag_read(&magfx,&magfy,&magfz);
		//x axis xalibration 
		if (max_magx < magfx)
		{
			max_magx = magfx;
			changed = true; 
		}
		if (min_magx > magfx)
		{
			min_magx = magfx;
			changed = true; 
		}
		//y axis xalibration 
		if (max_magy < magfy)
		{
			max_magy = magfy;
			changed = true; 
		}
		if (min_magy > magfy)
		{
			min_magy = magfy;
			changed = true; 
		}
		//z axis xalibration 
		if (max_magz < magfz)
		{
			max_magz = magfz;
			changed = true; 
		}
		if (min_magz > magfz)
		{
			min_magz = magfz;
			changed = true; 
		}

		elapsed_time = esp_timer_get_time() - start_time;
		

	} while (changed || (elapsed_time<CAL_TIMEOUT*1000000));

	//computing the offset
	mag_off_x = (max_magx + min_magx)/2;
	mag_off_y = (max_magy + min_magy)/2;
	mag_off_z = (max_magz + min_magz)/2;


	ESP_LOGI(TAG1, "computed offset:");
	ESP_LOGI(TAG1, "mag_off_x: %f", mag_off_x);
	ESP_LOGI(TAG1, "mag_off_x: %f", mag_off_y);
	ESP_LOGI(TAG1, "mag_off_x: %f", mag_off_z);

	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_OFF_X_MSB_REG,(((int)mag_off_x<<1)>>8) & 0xFF));
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_OFF_X_LSB_REG,((int)mag_off_x<<1) & 0xFF));
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_OFF_Y_MSB_REG,(((int)mag_off_y<<1)>>8) & 0xFF));
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_OFF_Y_LSB_REG,(int)mag_off_y<<1 & 0xFF));
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_OFF_Z_MSB_REG,(((int)mag_off_z<<1)>>8) & 0xFF));
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_OFF_Z_LSB_REG,(int)mag_off_z<<1 & 0xFF));

	return; 

	
}
/////////////////////////////////////////////////
//BUTTON SETUP 
//
///////////////////////////////////////////////////

//call back that toggle the value of a variable
void btn_cb() {
	uint8_t tmp = 0; 
	ESP_LOGI(TAG1,"User defines button was pushed");
    reset_accel ^= 1;		
	ESP_LOGI(TAG1, "reset_accel = %d", reset_accel);
	if(!uxQueueSpacesAvailable(data_queue)){
		xQueueReceive(data_queue,&tmp, 0);
	} 
	xQueueSend(data_queue, &reset_accel, 0);
	return; 		
}

void button_init(){
	ESP_LOGI(TAG1, "STARTED: button initialization"); 
	btn_handle = iot_button_create((gpio_num_t)BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
	//set event callback
	ESP_ERROR_CHECK(iot_button_set_evt_cb(btn_handle, BUTTON_CB_TAP, btn_cb, NULL));
    ESP_LOGI(TAG1, "COMPLETED: Button initialization");
}
