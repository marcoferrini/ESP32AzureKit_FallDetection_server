#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "sdkconfig.h"

#include "bt_functions.c"
#include "xi2c.h"
#include "fonts.h"
#include "ssd1306.h"
#include "iot_button.h"
#include "sensors.c"

const static char *TAG = "FALL_DETECTION:";

//TASK
#define TASK_PRIORITY				5

//Angle threshold to detect a fall
#define ANGLE_THTRESHOLD					45   //deg

//Accel param confid
#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B
#define MPU6050_PWR_MGMT_1_VALUE 			0x00

#define MPU6050_ACCEL_CONFIG_REG_ADDR     	0x1C
#define MPU6050_ACCEL_CONFIG_2G_VALUE		0x00


//MAGNETOMETER registers and values config
#define MAG3110_SENSOR_ADDR					0x0E  //the problem is with the address of this sensor or more in general with the sensor itself
#define MAG3110_CTRL_REG1_ADDR				0x10
#define MAG3110_CTRL_REG2_ADDR				0x11

#define MAG3110_DR_STATUS_ADDR              0x00
#define MAG3110_OFF_X_MSB_REG				0x09
#define MAG3110_OFF_X_LSB_REG				0x0A
#define MAG3110_OFF_Y_MSB_REG				0x0B
#define MAG3110_OFF_Y_LSB_REG				0x0C
#define MAG3110_OFF_Z_MSB_REG				0x0D
#define MAG3110_OFF_Z_LSB_REG				0x0E


#define MAG3110_AUTO_MRST_EN				0x80    //auto_mrst_en 

#define MAG3110_CTRL_REG1_VALUE				0x01



#define ACCEL_THRESHOLD						1.5    //in gs 

#define TEST_MODE 							1
#define BLE_ON								1




QueueHandle_t data_queue;
TaskHandle_t fall_detection_task_handle = NULL;




static char print[8][16] = { "|Acc|  =   .    ",
						"x-axis =   .    ",
						"y-axis =   .    ",
						"z-axis =   .    ",
						"|Magf|  =   .   ",
						"x-axis =   .    ",
						"y-axis =   .    ",
						"z-axis =   .    "
	};





/**
 * @brief Task to monitor accelerometer and magnetometer data for fall detection.
 *
 * This task reads from accelerometer and magnetometer sensors to determine if a fall has occurred
 * based on computed angles and thresholds. Detected falls can trigger BLE notifications if configured.
 *
 * @param pvParameter Pointer to task parameter, not used in this task.
 */
void fall_detection_task(void *pvParameter){
	float accelx;
	float accely;
	float accelz;

	float magfx;
	float magfy;
	float magfz; 

	int print_array[8];

	float accel_mod;
	float mag_mod;

	float norm_standing_mag_component[3];
	float norm_mag_component[3];
	float norm_standing_acc_component[3];
	float norm_acc_component[3];
	float angle_accel = 0;
	float angle_mag = 0;
	
	//setted to one when the fall is detected would be nice to put it at zero when in the client a button is pressed
	uint8_t fall_detected[ATTR_VALUE_SIZE];

	vTaskDelay(10000 / portTICK_PERIOD_MS);  
	ESP_LOGI(TAG, "fall_detection_task started");
	fall_detected[0] = 0;
	#ifdef BLE_ON
		ble_attr_data_write(fall_detected);
	#endif


	mag3110_mag_read(&magfx,&magfy,&magfz);
	mag_mod = sqrt(pow(magfx,2) + pow(magfy,2) + pow(magfz,2)); 
	norm_standing_mag_component[0] = magfx/mag_mod;
	norm_standing_mag_component[1] = magfy/mag_mod;
	norm_standing_mag_component[2] = magfz/mag_mod;

	mpu6050_accel_read(&accelx, &accely, &accelz);
	accel_mod = (sqrt(pow(accelx,2) + pow(accely,2) + pow(accelz,2)));  
	norm_standing_acc_component[0] = accelx/accel_mod;
	norm_standing_acc_component[1] = accely/accel_mod;
	norm_standing_acc_component[2] = accelz/accel_mod;

	SSD1306_Fill(SSD1306_COLOR_BLACK);


	for(;;){ 
		#ifdef BLE_ON
			ble_attr_data_read(fall_detected);
		#endif
		//compute acceleration module and absolute value of its components
    	mpu6050_accel_read(&accelx, &accely, &accelz);

		
		print_array[1] = abs(1000*accelx); 
		print_array[2] = abs(1000*accely);
		print_array[3] = abs(1000*accelz);

		//accel module
		accel_mod =  1000*(sqrt(pow(accelx,2) + pow(accely,2) + pow(accelz,2)));  
		print_array[0] = accel_mod;
		

		//compute magnet field module and absolute value of its components
		mag3110_mag_read(&magfx, &magfy, &magfz);
		print_array[5] = magfx; 
		print_array[6] = magfy;
		print_array[7] = magfz;

		//magnetic field module 
		mag_mod= sqrt(pow(magfx,2)+pow(magfy,2)+pow(magfz,2));
		print_array[4] = mag_mod;
		  
		
		//detect the impact
		if((accel_mod > ACCEL_THRESHOLD*1000)) {
			angle_accel = 0; 
			angle_mag = 0; 

			//dalay task execution of 1s because of a eventual transient 
			vTaskDelay(1000 / portTICK_PERIOD_MS); 
			
			mpu6050_accel_read(&accelx, &accely, &accelz);
			accel_mod = (sqrt(pow(accelx,2) + pow(accely,2) + pow(accelz,2)));  
			norm_acc_component[0] = accelx/accel_mod;
			norm_acc_component[1] = accely/accel_mod;
			norm_acc_component[2] = accelz/accel_mod;

			mag3110_mag_read(&magfx, &magfy, &magfz);
			mag_mod = sqrt(pow(magfx,2) + pow(magfy,2) + pow(magfz,2));
			norm_mag_component[0]=magfx/mag_mod;
			norm_mag_component[1]=magfy/mag_mod;
			norm_mag_component[2]=magfz/mag_mod;



			for (int i = 0; i < 3; i++) {
        		angle_accel += norm_acc_component[i] * norm_standing_acc_component[i];
				angle_mag+= norm_mag_component[i] * norm_standing_mag_component[i];
    		}

			ESP_LOGI(TAG, "angle_accel = %f", angle_accel); 
			ESP_LOGI(TAG, "angle_mag = %f", angle_mag);
			angle_accel = fabs(acos(angle_accel)*180/(3.14)); 
			angle_mag = fabs(acos(angle_mag)*180/(3.14)); 

			ESP_LOGI(TAG, "angle_accel = %f", angle_accel); 
			ESP_LOGI(TAG, "angle_mag = %f", angle_mag); 

			if((angle_accel > ANGLE_THTRESHOLD)&&(angle_mag   > ANGLE_THTRESHOLD)){
				fall_detected[0] = 1;
				#ifdef BLE_ON
					ble_attr_data_write(fall_detected);
				#endif
			}
			  
		}

		#ifdef TEST_MODE 
			if (fall_detected[0])
			{
				SSD1306_GotoXY(8, 5);
				SSD1306_Puts("       ", &Font_7x10, SSD1306_COLOR_WHITE);
				SSD1306_GotoXY(8, 5);
				SSD1306_Puts("fall detected", &Font_7x10, SSD1306_COLOR_WHITE);
			} else
			{
				SSD1306_GotoXY(8, 5);
				SSD1306_Puts("             ", &Font_7x10, SSD1306_COLOR_WHITE);
				SSD1306_GotoXY(8, 5);
				SSD1306_Puts("NO fall", &Font_7x10, SSD1306_COLOR_WHITE);
			}

			//print in the display
			for (int j = 0; j < 8; j++)
			{
				print[j][14] = '0' + (print_array[j]%10);
    			print[j][13] = '0' + (print_array[j]%100)/10;
    			print[j][12] = '0' + (print_array[j]%1000)/100;
    			print[j][11] = '.';
    			print[j][10] = '0' + (print_array[j]%10000)/1000;
    			print[j][9] = '0' + (print_array[j])/10000;
    			while (print[j][9] == '0' && print[j][10] != '.'){
    				for (int i=9; i<=13;i++){
    					print[j][i] = print[j][i+1];
    				}
    				print[j][14] = ' ';
    			}

				if (j<4)
				{
					SSD1306_GotoXY(8, 20+j*10);
    				SSD1306_Puts(print[j], &Font_7x10, SSD1306_COLOR_WHITE);
				}

	
			}
			SSD1306_UpdateScreen();
			//print to the terminal the magnetic field
			ESP_LOGW(TAG, "MAG measurement");
			ESP_LOGI(TAG, "magx = %f", magfx);
			ESP_LOGI(TAG, "magy = %f", magfy);
			ESP_LOGI(TAG, "magz = %f", magfz);
			ESP_LOGI(TAG, "mag module = %f", mag_mod);

		#endif
		
		//ESP_LOGW(TAG, "ACCEL measurement");
		//ESP_LOGI(TAG, "accx = %f", accelx);
		//ESP_LOGI(TAG, "accy = %f", accely);
		//ESP_LOGI(TAG, "accz = %f", accelz);

    	
    	vTaskDelay(200 / portTICK_PERIOD_MS);  

		
		//the task waits until there is a reset from the client
		#ifdef BLE_ON
			if (fall_detected[0])
			{
				ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    		}
		#endif
	}
		
			
}

/**
 * @brief Main entry point for the ESP32 application.
 *
 * Initializes the system peripherals, sets up the I2C communication, configures display and sensors,
 * initializes BLE functionality if enabled, and starts the fall detection task.
 */
void app_main(void)
{

	/////////////////////////////////////////////
	//peripheral config
	//
	/////////////////////////////////////////////
	data_queue = xQueueCreate(1, sizeof(uint8_t)); 
	ESP_LOGI(TAG, "data queue created");
	button_init();
	ESP_LOGI(TAG, "botton configured");

	//i2c master setup
	ESP_ERROR_CHECK(i2c_master_init());
	
	//Display setup 
    SSD1306_Init();
	vTaskDelay(1000/portTICK_PERIOD_MS);
	SSD1306_Fill(SSD1306_COLOR_BLACK);

	/////////////////////////////////////////////
	//sensor config
	//
	/////////////////////////////////////////////

	//Resetting the offset for the magnetic sensor 
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_OFF_X_MSB_REG,0x00));
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_OFF_X_LSB_REG,0x00));
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_OFF_Y_MSB_REG,0x00));
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_OFF_Y_LSB_REG,0x00));
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_OFF_Z_MSB_REG,0x00));
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_OFF_Z_LSB_REG,0x00));


	//mag3110 configuration 
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_CTRL_REG1_ADDR, MAG3110_CTRL_REG1_VALUE));
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_CTRL_REG2_ADDR, MAG3110_AUTO_MRST_EN));

	//mag3110 offset correction
	ESP_LOGI(TAG, "start mag3110 calibration");
	SSD1306_GotoXY(8, 5);
	SSD1306_Puts("calibrating...", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
	mag3110_calibrating();
	ESP_LOGI(TAG, "mag3110 calibrated");
	SSD1306_GotoXY(8, 5);
	SSD1306_Puts("                  ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_Puts("mag3110 calibrated", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();

	
	//MPU6050 configuration 
    ESP_ERROR_CHECK(mpu6050_register_write_byte (MPU6050_PWR_MGMT_1_REG_ADDR,MPU6050_PWR_MGMT_1_VALUE));
    ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU6050_ACCEL_CONFIG_REG_ADDR, MPU6050_ACCEL_CONFIG_2G_VALUE));

	/////////////////////////////////////////////
	//BLE SETUP
	//
	/////////////////////////////////////////////
	#ifdef BLE_ON
		// Initialize NVS.
		ESP_ERROR_CHECK(nvs_flash_init());	

		//bluetooth config
		esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
		ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
		ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

		//bluedroid config
		//esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
		ESP_ERROR_CHECK(esp_bluedroid_init());
		ESP_ERROR_CHECK( esp_bluedroid_enable());

		//GAP and GATTS event handlers registration
		ESP_ERROR_CHECK(esp_ble_gatts_register_callback(esp_gatts_cb));
		ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));


		ESP_ERROR_CHECK(esp_ble_gap_set_device_name("IOT_SERVER"));
		ESP_ERROR_CHECK(esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT,ESP_PWR_LVL_N12));
		ESP_ERROR_CHECK(esp_ble_gatts_app_register(PROFILE_A_APP_ID));
	
	//esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    //if (local_mtu_ret){
    //    ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    //}
	#endif
	/////////////////////////////////////////////
	//TASK CREATION
	//
	/////////////////////////////////////////////
 	

	xTaskCreate(&fall_detection_task, "fall_detection_task", 1024*10, NULL, TASK_PRIORITY, &fall_detection_task_handle);
	ESP_LOGI(TAG, "SCHEDULER: starting");
}
