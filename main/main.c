#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "xi2c.h"
#include "fonts.h"
#include "ssd1306.h"

#include <math.h>

//ATTENZIONE in questo modo mostro i moduli delle componenti delle accelerazioni sui tre assi 


//TODO aggiungere un bottonwe e piu schermate nel display 
//TODO non ha molto senso usare sia accelx che magfx, potrei usare un'unica variabile per entrambi i campi 
//TODO al momento leggo i dati dal magnetometro con un frequenza molto maggiore di quella necessaria, questo mi fa perdere in efficienza energetica 
//TODO potrei leggere le componenti del campo magnetico con un unica lettura consecutiva, lo stesso potrebbe essere fatto anche per l'accelerometro
//TODO imparare ad utiliazzare il bottone per switchare fra le due pagine del display 
//TODO la calibrazione andrebbe fatta ogni tot secondi un possibile modo per implementare questo comportamento è un nested for loop 
//o forse ha senso anche fare la calibrazione ad ogni ciclo for quando si è sicuri che non si è avuta una caduta, pero' visto che l'accelerometro ha un certo ritardo a rilevare la caduta è meglio aggiornare la direzione di g standing una volta ogni tanto
//TODO per il debug potresti fare usare un bottone per mettere a zero le componenti dell'accelerazione


const static char *TAG = "FALL_DETECTION:";

//Button parameters
#define BUTTON 						0
#define BUTTON_PIN	GPIO_NUM_2
#define TASK_PRIORITY				5


//i2c parameters 
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

//Angle threshold to detect a fall
#define ANGLE_THTRESHOLD					0.785   //in radians = pi/4


#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B
#define MPU6050_PWR_MGMT_1_VALUE 			0x00

#define MPU6050_ACCEL_CONFIG_REG_ADDR     	0x1C
#define MPU6050_ACCEL_CONFIG_2G_VALUE		0x00


//MAGNETOMETER registers and values
#define MAG3110_SENSOR_ADDR					0x0E  //the problem is with the address of this sensor or more in general with the sensor itself
#define MAG3110_CTRL_REG1_ADDR				0x10
#define MAG3110_CTRL_REG2_ADDR				0x11

#define MAG3110_DR_STATUS_ADDR              0x00
#define MAG3110_OUT_X_MSB_ADDR              0x01
#define MAG3110_OUT_X_LSB_ADDR              0x02
#define MAG3110_OUT_Y_MSB_ADDR              0x03
#define MAG3110_OUT_Y_LSB_ADDR              0x04
#define MAG3110_OUT_Z_MSB_ADDR              0x05
#define MAG3110_OUT_Z_LSB_ADDR              0x06

#define MAG3110_AUTO_MRST_EN				0x80    //auto_mrst_en e raw data(no offset)
#define MAG3110_CTRL_REG1_VALUE				0x01
//Parameters
#define STEP_ACCEL_THRESHOLD				0.9  //In g

#define ACCEL_THRESHOLD						0.3


//SemaphoreHandle_t xButtonSemaphore;



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

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}



//ACCELERATION reading function
void mpu6050_accel_read(double *accelx, double *accely, double *accelz) {
	uint8_t data[2];
    data[0] = 0;
    data[1] = 0;
	int16_t accel_aux;


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
}

void mpu6050_mag_read(double *magfx, double *magfy, double *magfz){

	uint8_t data[2];
    data[0] = 0;
    data[1] = 0;
	int16_t magf_aux;

	ESP_ERROR_CHECK(mag3110_register_read(MAG3110_OUT_X_MSB_ADDR, data, 2));
    magf_aux = (((data[0] <<8)) | data[1]);
    *magfx = magf_aux;
    *magfx = *magfx*0.1;  //to get the field in uT
    ESP_ERROR_CHECK(mag3110_register_read(MAG3110_OUT_Y_MSB_ADDR, data, 2));
    magf_aux = (((data[0] <<8)) | data[1]);
    *magfy = magf_aux;
    *magfy = *magfy*0.1;
    ESP_ERROR_CHECK(mag3110_register_read(MAG3110_OUT_Z_MSB_ADDR, data, 2));
    magf_aux = (((data[0] <<8)) | data[1]);
    *magfz = magf_aux;
    *magfz = *magfz*0.1;
}

//void button_isr_handler(void* arg) {
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//
//    // Notify the task that the button was pressed
//    xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);
//
//    if (xHigherPriorityTaskWoken == pdTRUE) {
//        portYIELD_FROM_ISR();
//    }
//}
//
//
//
//void configure_button(){
//	gpio_config_t io_conf = {
//        .pin_bit_mask = (1ULL << BUTTON_PIN),
//        .mode = GPIO_MODE_INPUT,
//        .intr_type = GPIO_INTR_ANYEDGE,
//        .pull_up_en = GPIO_PULLUP_ENABLE,
//    };
//    gpio_config(&io_conf);
//
//    // Install ISR Service with default configuration
//    gpio_install_isr_service(0);
//
//    // Hook ISR handler for specific GPIO pin
//    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, (void*) BUTTON_PIN);
//}

void app_main(void)
{
	double accelx;
	double accely;
	double accelz;

	double magfx;
	double magfy;
	double magfz;

	double old_accelx = 0;
	double old_accely = 0;
	double old_accelz = 0;
	char print[8][16] = { "|Acc|  =   .    ",
						"x-axis =   .    ",
						"y-axis =   .    ",
						"z-axis =   .    ",
						"|Magf|  =   .   ",
						"x-axis =   .    ",
						"y-axis =   .    ",
						"z-axis =   .    "
	};
	int print_array[8];
	double step_accel_mod;
	double accel_mod;

	double norm_standing_accel_component[3]; //normalized vector components
	double norm_accel_component[3];
	double angle = 0;

	int16_t step_accelx;
	int16_t step_accely;
	int16_t step_accelz;

	//// Create a semaphore to notify the task when the button is pressed
   //xButtonSemaphore = xSemaphoreCreateBinary();

	//// Configure the button and set up the ISR
   //configure_button();

	//// Create a task to handle the button press
   //xTaskCreate(&button_task, "button_task", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY, NULL);

   //// Start the FreeRTOS scheduler
   //vTaskStartScheduler();

	
    ESP_ERROR_CHECK(i2c_master_init());
    SSD1306_Init();

	ESP_LOGW(TAG, "Arrived befor the for loop");
	vTaskDelay(1000/portTICK_PERIOD_MS);	

	//MAG3110 configuration
	
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_CTRL_REG1_ADDR, MAG3110_CTRL_REG1_VALUE));
	ESP_ERROR_CHECK(mag3110_register_write_byte(MAG3110_CTRL_REG2_ADDR, MAG3110_AUTO_MRST_EN));

	
	//MPU6050 configuration 
    ESP_ERROR_CHECK(mpu6050_register_write_byte (MPU6050_PWR_MGMT_1_REG_ADDR,MPU6050_PWR_MGMT_1_VALUE));
    ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU6050_ACCEL_CONFIG_REG_ADDR, MPU6050_ACCEL_CONFIG_2G_VALUE));


	//Fist reading to calibrate the sensor and understand how it is placed 
	mpu6050_accel_read(&accelx, &accely, &accelz);
	accel_mod = sqrt(pow(accelx,2) + pow(accely,2) + pow(accelz,2));

	norm_standing_accel_component[0] = accelx/accel_mod;
	norm_standing_accel_component[1] = accely/accel_mod;
	norm_standing_accel_component[2] = accelz/accel_mod;

	
    for(;;){ 

		//compute acceleration module and absolute value of its components and the derivative of the module
    	mpu6050_accel_read(&accelx, &accely, &accelz);

		step_accelx = (accelx - old_accelx);
		step_accely = (accely - old_accely);
		step_accelz = (accelz - old_accelz);   
		print_array[1] = abs(1000*accelx); 
		print_array[2] = abs(1000*accely);
		print_array[3] = abs(1000*accelz);

		//accel module
    	print_array[0] = 1000*(sqrt(pow(accelx,2) + pow(accely,2) + pow(accelz,2)));  

		//accel derivate module
		step_accel_mod = sqrt(pow(step_accelx,2) + pow(step_accely,2) + pow(step_accelz,2));

		//save the old values
		old_accelx = accelx;
		old_accely = accely;
		old_accelz = accelz;

		//compute acceleration module and absolute value of its components and the derivative of the module
		mpu6050_mag_read(&magfx, &magfy, &magfz);
		print_array[5] = abs(magfx); 
		print_array[6] = abs(magfy);
		print_array[7] = abs(magfz);

		//magnetic field module 
		accel_mod= sqrt(pow(magfx,2)+pow(magfy,2)+pow(magfz,2));
		print_array[4] = accel_mod;
		  
		SSD1306_Fill(SSD1306_COLOR_BLACK);

		if((step_accel_mod>STEP_ACCEL_THRESHOLD)&&(accel_mod < ACCEL_THRESHOLD)) {
			
			//wait for the transient
			vTaskDelay(1000 / portTICK_PERIOD_MS);

			//read acceleration once again 
			mpu6050_accel_read(&accelx, &accely, &accelz);
			accel_mod = sqrt(pow(accelx,2) + pow(accely,2) + pow(accelz,2));
			norm_accel_component[0]=accelx/accel_mod;
			norm_accel_component[1]=accely/accel_mod;
			norm_accel_component[2]=accelz/accel_mod;

			for (int i = 0; i < 3; i++) {
        		angle += norm_accel_component[i] * norm_standing_accel_component[i];
    		}

			if(angle > ANGLE_THTRESHOLD){
				SSD1306_GotoXY(8, 5);
    			SSD1306_Puts("fall detected", &Font_7x10, SSD1306_COLOR_WHITE);
			} else
			{
				SSD1306_GotoXY(8, 5);
    			SSD1306_Puts("NO fall", &Font_7x10, SSD1306_COLOR_WHITE);
			}
			
			//compare the angle with a threshold 
			//do the same thing with the magnetic field
			  
		} else {
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
    			SSD1306_Puts(print[j+BUTTON*4], &Font_7x10, SSD1306_COLOR_WHITE);
			}
			
    		
		}
		//print in the terminal
		ESP_LOGW(TAG, "%s",print[0]);
		
    	SSD1306_UpdateScreen();
    	vTaskDelay(1000 / portTICK_PERIOD_MS);  
    }
}
