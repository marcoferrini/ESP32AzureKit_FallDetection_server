#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/event_groups.h"

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_bt_defs.h"

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

#define GATTS_SERVICE_UUID_A   		0x00FF
#define GATTS_CHAR_UUID_A      		0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4

#define ATTR_VALUE_SIZE 1

const static char *SERVER_TAG = "BLE SERVER:";

extern TaskHandle_t fall_detection_task_handle;

static esp_gatt_char_prop_t a_property = 0;

static uint8_t char1_val[ATTR_VALUE_SIZE]={0x00};
static esp_attr_value_t gatts_char1_val =
{
    .attr_max_len = ATTR_VALUE_SIZE,
    .attr_len     = ATTR_VALUE_SIZE,
    .attr_value   = char1_val,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)


struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void esp_gatts_A_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = esp_gatts_A_cb,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};


static esp_ble_adv_params_t ble_adv_params = {

	.adv_int_min = 0x20,
	.adv_int_max = 0x40,
	.adv_type = ADV_TYPE_IND,
	.own_addr_type  = BLE_ADDR_TYPE_PUBLIC,
	.channel_map = ADV_CHNL_ALL,
	.adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {

	.set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

void ble_attr_data_write(uint8_t* value){

     // Check if newVal pointer is not NULL
    if (value != NULL) {
        ESP_ERROR_CHECK(esp_ble_gatts_set_attr_value(gl_profile_tab[PROFILE_A_APP_ID].char_handle, ATTR_VALUE_SIZE, value));

    }else
	{
		ESP_LOGE(SERVER_TAG,"ble_attr_data_write error: new value pointing to NULL");
	}
	
	return;
}

void ble_attr_data_read( uint8_t* value){
    uint16_t len = 0;
	const uint8_t *char1_val_aux = NULL;
	esp_err_t ret = esp_ble_gatts_get_attr_value(gl_profile_tab[PROFILE_A_APP_ID].char_handle,  &len, &char1_val_aux);

	// Check if the function call was successful and that the pointer is not NULL
	if (ret == ESP_OK && char1_val_aux != NULL && len > 0) {
		*value = *char1_val_aux; // Dereference the pointer safely
	} else {
		// Handle the error: either log it or set default value
    	ESP_LOGE(SERVER_TAG, "Failed to get attribute value, error: %d", ret);
    	*value = 0; // Set a default or error value
	}
	return;
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){	//Funzione chiamata dal driver bluetooth ogni volta che si verifica un evento

	switch (event) {

		case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        	adv_config_done &= (~adv_config_flag);
        	if (adv_config_done == 0){
        	    esp_ble_gap_start_advertising(&ble_adv_params);
        	}
        break;

		case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        	adv_config_done &= (~scan_rsp_config_flag);
        	if (adv_config_done == 0){
            	esp_ble_gap_start_advertising(&ble_adv_params);
        	}
        break;
    
		case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:

			ESP_LOGI(SERVER_TAG,"ESP_GAP_BLE_ADV_START_COMPLETE_EVT\n");

		break;

		case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:

			ESP_LOGI(SERVER_TAG,"ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT\n");

		break;

		default:

			ESP_LOGW(SERVER_TAG,"GAP Event %d unhandled\n\n", event);

		break;
	}
}

static void esp_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }
        else {
            ESP_LOGE(SERVER_TAG,"ESP_GATTS_REG_EVT error: %d \n", param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
	for (int i = 0; i < PROFILE_NUM; i++) {
		if (gl_profile_tab[i].gatts_cb) {
			if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[i].gatts_if) {	/* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
				gl_profile_tab[i].gatts_cb(event, gatts_if, param);
    		}
    	}
	}
}

static void esp_gatts_A_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param){
	switch (event) {
		case ESP_GATTS_REG_EVT:

			ESP_LOGI(SERVER_TAG,"ESP_GATTS_REG_EVT \n");

			gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
			gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
			gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
			gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_A;

			//config adv data
        	esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        	if (ret){
            	ESP_LOGE(SERVER_TAG, "config adv data failed, error code = %x", ret);
        	}
        	adv_config_done |= adv_config_flag;
       	 	//config scan response data
        	ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        	if (ret){
            	ESP_LOGE(SERVER_TAG, "config scan response data failed, error code = %x", ret);
        	}
       	 	adv_config_done |= scan_rsp_config_flag;

			ESP_ERROR_CHECK(esp_ble_gatts_create_service(gatts_if, &(gl_profile_tab[PROFILE_A_APP_ID].service_id), GATTS_NUM_HANDLE_TEST_A));

		break;

		case ESP_GATTS_CREATE_EVT:

			ESP_LOGI(SERVER_TAG,"Service created\n");

			gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
			ESP_ERROR_CHECK(esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle));

		break;

		case ESP_GATTS_START_EVT:{

			ESP_LOGI(SERVER_TAG,"Service started\n");
			ESP_LOGI(SERVER_TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);

			static esp_attr_control_t control = {.auto_rsp = ESP_GATT_AUTO_RSP,};
			a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
			gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
			gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_A;
			ESP_ERROR_CHECK(esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
													ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE, a_property, &gatts_char1_val, &control));
		break;
		}

		case ESP_GATTS_ADD_CHAR_EVT:
		{
			ESP_LOGI(SERVER_TAG,"ESP_GATTS_ADD_CHAR_EVT \n");
			uint16_t len = 0;
			const uint8_t *char1_val_aux = NULL;

			ESP_LOGI(SERVER_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

			gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
			gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        	gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
			ESP_ERROR_CHECK(esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &len, &char1_val_aux));
			if (len!=0){
				ESP_LOGI(SERVER_TAG,"data = ");  
				for (int i = len -1; i >= 0; i--){
					ESP_LOGI(SERVER_TAG,"%02x",char1_val_aux[i]);
				}
				ESP_LOGI(SERVER_TAG,"\n\n");

				ESP_ERROR_CHECK(esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE  , NULL, NULL));

			}
			else ESP_LOGE(SERVER_TAG,"esp_ble_gatts_get_attr_value error\n");

		break;
		}
		case ESP_GATTS_ADD_CHAR_DESCR_EVT:
			gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        	ESP_LOGI(SERVER_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;

	    case ESP_GATTS_CONNECT_EVT:

	    	ESP_LOGI(SERVER_TAG,"ESP_GATTS_CONNECT_EVT \n");

	        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
	        ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());

		break;

	    case ESP_GATTS_READ_EVT:{
			uint16_t len = 0;
			const uint8_t *char1_val_aux = NULL;
			
	    	ESP_LOGI(SERVER_TAG,"ESP_GATTS_READ_EVT \n");
			ESP_ERROR_CHECK(esp_ble_gatts_get_attr_value(gl_profile_tab[PROFILE_A_APP_ID].char_handle,  &len, &char1_val_aux));
	    	//ESP_ERROR_CHECK(esp_ble_gatts_set_attr_value(gl_profile_tab[PROFILE_A_APP_ID].char_handle, ATTR_VALUE_SIZE, char1_val));
	    	ESP_LOGI(SERVER_TAG,"data = ");
	    		for (int i = 0; i <ATTR_VALUE_SIZE ; i++){
	    			ESP_LOGI(SERVER_TAG,"%02x",char1_val_aux[i]);
	    		}
	    		ESP_LOGI(SERVER_TAG,"\n\n");

		break;
	    }

		case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                ESP_LOGI(SERVER_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(SERVER_TAG, param->write.value, param->write.len);
                if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2){ // Handle of the descriptor
					uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
					if (descr_value == 0x0001){
						if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        	ESP_LOGI(SERVER_TAG, "notify enable");
                    	}
					} else if(descr_value == 0x0002)
					{
						if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(SERVER_TAG, "indicate enable");
						}
					} else if (descr_value == 0x0000)
					{
						ESP_LOGI(SERVER_TAG, "notify/indicate disable ");
					} else
					{
						 ESP_LOGE(SERVER_TAG, "unknown descr value");
                    	esp_log_buffer_hex(SERVER_TAG, param->write.value, param->write.len);
					}
					
					
					
				} else
				{
					ble_attr_data_write(param->write.value);
					xTaskNotifyGive(fall_detection_task_handle);
				}
				
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }


            }else{
                ESP_LOGE(SERVER_TAG, "ESP_GATTS_WRITE_EVT unhandled");
            }
      	break;

		case ESP_GATTS_RESPONSE_EVT:
			ESP_LOGI(SERVER_TAG, "ESP_GATTS_RESPONSE_EVT" );
		break;

	    case ESP_GATTS_SET_ATTR_VAL_EVT:

	    	ESP_LOGI(SERVER_TAG,"ESP_GATTS_SET_ATTR_VAL_EVT \n");
			uint8_t notify_value[ATTR_VALUE_SIZE];
			ble_attr_data_read(notify_value);
			
			for (int i = 0; i <ATTR_VALUE_SIZE ; i++){
				ESP_LOGI(SERVER_TAG,"%02x",notify_value[i]);
			}
			//send a notification 
			esp_ble_gatts_send_indicate(gatts_if,gl_profile_tab[PROFILE_A_APP_ID].conn_id,param->set_attr_val.attr_handle,ATTR_VALUE_SIZE, notify_value,false);
			
			
	    break;

		case  ESP_GATTS_CONF_EVT: 
			ESP_LOGI(SERVER_TAG,"ESP_GATTS_SET_ATTR_VAL_EVT \n");
		break;

	    case ESP_GATTS_DISCONNECT_EVT:

	    	ESP_LOGI(SERVER_TAG,"ESP_GATTS_DISCONNECT_EVT \n");

	        ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));

	 	break;


		default:

			ESP_LOGW(SERVER_TAG,"GATT Event %d unhandled\n\n", event);

		break;
	}

}

