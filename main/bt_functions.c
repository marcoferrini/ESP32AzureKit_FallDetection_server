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
#define GATTS_NUM_HANDLE_TEST_A     4

#define ATTR_VALUE_SIZE 1

const static char *SERVER_TAG = "BLE SERVER:";

static uint8_t char1_val[ATTR_VALUE_SIZE]={0x00};
static esp_attr_value_t gatts_char1_val =
{
    .attr_max_len = ATTR_VALUE_SIZE,
    .attr_len     = ATTR_VALUE_SIZE,
    .attr_value   = char1_val,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
};

static void esp_gatts_A_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = esp_gatts_A_cb,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
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

	.include_name = true,
	.include_txpower = true,
	.flag = ESP_BLE_ADV_FLAG_LIMIT_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};


void ble_attr_data_write(uint8_t* value){
    // Check if newVal pointer is not NULL
    if (value != NULL) {
        // Copy the new value pointed by newVal into char1_val
        for (int i = 0; i < ATTR_VALUE_SIZE; i++)
        {
            char1_val[i] = value[i];
        }
        
        
    }
	return;
}

void ble_attr_data_read(uint8_t* value){
    // Copy the new value pointed by newVal into char1_val
    for (int i = 0; i < ATTR_VALUE_SIZE; i++)
    {
        value[i] = char1_val[i];
    }
	return;
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){	//Funzione chiamata dal driver bluetooth ogni volta che si verifica un evento

	switch (event) {

		case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:

			ESP_LOGI(SERVER_TAG,"ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT\n");
			ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));

			break;

		case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:

			ESP_LOGI(SERVER_TAG,"ESP_GAP_BLE_ADV_START_COMPLETE_EVT\n");

		break;

		case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:

			ESP_LOGI(SERVER_TAG,"ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT\n");

		break;

		default:

			ESP_LOGI(SERVER_TAG,"GAP Event %d unhandled\n\n", event);

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
            ESP_LOGI(SERVER_TAG,"ESP_GATTS_REG_EVT error: %d \n", param->reg.status);
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

			ESP_ERROR_CHECK(esp_ble_gatts_create_service(gatts_if, &(gl_profile_tab[PROFILE_A_APP_ID].service_id), GATTS_NUM_HANDLE_TEST_A));

		break;

		case ESP_GATTS_CREATE_EVT:

			ESP_LOGI(SERVER_TAG,"Service created\n");

			gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
			ESP_ERROR_CHECK(esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle));

		break;

		case ESP_GATTS_START_EVT:{

			ESP_LOGI(SERVER_TAG,"Service started\n");

			static esp_attr_control_t control = {.auto_rsp = ESP_GATT_AUTO_RSP,};

			gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
			gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_A;
			ESP_ERROR_CHECK(esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
													ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ, &gatts_char1_val, &control));
		break;
		}

		case ESP_GATTS_ADD_CHAR_EVT:
		{
			ESP_LOGI(SERVER_TAG,"ESP_GATTS_ADD_CHAR_EVT \n");
			uint16_t len = 0;
			const uint8_t *char1_val_aux = NULL;

			gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
			ESP_ERROR_CHECK(esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &len, &char1_val_aux));
			if (len!=0){
				ESP_LOGI(SERVER_TAG,"data = ");
				for (int i = len -1; i >= 0; i--){
					ESP_LOGI(SERVER_TAG,"%02x",char1_val_aux[i]);
				}
				ESP_LOGI(SERVER_TAG,"\n\n");

				ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
			}
			else ESP_LOGI(SERVER_TAG,"esp_ble_gatts_get_attr_value error\n");

		break;
		}

	    case ESP_GATTS_CONNECT_EVT:

	    	ESP_LOGI(SERVER_TAG,"ESP_GATTS_CONNECT_EVT \n");

	        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
	        ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());

		break;

	    case ESP_GATTS_READ_EVT:{

	    	ESP_LOGI(SERVER_TAG,"ESP_GATTS_READ_EVT \n");

	    	ESP_ERROR_CHECK(esp_ble_gatts_set_attr_value(gl_profile_tab[PROFILE_A_APP_ID].char_handle, ATTR_VALUE_SIZE, char1_val));
	    	ESP_LOGI(SERVER_TAG,"data = ");
	    		for (int i = 0; i <ATTR_VALUE_SIZE ; i++){
	    			ESP_LOGI(SERVER_TAG,"%02x",char1_val[i]);
	    		}
	    		ESP_LOGI(SERVER_TAG,"\n\n");

		break;
	    }

	    case ESP_GATTS_SET_ATTR_VAL_EVT:

	    	ESP_LOGI(SERVER_TAG,"ESP_GATTS_SET_ATTR_VAL_EVT \n");

	    break;

	    case ESP_GATTS_DISCONNECT_EVT:

	    	ESP_LOGI(SERVER_TAG,"ESP_GATTS_DISCONNECT_EVT \n");

	        ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));

	 	break;


		default:

			ESP_LOGI(SERVER_TAG,"GATT Event %d unhandled\n\n", event);

		break;
	}

}

