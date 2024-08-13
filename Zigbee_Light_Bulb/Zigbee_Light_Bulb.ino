/*
Based on the Arduino IDE ESP32 Example for Zigbee Light Bulb
Used sources:
  - https://github.com/espressif/esp-zigbee-sdk/tree/main/examples/esp_zigbee_customized_devices/customized_server
  - https://github.com/espressif/esp-idf/issues/10662

In Arduino IDE check for latest version of Board library ESP32. Check that the correct board was selected.
Used settings:
Board: ESP32C6 Dev Module
Partition Scheme: Zigbee 4MB with spiffs
Zigbee Mode: Zigbee ED

Irrelevant options set on my side:
USB CDC On Board: Disabled (default)
CPU Frequency: 160MHz (default)
Core Debug Level: Warn
Erase All Flash Before Sketch Upload: Enabled
Flash Frequency: 80MHz
Flash Mode: QIO
Flash Size 4MB
JTAG Adapter: Disabled
Upload Speed 921600

---

Code is not great, but works as a PoC
*/


#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

#define LED_PIN RGB_BUILTIN

/* Default End Device config */
#define ESP_ZB_ZED_CONFIG()                                                                 \
  {                                                                                         \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
    .nwk_cfg = {                                                                            \
      .zed_cfg =                                                                            \
        {                                                                                   \
          .ed_timeout = ED_AGING_TIMEOUT,                                                   \
          .keep_alive = ED_KEEP_ALIVE,                                                      \
        },                                                                                  \
    },                                                                                      \
  }

#define ESP_ZB_DEFAULT_RADIO_CONFIG() \
  { .radio_mode = ZB_RADIO_MODE_NATIVE, }

#define ESP_ZB_DEFAULT_HOST_CONFIG() \
  { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, }

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE   false /* enable the install code policy for security */
#define ED_AGING_TIMEOUT            ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE               3000                                 /* 3000 millisecond */
#define HA_ESP_LIGHT_ENDPOINT       10                                   /* esp light bulb device endpoint, used to process light controlling commands */
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */
#define ESP_ZB_ZCL_CLUSTER_ID_LEVEL 0x8
#define ESP_ZB_ZCL_ATTR_LEVEL_LEVEL_ID 0x0

/********************* Zigbee functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
  uint32_t *p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
  switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      Serial.printf("[INFO] Zigbee stack initialized\n");
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
      break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      if (err_status == ESP_OK) {
        Serial.printf("[INFO] Device started up in %s factory-reset mode\n", esp_zb_bdb_is_factory_new() ? "" : "non");
        if (esp_zb_bdb_is_factory_new()) {
          Serial.printf("[INFO] Start network formation\n");
          esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
          Serial.printf("[INFO] Device rebooted\n");
        }
      } else {
        /* commissioning failed */
        Serial.printf("[WARN] Failed to initialize Zigbee stack (status: %s)\n", esp_err_to_name(err_status));
      }
      break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
      if (err_status == ESP_OK) {
        esp_zb_ieee_addr_t extended_pan_id;
        esp_zb_get_extended_pan_id(extended_pan_id);
        Serial.printf("[INFO] Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)\n",
          extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4], extended_pan_id[3], extended_pan_id[2], extended_pan_id[1],
          extended_pan_id[0], esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address()
        );
      } else {
        Serial.printf("[INFO] Network steering was not successful (status: %s)\n", esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
      }
      break;
    default: Serial.printf("[INFO] ZDO signal: %s (0x%x), status: %s\n", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status)); break;
  }
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
  esp_err_t ret = ESP_OK;
  switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID: ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message); break;
    default:                               log_w("Receive Zigbee action(0x%x) callback", callback_id); break;
  }
  return ret;
}

static void esp_zb_task(void *pvParameters) {
  esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
  esp_zb_init(&zb_nwk_cfg);
  
  /* set the on-off light device config */
  char modelid[] = {13, 'E', 'S', 'P', '3', '2', 'C', '6', '.', 'L', 'i', 'g', 'h', 't'};
  char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};
  uint8_t test_attr, test_attr2;

  test_attr = 0;
  test_attr2 = 4;
  /* basic cluster create with fully customized */
  esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &test_attr2);
  esp_zb_cluster_update_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr2);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, &modelid[0]);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, &manufname[0]);
  /* identify cluster create with fully customized */
  esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
  esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &test_attr);
  /* group cluster create with fully customized */
  esp_zb_attribute_list_t *esp_zb_groups_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_GROUPS);
  esp_zb_groups_cluster_add_attr(esp_zb_groups_cluster, ESP_ZB_ZCL_ATTR_GROUPS_NAME_SUPPORT_ID, &test_attr);
  /* scenes cluster create with standard cluster + customized */
  esp_zb_attribute_list_t *esp_zb_scenes_cluster = esp_zb_scenes_cluster_create(NULL);
  esp_zb_cluster_update_attr(esp_zb_scenes_cluster, ESP_ZB_ZCL_ATTR_SCENES_NAME_SUPPORT_ID, &test_attr);

  /* color dimmable light cluster */
  esp_zb_color_dimmable_light_cfg_t light_cfg = ESP_ZB_DEFAULT_COLOR_DIMMABLE_LIGHT_CONFIG();
  esp_zb_cluster_list_t *esp_zb_light_cluster_list = esp_zb_color_dimmable_light_clusters_create(&light_cfg);
  esp_zb_cluster_list_update_basic_cluster(esp_zb_light_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
  esp_zb_endpoint_config_t endpoint_config = {
    .endpoint = HA_ESP_LIGHT_ENDPOINT,
    .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
    .app_device_id = ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID,
    .app_device_version = 0
  };
  //esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config1);
  esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_light_cluster_list, endpoint_config);
  esp_zb_device_register(esp_zb_ep_list);
  esp_zb_core_action_handler_register(zb_action_handler);
  esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
  //esp_zb_device_add_set_attr_value_cb(attr_cb);

  esp_zb_nvram_erase_at_start(true);

  ESP_ERROR_CHECK(esp_zb_start(false));
  esp_zb_main_loop_iteration();
}

/* Handle the light attribute */

bool led_on = false;
uint8_t led_brightness = 0;
bool led_color_type_x = false;
bool led_color_type_y = false;
bool led_color_xy_updated_x = true;
bool led_color_xy_updated_y = true;
int32_t led_color_x = 45914;
int32_t led_color_y = 19615;
double led_color_r = 1;
double led_color_g = 1;
double led_color_b = 1;
const double xy_mul_factor = 0.00001525902;

static void set_neopixel() {
  neopixelWrite(LED_PIN, (uint8_t)(led_color_r * led_brightness * led_on), (uint8_t)(led_color_g * led_brightness * led_on), (uint8_t)(led_color_b * led_brightness * led_on));
}

static double getReversedGammaCorrectedValue(double val) {
  return val <= 0.0031308 ? 12.92 * val : (1.0 + 0.055) * pow(val, (1.0 / 2.4)) - 0.055;
}

static void update_color() {
  Serial.printf("[DEBG] XY raw: %d/%d\n", led_color_x, led_color_y);
  double x = led_color_x*xy_mul_factor;
  double y = led_color_y*xy_mul_factor;
  Serial.printf("[DEBG] XY norm: %lf/%lf\n", x, y);

  double z = 1.0 - x - y;
  double Y = led_brightness / 255.0;
  double X = (Y / y) * x;
  double Z = (Y / y) * z;
  double r = X * 1.656492 - Y * 0.354851 - Z * 0.255038;
  double g = -X * 0.707196 + Y * 1.655397 + Z * 0.036152;
  double b = X * 0.051713 - Y * 0.121364 + Z * 1.011530;
  Serial.printf("[DEBG] RGB raw: %lf/%lf/%lf\n", r, g, b);

  r = getReversedGammaCorrectedValue(r);
  g = getReversedGammaCorrectedValue(g);
  b = getReversedGammaCorrectedValue(b);
  Serial.printf("[DEBG] RGB Gamma: %lf/%lf/%lf\n", r, g, b);

  r = r < 0 ? 0 : r;
  g = g < 0 ? 0 : g;
  b = b < 0 ? 0 : b;
  Serial.printf("[DEBG] RGB Norm < 0, %lf, %lf, %lf\n", r, g, b);

  double max_rg = r > g ? r : g;
  double max = max_rg > b ? max_rg : b;
  if (max > 1) {
      r = r / max;
      g = g / max;
      b = b / max;
  }
  Serial.printf("[DEBG] RGB calculated, %lf, %lf, %lf\n", r, g, b);

  led_color_r = r;
  led_color_g = g;
  led_color_b = b;
  set_neopixel();
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message) {
  esp_err_t ret = ESP_OK;
  bool light_state = 0;

  if (!message) {
    Serial.printf("[ERRO] Empty message\n");
  }
  if (message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
    Serial.printf("[ERRO] Received message: error status(%d)\n", message->info.status);
  }

  Serial.printf("[INFO] Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)\n", message->info.dst_endpoint, message->info.cluster, message->attribute.id,
    message->attribute.data.size
  );

  if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT) {
    switch(message->info.cluster) {
      case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
          led_on = message->attribute.data.value ? *(bool *)message->attribute.data.value : led_on;
          Serial.printf("[INFO] Light sets to %s\n", led_on ? "On" : "Off");
          update_color();
        } else {
          Serial.printf("[WARN] Somethings wrong with ESP_ZB_ZCL_CLUSTER_ID_ON_OFF\n");
        }
        break;

      case ESP_ZB_ZCL_CLUSTER_ID_LEVEL:
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
          led_brightness = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : led_brightness;
          Serial.printf("[INFO] Brightness set to %d\n", led_brightness);
          update_color();
        } else {
          Serial.printf("[WARN] Somethings wrong with ESP_ZB_ZCL_CLUSTER_ID_LEVEL\n");
          Serial.printf("[DEBG] Attribute ID %d\n", message->attribute.id);
          Serial.printf("[DEBG] Attribute data type %d\n", (int)message->attribute.data.type);
        }
        break;

      case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
        led_color_type_x = false;
        led_color_type_y = false;
        if(message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID) { led_color_type_x = true; }
        if(message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID) { led_color_type_y = true; }
        
        if(!led_color_xy_updated_x && !led_color_xy_updated_y && led_color_type_x && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
          led_color_x = *(uint16_t *)message->attribute.data.value;
          led_color_xy_updated_x = true;
          Serial.printf("[INFO] Color X set to %d\n", led_color_x);
        }
        // Need led_color_xy_updated_x, because require X first, then Y
        if(led_color_xy_updated_x && !led_color_xy_updated_y && led_color_type_y && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
          led_color_y = *(uint16_t *)message->attribute.data.value;
          led_color_xy_updated_y = true;
          Serial.printf("[INFO] Color Y set to %d\n", led_color_y);
        }
        if(led_color_xy_updated_x && led_color_xy_updated_y) {
          led_color_xy_updated_x = false;
          led_color_xy_updated_y = false;
          Serial.printf("[DEBG] New XY, calculate color\n");
          update_color();
        }
        break;

      default:
        Serial.printf("[WARN] Unknown cluster\n");
        break;
    }
  }
  return ret;
}

/********************* Arduino functions **************************/
void setup() {
  Serial.begin(115200);
  // Init Zigbee
  esp_zb_platform_config_t config = {
    .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
    .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
  };
  ESP_ERROR_CHECK(esp_zb_platform_config(&config));

  // Init RMT and leave light OFF
  neopixelWrite(LED_PIN, 20, 0, 0);

  // Start Zigbee task
  xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}

void loop() {
  //empty, zigbee running in task
}


