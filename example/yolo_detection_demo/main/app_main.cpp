#include "core/algorithm/el_algorithm_yolo.h"
#include "core/engine/el_engine_tflite.h"
#include "core/utils/el_cv.h"
#include "porting/el_device.h"
#include "yolo_model_data.h"
#include <esp_console.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <ctype.h>
#include <sys/time.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl__lvgl/lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/gpio.h"

/* For wifi and sending request over HTTP*/
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_system.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#define EXAMPLE_ESP_WIFI_SSID      "Galaxy A12 4772"
#define EXAMPLE_ESP_WIFI_PASS      "xtfk9410"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10


#define BUTTON_GPIO  GPIO_NUM_2  // Example GPIO pin for the button
#define LONG_PRESS_DELAY 2000  // Long press duration in milliseconds

static const char *TAG = "Meter Reading Model";

#define I2C_HOST  I2C_NUM_0

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           GPIO_NUM_5
#define EXAMPLE_PIN_NUM_SCL           GPIO_NUM_6
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#define EXAMPLE_LCD_H_RES              64
#define EXAMPLE_LCD_V_RES              128
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

typedef enum {
    STATE_SIDE_BY_SIDE,
    STATE_UP_DOWN,
    STATE_SELECTED
} State;


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
	        .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

esp_err_t client_event_post_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
        break;

    default:
        break;
    }
    return ESP_OK;
}

static void post_rest_function(const char* timeStr, const char* tempStr, const char* humidityStr)
{   
    esp_http_client_config_t config_post = {
        .url = "http://192.168.168.106:5000/upload",
        .cert_pem = NULL,
        .method = HTTP_METHOD_POST,
        .event_handler = client_event_post_handler};
        
    esp_http_client_handle_t client = esp_http_client_init(&config_post);

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");

    char json_payload[128];
    snprintf(json_payload, sizeof(json_payload), "{\"time\":\"%s\",\"temperature\":\"%s\",\"humidity\":\"%s\"}",
             timeStr, tempStr, humidityStr);

    char content_length_str[10];
    snprintf(content_length_str, sizeof(content_length_str), "%d", (int)strlen(json_payload));
    esp_http_client_set_header(client, "Content-Length", content_length_str);

    esp_http_client_set_post_field(client, json_payload, strlen(json_payload));
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

void example_lvgl_demo_ui(lv_disp_t *disp, const char *text)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_clean(scr);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(label, text);
    lv_obj_set_width(label, disp->driver->hor_res);
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t * disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}

#define kTensorArenaSize (1024 * 1024)

uint16_t color[] = {
  0x0000,
  0x03E0,
  0x001F,
  0x7FE0,
  0xFFFF,
};

char* get_current_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_t now = tv.tv_sec;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char strftime_buf[64];

    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    
    // Allocate memory for the string and copy the formatted time
    char* current_time_str = (char*)malloc(strlen(strftime_buf) + 1);
    if (current_time_str != NULL) {
        strcpy(current_time_str, strftime_buf);
    }

    return current_time_str;
}

void appendToArr(int arr[][3], int index, int x,int y, int target) {
    arr[index][0] = x;
    arr[index][1] = y;
    arr[index][2] = target;
}

int compareY(const void *a, const void *b) {
    const int *pointA = (const int *)a;
    const int *pointB = (const int *)b;
    return pointA[1] - pointB[1];
}

int compareX(const void *a, const void *b) {
    const int *pointA = (const int *)a;
    const int *pointB = (const int *)b;
    return pointA[0] - pointB[0];
}


extern "C" void app_main() {
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = EXAMPLE_LCD_PIXEL_CLOCK_HZ},
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Install panel IO");

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
        
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v1((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
        .bits_per_pixel = 1,
    };
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh1107(io_handle, &panel_config, &panel_handle));
#endif

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif

    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };

    lv_disp_t * disp = lvgl_port_add_disp(&disp_cfg);
    /* Register done callback for IO */
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, disp);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    ESP_LOGI(TAG, "Display LVGL Scroll Text");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(0)) {
        example_lvgl_demo_ui(disp,"Hello!!! Enter the type of your temperature meter. The 2 options are sidebyside and updown");
        // Release the mutex
        lvgl_port_unlock();
    }

    gpio_reset_pin(BUTTON_GPIO);
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    State current_state = STATE_SIDE_BY_SIDE;
    TickType_t press_start_time = 0;

    char str[20];

    while (1)
    {
        int button_state = gpio_get_level(BUTTON_GPIO);

        if (button_state == 0)
        {
            if (press_start_time == 0)
            {
                // Start timing for long press
                press_start_time = xTaskGetTickCount();
            }

            // Long press checking
            if ((xTaskGetTickCount() - press_start_time) >= pdMS_TO_TICKS(LONG_PRESS_DELAY))
            {
                switch (current_state)
                {
                    case STATE_UP_DOWN:
                        printf("Option chosen: Up Down\n");
                        example_lvgl_demo_ui(disp,"Option chosen: Up Down \n");
                        strcpy(str,"updown");
                        break;
                    case STATE_SIDE_BY_SIDE :
                        printf("Option chosen: Side by Side\n");
                        example_lvgl_demo_ui(disp,"Option chosen: Side by side\n");
                        strcpy(str,"sidebyside");
                        
                        break;
                    default:
                        strcpy(str,"none");
                        break;
                }
                break;  // Exit loop after selecting option
            }
        }
        else
        {
            // Button released
            if (press_start_time > 0)
            {
                // Long press not detected, handle short press (navigation)
                press_start_time = 0;  // Reset timing for next press

                // Navigate between options
                switch (current_state)
                {
                    case STATE_UP_DOWN:
                        current_state = STATE_SIDE_BY_SIDE;
                        printf("Choose option\n > Side by side \n Up Down \n");
                        example_lvgl_demo_ui(disp,"Choose option\n > Side by side\n  Up Down \n");
                        break;
                    case  STATE_SIDE_BY_SIDE:
                        current_state = STATE_UP_DOWN;
                        printf("Choose option\n Side by side \n > Up Down\n");
                        example_lvgl_demo_ui(disp,"Choose option\n  Side by side \n > Up Down \n");
                        break;
                    default:
                        break;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();


    using namespace edgelab;

    Device*  device  = Device::get_device();
    Camera*  camera  = device->get_camera();

    camera->init(240, 240);
   
    auto* engine       = new EngineTFLite();
    auto* newEngine    = new EngineTFLite();
    auto* tensor_arena = heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    engine->init(tensor_arena, kTensorArenaSize);
    engine->load_model(g_yolo_model_data, g_yolo_model_data_len);
    auto* algorithm = new AlgorithmYOLO(engine);
    
    while (true) {
        char* current_time =get_current_time();
        el_printf(current_time);

        el_img_t img;
        camera->start_stream();
        camera->get_frame(&img);
        algorithm->run(&img);
        
        uint8_t  i = 0;
        int arr[5][3] = {0};
        for (const auto& box : algorithm->get_results()) {
            el_printf("\tbox -> cx_cy_w_h: [%d, %d, %d, %d] t: [%d] s: [%d]\n",
                      box.x,
                      box.y,
                      box.w,
                      box.h,
                      box.target,
                      box.score);
            
            if (i<5){
                appendToArr(arr, i, box.x,box.y, box.target);
            }

            int16_t y = box.y - box.h / 2;
            int16_t x = box.x - box.w / 2;
            el_draw_rect(&img, x, y, box.w, box.h, color[++i % 5], 4);
        }

        camera->stop_stream();

        if (i < 5) {
            el_printf("NO detection\n");
        }

        else if (strcmp(str,"sidebyside")==0){
            for (int j = 0; j < 5; j++) {
                for (int k = 0; k < 4 - j; k++) {
                    if (arr[k][0] > arr[k + 1][0]) {
                        int tempX = arr[k][0];
                        int tempTarget = arr[k][1];
                        arr[k][0] = arr[k + 1][0];
                        arr[k][1] = arr[k + 1][1];
                        arr[k + 1][0] = tempX;
                        arr[k + 1][1] = tempTarget;
                    }
                }
            }
            char formattedString[100]; 
            char temp[10];
            char humidity[10];
            sprintf(temp,"%d %d . %d C",arr[0][2],arr[1][2],arr[2][2]);
            sprintf(humidity,"%d %d",arr[3][2], arr[4][2]);
            sprintf(formattedString, "For Side By Side:\n Temperature: %d %d . %d C\n and Humidity: %d %d ",arr[0][2], arr[1][2], arr[2][2], arr[3][2], arr[4][2]);
            example_lvgl_demo_ui(disp,formattedString);
            post_rest_function(current_time,temp,humidity);
        }

        else if (strcmp(str,"updown")==0){
            qsort(arr, sizeof(arr) / sizeof(arr[0]), sizeof(arr[0]), compareY);
            int firstThree[3][3] = {0};
            int rest[2][3] = {0};

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    firstThree[i][j] = arr[i][j];
                }
            }

            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 3; j++) {
                    rest[i][j] = arr[i + 3][j];
                }
            }

            qsort(firstThree, 3, sizeof(firstThree[0]), compareX);
            qsort(rest, 2, sizeof(rest[0]), compareX);

            char formattedString[100]; 
            char temp[10];
            char humidity[10];
            sprintf(temp,"%d %d . %d C",firstThree[0][2], firstThree[1][2], firstThree[2][2]);
            sprintf(humidity,"%d %d ",rest[0][2], rest[1][2]);
            sprintf(formattedString, "For Up Down:\n Temperature: %d %d . %d C \n and Humidity: %d %d %%",firstThree[0][2], firstThree[1][2], firstThree[2][2], rest[0][2], rest[1][2]);
            example_lvgl_demo_ui(disp,formattedString);
            post_rest_function(current_time,temp,humidity);
        }

        else{
            el_printf("Unidentified type of detector");
        }
    el_sleep(2000);
    }

    delete algorithm;
    delete engine;
}
