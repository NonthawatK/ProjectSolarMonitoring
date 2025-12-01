#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "cJSON.h"

// ตัวแปร global
int x = 0;
int y = 0;
int z = 0;

int count_failed = 0;

// Event Group
#define TEMP_OK_BIT BIT0      // อ่านอุณหภูมิสำเร็จ
#define TIME_OK_BIT BIT1      // อ่านเวลา DS3231 สำเร็จ
#define READY_DELAY_BIT BIT2  // ดีเลย์ครบ 5 วิ
#define SEND_SUCCESS_BIT BIT3 // ESP-NOW ส่งสำเร็จ

// I2C config
#define I2C_MASTER_PORT 0
#define I2C_MASTER_SDA_IO 6
#define I2C_MASTER_SCL_IO 7
#define I2C_MASTER_FREQ_HZ 100000

#define HTU21D_ADDR 0x40
#define DS3231_ADDR 0x68

#define CMD_TEMP 0xE3
#define CMD_HUM 0xE5

#define DELAY_MINUTE_MS 100
#define SLEEP_MINUTES 60

#define BUTTON_GPIO 21

// WIFI AP
#define WIFI_SSID "DS3231_Setup"
#define WIFI_PASS "12345678"

volatile bool send_done_success = false;
bool is_configured = false;
volatile bool web_mode = false;

static const char *TAG = "WEB";

EventGroupHandle_t send_event_group;
EventGroupHandle_t data_event_group;

i2c_master_bus_handle_t bus;
i2c_master_dev_handle_t htu_dev;
i2c_master_dev_handle_t ds3231_dev;

static SemaphoreHandle_t xDataMutex;
static float g_temp = 0.0f;                                  // เก็บค่าอุณหภูมิ
static uint8_t g_hour, g_min, g_sec, g_day, g_month, g_year; // เก็บเวลา

// MAC ของตัวรับ ESP-NOW
uint8_t receiver_mac[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t bcd_to_dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

uint8_t dec_to_bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

// NVS ฟังก์ชันสำหรับเก็บสถานะการตั้งค่า
void check_nvs_flag()
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS");
        return;
    }
    uint8_t value = 0;
    err = nvs_get_u8(nvs, "configured", &value);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGW(TAG, "No config found, first time boot");
        is_configured = false;
    }
    else if (err == ESP_OK)
    {
        is_configured = (value == 1);
        ESP_LOGI(TAG, "Configured flag: %d", is_configured);
    }
    nvs_close(nvs);
}

// ฟังก์ชันบันทึก MAC ลง NVS
void save_mac_nvs(uint8_t *mac)
{
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs));
    ESP_ERROR_CHECK(nvs_set_blob(nvs, "mac", mac, 6));
    ESP_ERROR_CHECK(nvs_commit(nvs));
    nvs_close(nvs);
}

// ฟังก์ชันโหลด MAC จาก NVS
void load_mac_nvs()
{
    nvs_handle_t nvs;
    esp_err_t open_err = nvs_open("storage", NVS_READWRITE, &nvs);
    if (open_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS for MAC load (%s)", esp_err_to_name(open_err));
        return;
    }
    size_t required_size = 6;
    esp_err_t err = nvs_get_blob(nvs, "mac", receiver_mac, &required_size);
    switch (err)
    {
    case ESP_OK:
    {
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 receiver_mac[0], receiver_mac[1], receiver_mac[2],
                 receiver_mac[3], receiver_mac[4], receiver_mac[5]);
        ESP_LOGI(TAG, "Loaded MAC from NVS: %s", mac_str);
        break;
    }
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGW(TAG, "No MAC found in NVS. Using default: %02X:%02X:%02X:%02X:%02X:%02X",
                 receiver_mac[0], receiver_mac[1], receiver_mac[2],
                 receiver_mac[3], receiver_mac[4], receiver_mac[5]);
        break;
    default:
        ESP_LOGE(TAG, "Error reading MAC from NVS (%s)", esp_err_to_name(err));
        break;
    }
    nvs_close(nvs);
}

// บันทึก flag ว่าตั้งค่าเสร็จแล้ว
void save_nvs_flag()
{
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs));
    ESP_ERROR_CHECK(nvs_set_u8(nvs, "configured", 1));
    ESP_ERROR_CHECK(nvs_commit(nvs));
    nvs_close(nvs);
}

// DS3231 ฟังก์ชันอ่าน/ตั้งเวลา
void ds3231_set_time(i2c_master_dev_handle_t dev, uint8_t hour, uint8_t min, uint8_t sec,
                     uint8_t day, uint8_t month, uint8_t year)
{
    uint8_t data[8];
    data[0] = 0x00;
    data[1] = dec_to_bcd(sec);  // วินาที
    data[2] = dec_to_bcd(min);  // นาที
    data[3] = dec_to_bcd(hour); // ชั่วโมง
    data[4] = 1;
    data[5] = dec_to_bcd(day);   // วัน
    data[6] = dec_to_bcd(month); // เดือน
    data[7] = dec_to_bcd(year);  // ปี
    ESP_ERROR_CHECK(i2c_master_transmit(dev, data, 8, 1000));
}

// อ่าน DS3231
void read_ds3231(uint8_t *hour, uint8_t *min, uint8_t *sec,
                 uint8_t *day, uint8_t *month, uint8_t *year)
{
    uint8_t reg = 0x00;
    uint8_t data[7];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(ds3231_dev, &reg, 1, data, 7, 1000));
    *hour = bcd_to_dec(data[2]);
    *min = bcd_to_dec(data[1]);
    *sec = bcd_to_dec(data[0]);
    *day = bcd_to_dec(data[4]);
    *month = bcd_to_dec(data[5] & 0x1F);
    *year = bcd_to_dec(data[6]);
}

// อ่าน DS3231 และเช็ค
bool read_ds3231_check(uint8_t *hour, uint8_t *min, uint8_t *sec,
                       uint8_t *day, uint8_t *month, uint8_t *year)
{
    uint8_t reg = 0x00;
    uint8_t data[7];
    if (i2c_master_transmit_receive(ds3231_dev, &reg, 1, data, 7, 1000) != ESP_OK)
    {
        return false;
    }
    *sec = bcd_to_dec(data[0]);
    *min = bcd_to_dec(data[1]);
    *hour = bcd_to_dec(data[2]);
    *day = bcd_to_dec(data[4]);
    *month = bcd_to_dec(data[5] & 0x1F);
    *year = bcd_to_dec(data[6]);
    if (*year == 0 || *month == 0 || *day == 0)
    {
        ESP_LOGI("DS3231", "faill");
        return false;
    }
    if (*year == 0 && *month == 1 && *day == 1)
    {
        ESP_LOGI("DS3231", "faill");
        return false;
    }
    return true;
}

// HTU21D ฟังก์ชันอ่านอุณหภูมิ
bool read_temp(float *temp_out)
{
    uint8_t cmd = CMD_TEMP;
    uint8_t data[3];
    if (i2c_master_transmit(htu_dev, &cmd, 1, 1000) != ESP_OK)
    {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    if (i2c_master_receive(htu_dev, data, 3, 1000) != ESP_OK)
    {
        return false;
    }

    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
    raw &= 0xFFFC;
    float temp = -46.85 + 175.72 * (float)raw / 65536.0;

    if (temp > -50 && temp < 100)
    {
        *temp_out = temp;

        return true;
    }
    ESP_LOGI("HTU21D", "faill");
    return false;
}

// ฟังก์ชัน init I2C
void i2c_init()
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));

    // Add HTU21D device
    i2c_device_config_t htu_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = HTU21D_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &htu_cfg, &htu_dev));

    // Add DS3231 device
    i2c_device_config_t ds_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DS3231_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &ds_cfg, &ds3231_dev));
}

// WiFi & AP สำหรับเปิดเว็บ
void wifi_init_ap(void)
{
    esp_wifi_disconnect();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
    if (strlen(WIFI_PASS) == 0)
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();
    esp_wifi_set_max_tx_power(78);
    ESP_LOGI(TAG, "AP Started. SSID:%s Password:%s", WIFI_SSID, WIFI_PASS);
}

// SPIFFS
void spiffs_init(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    size_t total = 0, used = 0;
    ESP_ERROR_CHECK(esp_spiffs_info(NULL, &total, &used));
    ESP_LOGI(TAG, "SPIFFS mounted. Total: %d, Used: %d", total, used);
}

// HTTP Handlers
static esp_err_t root_get_handler(httpd_req_t *req)
{
    FILE *f = fopen("/spiffs/index.html", "r");
    if (!f)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    fseek(f, 0, SEEK_END);
    size_t len = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = malloc(len + 1);
    if (!buf)
    {
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    fread(buf, 1, len, f);
    buf[len] = '\0';
    fclose(f);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, buf);
    free(buf);
    return ESP_OK;
}

// ESP-NOW
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI("ESP-NOW", "Send success"); // ส่งสำเร็จ
        xEventGroupSetBits(send_event_group, SEND_SUCCESS_BIT);
    }
    else
    {
        ESP_LOGI("ESP-NOW", "Failled"); // ส่งล้มเหลว
    }
}

//   ESP-NOW: โหมดตัวส่ง
void espnow_init_sender()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());

    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false};
    memcpy(peer.peer_addr, receiver_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
}

// Web POST handler
static esp_err_t save_post_handler(httpd_req_t *req)
{
    char buf[512];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // อ่านเวลา
    cJSON *jday = cJSON_GetObjectItem(root, "day");
    cJSON *jmonth = cJSON_GetObjectItem(root, "month");
    cJSON *jyear = cJSON_GetObjectItem(root, "year");
    cJSON *jhour = cJSON_GetObjectItem(root, "hour");
    cJSON *jminute = cJSON_GetObjectItem(root, "minute");

    if (jday && jmonth && jyear && jhour && jminute)
    {
        ds3231_set_time(ds3231_dev, jhour->valueint, jminute->valueint, 0,
                        jday->valueint, jmonth->valueint, jyear->valueint);
        ESP_LOGI(TAG, "Time updated");
    }

    // อ่าน MAC ใหม่
    cJSON *jmac = cJSON_GetObjectItem(root, "mac");
    if (jmac && jmac->valuestring)
    {
        uint8_t new_mac[6];
        sscanf(jmac->valuestring, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &new_mac[0], &new_mac[1], &new_mac[2],
               &new_mac[3], &new_mac[4], &new_mac[5]);
        ESP_LOGI(TAG, "ESP-NOW updated: %02X:%02X:%02X:%02X:%02X:%02X",
                 new_mac[0], new_mac[1], new_mac[2], new_mac[3], new_mac[4], new_mac[5]);
        save_mac_nvs(new_mac); // เซฟลง NVS
    }

    save_nvs_flag(); // บันทึกว่าเซ็ตเวลาแล้ว
    cJSON_Delete(root);
    httpd_resp_sendstr(req, "Saved successfully!");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart(); // รีสตาร์ทเครื่อง
    return ESP_OK;
}

//   เริ่ม Web Server
void start_webserver()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    config.stack_size = 8192;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t root_uri = {.uri = "/", .method = HTTP_GET, .handler = root_get_handler};
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t set_time_uri = {.uri = "/set_time", .method = HTTP_POST, .handler = save_post_handler};
        httpd_register_uri_handler(server, &set_time_uri);

        httpd_uri_t save_uri = {.uri = "/save", .method = HTTP_POST, .handler = save_post_handler};
        httpd_register_uri_handler(server, &save_uri);
    }
}

// เช็คปุ่มเพื่อเข้าหน้า Web Server
void BUTTON(void *arg)
{
    int last = 0;
    while (1)
    {
        int level = gpio_get_level(BUTTON_GPIO);
        if (level && !last)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (gpio_get_level(BUTTON_GPIO))
            {
                ESP_LOGI(TAG, "Button pressed: start config AP");
                web_mode = true;
                esp_wifi_stop();
                esp_wifi_deinit();
                wifi_init_ap();
                spiffs_init();
                start_webserver();
            }
        }
        last = level;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
// อ่านอุณหภูมิ
void TEMP(void *arg)
{
    while (1)
    {
        if (xSemaphoreTake(xDataMutex, 0))
        {
            bool ok = read_temp(&g_temp);
            if (ok)
            {
                if (y == 0)
                {
                    xEventGroupSetBits(data_event_group, TEMP_OK_BIT);
                    y = 1;
                }
            }
            else
            {
                ESP_LOGI("Temp", "READ FAILL");
            }
            xSemaphoreGive(xDataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// อ่านเวลา
void TIME(void *arg)
{
    while (1)
    {
        if (xSemaphoreTake(xDataMutex, 0))
        {
            bool ok = read_ds3231_check(&g_hour, &g_min, &g_sec, &g_day, &g_month, &g_year);
            if (ok)
            {
                if (x == 0)
                {
                    xEventGroupSetBits(data_event_group, TIME_OK_BIT);
                    x = 1;
                }
            }
            else
            {
                ESP_LOGI("ds3231", "READ FAILL");
            }
            xSemaphoreGive(xDataMutex);
        }
        else
        {
            ESP_LOGI("ds3231", "I2C is busy.");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task เช็คเวลาผ่านไปกี่วินาที
void TIME_DIFF(void *arg)
{
    uint8_t current_sec;
    uint8_t last_sec = g_sec;
    printf("last_sec: %u\n", last_sec);

    while (1)
    {
        current_sec = g_sec;
        if (last_sec == 0)
        {
            last_sec = current_sec;
        }

        uint8_t diff;
        if (current_sec >= last_sec)
        {
            diff = current_sec - last_sec;
        }
        else
        {
            diff = (60 - last_sec) + current_sec;
        }
    
        if ((diff >= 10 && z == 0) || z == 1)
        {
            xEventGroupSetBits(data_event_group, READY_DELAY_BIT);
            last_sec = current_sec;
            z = 1;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void ESP_NOW(void *arg)
{

    while (1)
    {
        if (web_mode) // ถ้าอยู่โหมดเว็บ ก็หยุดส่งข้อมูลชั่วคราว
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // รอให้อ่านอุณหภูมิ/เวลา/ดีเลย์ครบก่อนค่อยส่ง
        EventBits_t bits = xEventGroupWaitBits(
            data_event_group,
            TEMP_OK_BIT | TIME_OK_BIT | READY_DELAY_BIT,
            pdTRUE,
            pdTRUE,
            pdMS_TO_TICKS(20000));

        // เตรียมโครงสร้างเวลาที่จะใช้ทำ timestamp
        struct tm t = {0};

        t.tm_year = (2000 + g_year) - 1900; // ต้องลบ 1900
        t.tm_mon = g_month - 1;             // ต้องลบ 1 (0-11)
        t.tm_mday = g_day;
        t.tm_hour = g_hour;
        t.tm_min = g_min;
        t.tm_sec = g_sec;

        // แปลงเป็น timestamp แล้วปรับเป็น UTC
        time_t timestamp = mktime(&t);                 // แปลงเป็น timestamp (local time)
        time_t utc_timestamp = timestamp - (7 * 3600); // ลบ 7 ชั่วโมง

        printf("utc_timestamp: %lld\n", (long long)utc_timestamp);

        struct tm *local = localtime(&utc_timestamp);
        printf("local time: %04d-%02d-%02d %02d:%02d:%02d\n",
               local->tm_year + 1900, local->tm_mon + 1, local->tm_mday,
               local->tm_hour, local->tm_min, local->tm_sec);

        // ถ้า event ครบทั้ง 3 อย่าง
        if ((bits & (TEMP_OK_BIT | TIME_OK_BIT | READY_DELAY_BIT)) ==
            (TEMP_OK_BIT | TIME_OK_BIT | READY_DELAY_BIT))
        {
            // แสดง MAC ปลายทาง
            char mac_str[18];
            snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                     receiver_mac[0], receiver_mac[1], receiver_mac[2],
                     receiver_mac[3], receiver_mac[4], receiver_mac[5]);
            ESP_LOGI("ESP-NOW", "Sending to MAC: %s", mac_str);

            // สร้างข้อความ JSON ที่จะส่ง
            char msg[100];
            snprintf(msg, sizeof(msg), "{\"ts\":%lld,\"values\":{\"temperature\":%.2f,\"store\":true}}",
                     (long long)utc_timestamp * 1000, g_temp);

            // ส่งข้อมูลผ่าน ESP-NOW
            esp_now_send(receiver_mac, (uint8_t *)msg, strlen(msg));
            ESP_LOGI("ESP-NOW", "Sent: %s", msg);

            // รอตรวจสอบว่าส่งสำเร็จหรือไม่
            printf("Time: %02d:%02d:%02d, Date: 20%02d-%02d-%02d\n", g_hour, g_min, g_sec, g_year, g_month, g_day);
            EventBits_t bits = xEventGroupWaitBits(send_event_group, SEND_SUCCESS_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(2000));
            if (bits & SEND_SUCCESS_BIT)
            {
                // ส่งผ่านแล้ว → เข้าสู่โหมดประหยัดพลังงาน
                ESP_LOGI("SLEEP", "Send confirmed. Sleeping");
                esp_wifi_stop();
                x = 0;
                y = 0;
                z = 0;
                esp_deep_sleep(SLEEP_MINUTES * 60 * 1000000ULL);
            }
            else
            {
                // เพิ่มตัวนับเมื่อส่งไม่สำเร็จ 30 ครั้งจะเข้าโหมดประหยัดพลังงาน
                printf("count_failed: %d\n", count_failed);
                if (count_failed > 30)
                {
                    printf("End device cannot send data to the gateway more than 10 rounds -> go to deep sleep mode.\n");

                    esp_deep_sleep(SLEEP_MINUTES * 60 * 1000000ULL);
                }
                count_failed = count_failed + 1;
                x = 0;
                y = 0;

                ESP_LOGI("SLEEP", "Send fail");
            }
        }
        else
        {
            ESP_LOGW(TAG, "read fail");
        }

        vTaskDelay(pdMS_TO_TICKS(DELAY_MINUTE_MS));
    }
}

// Main
void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init()); // เริ่มระบบ NVS
    check_nvs_flag();                  // โหลดค่าตั้งต้นจาก NVS
    load_mac_nvs();

    xDataMutex = xSemaphoreCreateMutex();
    i2c_init();           // เริ่ม I2C
    espnow_init_sender(); // ตั้งค่า ESP-NOW ส่งข้อมูล

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    send_event_group = xEventGroupCreate();
    data_event_group = xEventGroupCreate();

    gpio_reset_pin(BUTTON_GPIO); // ตั้งค่า GPIO ปุ่ม
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);

    // ถ้ายังไม่เคยตั้งค่า (เริ่มต้นครั้งแรก)
    if (!is_configured)
    {
        ESP_LOGW(TAG, "first setting");
        wifi_init_ap(); // เปิด AP ให้ตั้งค่า
        spiffs_init();
        start_webserver();
    }
    else
    {
        xTaskCreate(BUTTON, "BUTTON", 4096, NULL, 6, NULL);       // อ่านปุ่ม
        xTaskCreate(TEMP, "TEMP", 4096, NULL, 5, NULL);           // อ่านอุณหภูมิ HTU21D
        xTaskCreate(TIME, "TIME", 4096, NULL, 5, NULL);           // อ่านเวลา DS3231
        xTaskCreate(TIME_DIFF, "TIME_DIFF", 4096, NULL, 5, NULL); // เช็คเวลาเพี้ยน
        xTaskCreate(ESP_NOW, "ESP_NOW", 4096, NULL, 5, NULL);     // ส่งข้อมูล ESP-NOW
    }
}
