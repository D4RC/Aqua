#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <esp_log.h>
#include <esp_log_internal.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "pn532.h"
#include "driver/timer.h"
#include "driver/i2c.h"

#define BLINK_GPIO CONFIG_BLINK_GPIO
#define STATUS_GPIO CONFIG_LED_STATUS_GPIO

#define PN532_SCK CONFIG_PN532_SCK
#define PN532_MOSI CONFIG_PN532_MOSI
#define PN532_SS CONFIG_PN532_SS
#define PN532_MISO CONFIG_PN532_MISO

#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

#define I2C_MASTER_SCL_IO               CONFIG_SCL_IO
#define I2C_MASTER_SDA_IO               CONFIG_SDA_IO
#define I2C_MASTER_NUM                  CONFIG_I2C_NUM
#define I2C_MASTER_TX_BUF_DISABLE       0
#define I2C_MASTER_RX_BUF_DISABLE       0
#define I2C_MASTER_FREQ_HZ              400000

#define TOF_INTR_GPIO CONFIG_TOF_INTR
#define TOF_XSHUT_IO CONFIG_XSHUT_IO
#define ESP_INTR_FLAG_DEFAULT 0

#define WATER_GPIO CONFIG_WATER_GPIO
#define FLAVOR1_GPIO CONFIG_FLAVOR1_GPIO
#define FLAVOR2_GPIO CONFIG_FLAVOR2_GPIO

static xQueueHandle s_timer_queue;
xQueueHandle s_service_queue;

typedef struct s_service_data{
    uint8_t serve;
    uint8_t flavor;
    uint8_t concentration;
    uint32_t quantity;
} s_service_data_t;

static TaskHandle_t xServiceTask = NULL;

static const char *TAG = "APP";
static pn532_t nfc;

extern void vl53l0x_init();
extern void vl53l0x_clear_interrupt();
extern void initialize_iot();
extern void mqtt_task(void *pvParameters);
extern void publish_service_request(uint8_t *target_uid, uint8_t uid_length);
extern void publish_transaction(uint8_t* target_uid, uint8_t uid_length, uint32_t quantity, uint8_t flavor);

const TickType_t xDelay = 10000 / portTICK_PERIOD_MS;

/*
    The Timer ISR callback will perform the following operations

    Turn off the water providers
    Stop the timer
*/
static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    gpio_set_level(WATER_GPIO, 0);
    gpio_set_level(FLAVOR1_GPIO, 0);
    gpio_set_level(FLAVOR2_GPIO, 0);
    timer_group_set_counter_enable_in_isr(TIMER_GROUP_0, TIMER_0, TIMER_PAUSE);

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0);
    xQueueSendFromISR(s_timer_queue, &timer_counter_value, &high_task_awoken);
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

/*
    The TOF ISR will perform the following operations

    Stop the timer if it is already not stoped
    Notify the end of service
*/
static void IRAM_ATTR tof_isr(void *args)
{
    BaseType_t higher_priority_task_awoken = pdFALSE;

    gpio_set_level(WATER_GPIO, 0);
    gpio_set_level(FLAVOR1_GPIO, 0);
    gpio_set_level(FLAVOR2_GPIO, 0);
    timer_group_set_counter_enable_in_isr(TIMER_GROUP_0, TIMER_0, TIMER_PAUSE);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0);

    // Send the event data back to the task
    // Items are queued by copy
    xQueueSendFromISR(s_timer_queue, &timer_counter_value, &higher_priority_task_awoken);
    vTaskNotifyGiveFromISR(xServiceTask, &higher_priority_task_awoken);
    
    if(higher_priority_task_awoken)
    {
        portYIELD_FROM_ISR();
    }
}

static void service_tg_timer_init()
{
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_DIS,
    }; // default clock source is APB

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, NULL, 0);
}

static void i2c_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void io_config()
{
    gpio_config_t io_conf;

    // WATER PUMP GPIO
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL << WATER_GPIO) | (1ULL << FLAVOR1_GPIO) | (1ULL << FLAVOR2_GPIO));
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);

    // VL53L0X TOF XSHUT
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // The TOF module has external pullup resistor, hence open drain is configured
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask = (1ULL << TOF_XSHUT_IO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);
    gpio_set_level(CONFIG_XSHUT_IO, 0);

    // VL53L0X TOF INTR GPIO
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << TOF_INTR_GPIO);
    // The TOF module has 10k external pullup resistor
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;   
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(TOF_INTR_GPIO, tof_isr, NULL);
    gpio_intr_disable(TOF_INTR_GPIO);

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(STATUS_GPIO);
    gpio_set_direction(STATUS_GPIO, GPIO_MODE_OUTPUT);
}


void blink_task(void *pvParameter)
{
    while (1)
    {
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(900 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void nfc_task(void *pvParameter)
{
    pn532_spi_init(&nfc, PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);
    pn532_begin(&nfc);

    uint32_t versiondata = pn532_getFirmwareVersion(&nfc);
    if (!versiondata)
    {
        gpio_set_level(STATUS_GPIO, 1);
        ESP_LOGI(TAG, "Didn't find PN53x board");
        while (1)
        {
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }
    // Got ok data, print it out!
    ESP_LOGI(TAG, "Found chip PN5 %x", (versiondata >> 24) & 0xFF);
    ESP_LOGI(TAG, "Firmware ver. %d.%d", (versiondata >> 16) & 0xFF, (versiondata >> 8) & 0xFF);

    // configure board to read RFID tags
    pn532_SAMConfig(&nfc);

    ESP_LOGI(TAG, "Waiting for an ISO14443A Card ...");
    
    while (1)
    {
        uint8_t success;
        uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0}; // Buffer to store the returned UID
        uint8_t uidLength;                     // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

        // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
        // 'uid' will be populated with the UID, and uidLength will indicate
        // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
        success = pn532_readPassiveTargetID(&nfc, PN532_MIFARE_ISO14443A, uid, &uidLength, 0);

        if (success)
        {
            gpio_set_level(STATUS_GPIO, 1);

            // Display some basic information about the card
            ESP_LOGI(TAG, "Found an ISO14443A card");
            ESP_LOGI(TAG, "UID Length: %d bytes", uidLength);
            ESP_LOGI(TAG, "UID Value:");
            esp_log_buffer_hexdump_internal(TAG, uid, uidLength, ESP_LOG_INFO);

            publish_service_request(uid, uidLength);
            s_service_data_t service_data;

            xQueueReset(s_service_queue);
            if (xQueueReceive(s_service_queue, ( void * )&service_data, xDelay ) == pdPASS)
            {
                if (service_data.serve)
                {
                    gpio_set_level(CONFIG_XSHUT_IO, 1);
                    vl53l0x_init();
                    gpio_intr_enable(TOF_INTR_GPIO);
                    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
                    
                    int timer_interval_sec;
                    float time_scale = 0;

                    switch(service_data.flavor)
                    {
                        case 0: // Pure water
                            gpio_set_level(WATER_GPIO, 1);
                            timer_interval_sec = service_data.quantity / 13.5;
                            time_scale = 13.5;
                            break;
                        case 1: // Lime
                            gpio_set_level(WATER_GPIO, 1);
                            gpio_set_level(FLAVOR1_GPIO, 1);
                            timer_interval_sec = service_data.quantity / 17.5;    //18 ml / s
                            time_scale = 17.5;
                            break;
                        case 2: // Cherry
                            gpio_set_level(WATER_GPIO, 1);
                            gpio_set_level(FLAVOR2_GPIO, 1);
                            timer_interval_sec = service_data.quantity / 17.5;
                            time_scale = 17.5;
                            break;
                        default:
                            gpio_set_level(WATER_GPIO, 1);
                            timer_interval_sec = service_data.quantity / 17.5;
                            time_scale = 17.5;
                            break;
                    }

                    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer_interval_sec * TIMER_SCALE);
                    timer_start(TIMER_GROUP_0, TIMER_0);

                    uint64_t counter_value;

                    xQueueReset(s_timer_queue);
                    xQueueReceive(s_timer_queue, &counter_value, portMAX_DELAY);
                    printf("Time   : %.8f s\r\n", (double) counter_value / TIMER_SCALE);

                    uint32_t served_quantity = (counter_value / TIMER_SCALE) * time_scale;
                    publish_transaction(uid, uidLength, served_quantity, service_data.flavor);
                    
                     // Wait until bottle is retired
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                    gpio_intr_disable(TOF_INTR_GPIO);
                    gpio_set_level(CONFIG_XSHUT_IO, 0);

                    gpio_set_level(STATUS_GPIO, 0);
                }
                else
                {
                    ESP_LOGI(TAG, "Not enough credits");
                }
            }
            else
            {
                ESP_LOGI(TAG, "Timed out, no response received from the server");
                gpio_set_level(STATUS_GPIO, 0);
            }
        }
        else
        {
            // PN532 probably timed out waiting for a card
            ESP_LOGI(TAG, "Timed out waiting for a card");
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }
}

// void process_timer_task()
// {
//     while(1)
//     {
//         uint64_t counter_value;
//         xQueueReceive(s_timer_queue, &counter_value, portMAX_DELAY);
        
//         publish_transaction(uid, uidLength, , uint8_t flavor)
//         printf("Time   : %.8f s\r\n", (double) counter_value / TIMER_SCALE);
//     }
// }

void app_main()
{
    initialize_iot();
    s_timer_queue = xQueueCreate(1, sizeof(uint64_t));
    s_service_queue = xQueueCreate(1, sizeof(s_service_data_t));
    
    service_tg_timer_init();
    i2c_init();
    io_config();

    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 8192, NULL, 5, NULL);
    xTaskCreate(&nfc_task, "nfc_task", 4096, NULL, 4, &xServiceTask);
    //xTaskCreate(&process_timer_task, "process_timer_task", 4096, NULL, 3, NULL);
}

