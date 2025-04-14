#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define TAG "SmartHome"

// UART Configuration
#define BUF_SIZE 1024

// Message Queue Configuration
#define MESSAGE_QUEUE_SIZE 5
static QueueHandle_t message_queue;

// Living Room Configuration
#define DOOR_SENSOR_PIN 15
#define BUTTON_LIVING_PIN 16
#define FAN_LIVING_PIN 12
#define LIGHT_LIVING_PIN 14
#define TEMP_SENSOR_CHANNEL ADC1_CHANNEL_0 // GPIO 36 (ADC)

// Kitchen Configuration
#define BUTTON_KITCHEN_PIN 19
#define FAN_KITCHEN_PIN 21
#define LIGHT_KITCHEN_PIN 22
#define GAS_SENSOR_CHANNEL ADC1_CHANNEL_3 // GPIO 39 (ADC)

// Bathroom Configuration
#define MOTION_SENSOR_BATHROOM_PIN 23
#define BUTTON_BATHROOM_PIN 25
#define FAN_BATHROOM_PIN 26
#define LIGHT_BATHROOM_PIN 27

// Task Handles
TaskHandle_t living_room_task_handle = NULL;
TaskHandle_t kitchen_task_handle = NULL;
TaskHandle_t bathroom_task_handle = NULL;

// Shared State
static volatile int has_error = 0;

// Function to send error to the message queue
void send_error_to_queue(const char *error_message) {
    char message[128];
    snprintf(message, sizeof(message), "%s", error_message);
    xQueueSend(message_queue, &message, portMAX_DELAY);
}

// *** Task: Living Room ***
void living_room_task(void *pvParameters) {
    int prev_door_state = 0;
    int prev_button_state = 0;
    int button_hold_time = 0;

    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);

    while (1) {
        if (has_error) vTaskSuspend(NULL);

        // Door sensor to control light
        int door_state = gpio_get_level(DOOR_SENSOR_PIN);
        if (door_state == 1 && prev_door_state == 0) {
            gpio_set_level(LIGHT_LIVING_PIN, !gpio_get_level(LIGHT_LIVING_PIN));
            ESP_LOGI(TAG, "Living Room: Light toggled");
        }
        prev_door_state = door_state;

        // Button to control fan
        int button_state = gpio_get_level(BUTTON_LIVING_PIN);
        if (button_state == 1) {
            button_hold_time++;
            if (button_hold_time > 50) {
                send_error_to_queue("Living Room: Button held too long (>5s)");
                ESP_LOGE(TAG, "Living Room: Button held too long");
            }
        } else if (button_hold_time > 0 && button_hold_time <= 50) {
            gpio_set_level(FAN_LIVING_PIN, !gpio_get_level(FAN_LIVING_PIN));
            ESP_LOGI(TAG, "Living Room: Fan toggled");
            button_hold_time = 0;
        }

        // Temperature sensor
        uint32_t adc_reading = adc1_get_raw(TEMP_SENSOR_CHANNEL);
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        float temperature = (voltage - 500) / 10.0;
        if (temperature > 50.0 || temperature < 0.0) {
            char error[128];
            snprintf(error, sizeof(error), "Living Room: Abnormal temperature (%.2f°C)", temperature);
            send_error_to_queue(error);
            ESP_LOGE(TAG, "%s", error);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// *** Task: Kitchen ***
void kitchen_task(void *pvParameters) {
    int prev_button_state = 0;
    int button_hold_time = 0;

    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);

    while (1) {
        if (has_error) vTaskSuspend(NULL);

        // Button to control fan
        int button_state = gpio_get_level(BUTTON_KITCHEN_PIN);
        if (button_state == 1) {
            button_hold_time++;
            if (button_hold_time > 50) {
                send_error_to_queue("Kitchen: Button held too long (>5s)");
                ESP_LOGE(TAG, "Kitchen: Button held too long");
            }
        } else if (button_hold_time > 0 && button_hold_time <= 50) {
            gpio_set_level(FAN_KITCHEN_PIN, !gpio_get_level(FAN_KITCHEN_PIN));
            ESP_LOGI(TAG, "Kitchen: Fan toggled");
            button_hold_time = 0;
        }

        // Gas sensor
        uint32_t adc_reading = adc1_get_raw(GAS_SENSOR_CHANNEL);
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        float gas_level = voltage / 1000.0; // Normalize to 0-1.0 range
        if (gas_level > 0.8) {
            send_error_to_queue("Kitchen: High gas level detected");
            ESP_LOGE(TAG, "Kitchen: High gas level detected");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// *** Task: Bathroom ***
void bathroom_task(void *pvParameters) {
    int prev_motion_state = 0;
    int prev_button_state = 0;
    int button_hold_time = 0;

    while (1) {
        if (has_error) vTaskSuspend(NULL);

        // Motion sensor to control light
        int motion_state = gpio_get_level(MOTION_SENSOR_BATHROOM_PIN);
        if (motion_state == 1 && prev_motion_state == 0) {
            gpio_set_level(LIGHT_BATHROOM_PIN, !gpio_get_level(LIGHT_BATHROOM_PIN));
            ESP_LOGI(TAG, "Bathroom: Light toggled");
        }
        prev_motion_state = motion_state;

        // Button to control fan
        int button_state = gpio_get_level(BUTTON_BATHROOM_PIN);
        if (button_state == 1) {
            button_hold_time++;
            if (button_hold_time > 50) {
                send_error_to_queue("Bathroom: Button held too long (>5s)");
                ESP_LOGE(TAG, "Bathroom: Button held too long");
            }
        } else if (button_hold_time > 0 && button_hold_time <= 50) {
            gpio_set_level(FAN_BATHROOM_PIN, !gpio_get_level(FAN_BATHROOM_PIN));
            ESP_LOGI(TAG, "Bathroom: Fan toggled");
            button_hold_time = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// *** Task: Error Handler ***
void error_handler_task(void *pvParameters) {
    char received_message[128];
    while (1) {
        if (xQueueReceive(message_queue, &received_message, portMAX_DELAY)) {
            ESP_LOGW(TAG, "Error: %s", received_message);

            // Suspend all tasks
            has_error = 1;
            vTaskSuspend(living_room_task_handle);
            vTaskSuspend(kitchen_task_handle);
            vTaskSuspend(bathroom_task_handle);

            // Simulate error handling
            ESP_LOGI(TAG, "Handling error...");
            vTaskDelay(pdMS_TO_TICKS(5000)); // Example handling time

            // Resume all tasks
            has_error = 0;
            vTaskResume(living_room_task_handle);
            vTaskResume(kitchen_task_handle);
            vTaskResume(bathroom_task_handle);

            ESP_LOGI(TAG, "Error resolved. Tasks resumed.");
        }
    }
}

// *** Main Application ***
void app_main() {
    // GPIO Configuration
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DOOR_SENSOR_PIN) | (1ULL << BUTTON_LIVING_PIN) |
                        (1ULL << FAN_LIVING_PIN) | (1ULL << LIGHT_LIVING_PIN) |
                        (1ULL << BUTTON_KITCHEN_PIN) | (1ULL << FAN_KITCHEN_PIN) |
                        (1ULL << LIGHT_KITCHEN_PIN) | (1ULL << MOTION_SENSOR_BATHROOM_PIN) |
                        (1ULL << BUTTON_BATHROOM_PIN) | (1ULL << FAN_BATHROOM_PIN) |
                        (1ULL << LIGHT_BATHROOM_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Ensure all devices are off initially
    gpio_set_level(FAN_LIVING_PIN, 0);
    gpio_set_level(LIGHT_LIVING_PIN, 0);
    gpio_set_level(FAN_KITCHEN_PIN, 0);
    gpio_set_level(LIGHT_KITCHEN_PIN, 0);
    gpio_set_level(FAN_BATHROOM_PIN, 0);
    gpio_set_level(LIGHT_BATHROOM_PIN, 0);

    // Create Message Queue
    message_queue = xQueueCreate(MESSAGE_QUEUE_SIZE, sizeof(char[128]));

    // Create Tasks
    xTaskCreate(&living_room_task, "living_room_task", 4096, NULL, 5, &living_room_task_handle);
    xTaskCreate(&kitchen_task, "kitchen_task", 4096, NULL, 5, &kitchen_task_handle);
    xTaskCreate(&bathroom_task, "bathroom_task", 2048, NULL, 5, &bathroom_task_handle);
    xTaskCreate(&error_handler_task, "error_handler_task", 2048, NULL, 5, NULL);
}
