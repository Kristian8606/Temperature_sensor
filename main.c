#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
//#include "wifi.h"
#include <wifi_config.h>
#include "i2c/i2c.h"
#include "bmp280/bmp280.h"

#define LED_GPIO 2
// In forced mode user initiate measurement each time.
// In normal mode measurement is done continuously with specified standby time.
// #define MODE_FORCED
const uint8_t i2c_bus = 0;
const uint8_t scl_pin = 0;
const uint8_t sda_pin = 4;

void led_write(bool on)
{
    gpio_write(LED_GPIO, on ? 0 : 1);
}

void identify_task(void *_args) {

    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            led_write(true);
            vTaskDelay(150 / portTICK_PERIOD_MS);
            led_write(false);
            vTaskDelay(150 / portTICK_PERIOD_MS);
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    led_write(false);

    vTaskDelete(NULL);
}

void temperature_sensor_identify(homekit_value_t _value) {
    printf("Temperature sensor identify\n");
   
    xTaskCreate(identify_task, "identify_task", 128, NULL, 2, NULL);

}
homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "Temperature");
homekit_characteristic_t current_temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t current_humidity    = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);


#ifdef MODE_FORCED
static void bmp280_task_forced(void *pvParameters)
{
    bmp280_params_t  params;
    float pressure, temperature, humidity;

    bmp280_init_default_params(&params);
    params.mode = BMP280_MODE_FORCED;

    bmp280_t bmp280_dev;
    bmp280_dev.i2c_dev.bus = i2c_bus;
    bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;

    while (1) {
        while (!bmp280_init(&bmp280_dev, &params)) {
            printf("BMP280 initialization failed\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        bool bme280p = bmp280_dev.id == BME280_CHIP_ID;
        printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

        while(1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            if (!bmp280_force_measurement(&bmp280_dev)) {
                printf("Failed initiating measurement\n");
                break;
            }
            // wait for measurement to complete
            while (bmp280_is_measuring(&bmp280_dev)) {};

            if (!bmp280_read_float(&bmp280_dev, &temperature, &pressure, &humidity)) {
                printf("Temperature/pressure reading failed\n");
                break;
            }
            printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
            if (bme280p)
                printf(", Humidity: %.2f\n", humidity);
            else
                printf("\n");
                
            current_temperature.value = HOMEKIT_FLOAT(temperature);
            current_humidity.value = HOMEKIT_FLOAT(humidity);

            homekit_characteristic_notify(&current_temperature, current_temperature.value);
            homekit_characteristic_notify(&current_humidity, current_humidity.value);
        }
    }
}
#else
static void bmp280_task_normal(void *pvParameters)
{
    bmp280_params_t  params;
    float pressure, temperature, humidity;

    bmp280_init_default_params(&params);

    bmp280_t bmp280_dev;
    bmp280_dev.i2c_dev.bus = i2c_bus;
    bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;

    while (1) {
        while (!bmp280_init(&bmp280_dev, &params)) {
            printf("BMP280 initialization failed\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        bool bme280p = bmp280_dev.id == BME280_CHIP_ID;
        printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

        while(1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            if (!bmp280_read_float(&bmp280_dev, &temperature, &pressure, &humidity)) {
                printf("Temperature/pressure reading failed\n");
                break;
            }
            printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
            if (bme280p)
                printf(", Humidity: %.2f\n", humidity);
            else
                printf("\n");
                
            current_temperature.value = HOMEKIT_FLOAT(temperature);
            current_humidity.value = HOMEKIT_FLOAT(humidity);

            homekit_characteristic_notify(&current_temperature, current_temperature.value);
            homekit_characteristic_notify(&current_humidity, current_humidity.value);
        }
    }
}
#endif

void temperature_sensor_init() {
	
	    i2c_init(i2c_bus, scl_pin, sda_pin, I2C_FREQ_400K);

#ifdef MODE_FORCED
    xTaskCreate(bmp280_task_forced, "bmp280_task", 256, NULL, 2, NULL);
#else
    xTaskCreate(bmp280_task_normal, "bmp280_task", 256, NULL, 2, NULL);
#endif

	 gpio_enable(LED_GPIO, GPIO_OUTPUT);
    led_write(false);

}


homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_thermostat, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Kriss"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "0012345"),
            HOMEKIT_CHARACTERISTIC(MODEL, "BMP280/BME280"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, temperature_sensor_identify),
            NULL
        }),
        HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"),
            &current_temperature,
            NULL
        }),
        HOMEKIT_SERVICE(HUMIDITY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Humidity Sensor"),
            &current_humidity,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};

void on_wifi_ready() {
    homekit_server_init(&config);
}

void user_init(void) {
    uart_set_baud(0, 115200);

    
    
	 uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);
    int name_len = snprintf(NULL, 0, "Temperature Sensor-%02X%02X%02X", macaddr[1], macaddr[2], macaddr[3]);
    char *name_value = malloc(name_len + 1);
    snprintf(name_value, name_len + 1, "Temperature Sensor-%02X%02X%02X", macaddr[1], macaddr[2], macaddr[3]);
    name.value = HOMEKIT_STRING(name_value);

    wifi_config_init("Temperature Sensor", NULL, on_wifi_ready);
	
    temperature_sensor_init();
}
