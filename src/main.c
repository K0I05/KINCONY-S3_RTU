/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file main.c
 * 
 * @brief This is a demo application for the Kincony S3 core development board
 * that connects to an IP network over WIFI with SNTP time synchronization and
 * polls 4 sensors over MODBUS, 1-Wire, and I2C.  The RS2E and WDC2E RTU sensors 
 * are polled over MODBUS via an RS-485 serial connection, the DS18B20 sensor is 
 * polled over one-wire-bus, the BMP280 is polled over I2C, and samples are 
 * printed via serial debug at a 6-second interval.
 * 
 * Demo source code is available here: https://github.com/K0I05/KINCONY-S3_RTU_20250110
 * 
 * Components used in this demo are available here: https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS
 * 
 * 
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

/*
 *  C:\Users\lavco\.platformio\penv\Scripts\platformio.exe run -t menuconfig
 * 
 *  ESP-MODBUS DOC: https://docs.espressif.com/projects/esp-modbus/en/latest/esp32/
 * 
 * 
 * KINCONY-S3-V1.0 Core Development Board (https://shop.kincony.com/products/kincony-esp32-s3-wroom-1u-n16r8-core-development-board)
 * 
 *                   C   
 * 3     D           B    h
 * V 2 1 N 4 3 2 1   S    t
 * 3 W W G A A A A   U    E
 * 
 * 1 2 3 4 5 6 7 8
 * 
 * Upper and lower terminal block connections:
 * 
 * 1 2 3 4 5 6 7 8 9
 * 
 * 3 D A L D - + D V
 * V N D C N B A N 4
 * 3 G S S G     G 2
 * 
 * 
 * Core Board Schematic (https://www.kincony.com/download/S3-schematic.pdf)
 * Core Board Pin Definitions (https://www.kincony.com/forum/showthread.php?tid=5715)
 * 
 * #define ANALOG_A1  GPIO7
 * #define ANALOG_A2  GPIO6
 * #define ANALOG_A3  GPIO5
 * #define ANALOG_A4  GPIO4
 * 
 * Ethernet (W5500) I/O define:
 * 
 * clk_pin: GPIO43
 * mosi_pin: GPIO44
 * miso_pin: GPIO42
 * cs_pin: GPIO41
 * interrupt_pin: GPIO2
 * reset_pin: GPIO1
 * 
 * 1-wire(TEM1): GPIO8
 * 1-wire(TEM2): GPIO40
 * 
 * SDA:GPIO39
 * SCL:GPIO38
 * 
 * RS485:
 * RXD:GPIO15
 * TXD:GPIO16
 * 
 * 4G module:
 * RXD:GPIO17
 * TXD:GPIO18
 * 
 * RF 433MHz receiver: GPIO9
 * 
 * free GPIOs on PCB:
 * GPIO10
 * GPIO11
 * GPIO12
 * GPIO13
 * GPIO14
 * GPIO21
 * GPIO47
 * GPIO48
 * 
 * 
 * See modbus_connect.c for MODBUS serial port configuration parameters.
 * See network_connect.c for WIFI configuration parameters.
 * 
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/param.h>
#include <sdkconfig.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_check.h>
#include <esp_log.h>
#include <esp_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <driver/gpio.h>

/* app includes */
#include <network_connect.h>
#include <modbus_connect.h>

/* component includes */
#include <time_into_interval.h>
#include <nvs_ext.h>
//#include <onewire_bus.h>
#include <ds18b20.h>
#include <bmp280.h>



/**
 * @brief FreeRTOS definitions
 */

#define MINIMAL_STACK_SIZE                      (1024)

/**
 * @brief I2C and 1-Wire definitions
 */

#define I2C0_MASTER_PORT                        I2C_NUM_0
#define I2C0_MASTER_SDA_IO                      GPIO_NUM_39 // blue
#define I2C0_MASTER_SCL_IO                      GPIO_NUM_38 // yellow
#define OWB0_MASTER_DQ_IO                       GPIO_NUM_8

/**
 * @brief macro definitions
 */

#define I2C0_MASTER_DEFAULT_CONFIG {                                \
        .clk_source                     = I2C_CLK_SRC_DEFAULT,      \
        .i2c_port                       = I2C0_MASTER_PORT,         \
        .scl_io_num                     = I2C0_MASTER_SCL_IO,       \
        .sda_io_num                     = I2C0_MASTER_SDA_IO,       \
        .glitch_ignore_cnt              = 7,                        \
        .flags.enable_internal_pullup   = true, }

#define OW0_RMT_CONFIG_DEFAULT { .max_rx_bytes = 10 } // 1-byte ROM command + 8-byte ROM number + 1-byte device command

#define OW0_MASTER_CONFIG_DEFAULT { .bus_gpio_num = OWB0_MASTER_DQ_IO }

/**
 * @brief static constant and global definitions
 */

/* app tag for esp logging */
static const char *TAG = "RTU [APP]";

static TaskHandle_t             s_sample_sensor_task_hdl = NULL;
static i2c_master_bus_config_t  s_i2c0_bus_cfg = I2C0_MASTER_DEFAULT_CONFIG;
static i2c_master_bus_handle_t  s_i2c0_bus_hdl = NULL;
static onewire_bus_rmt_config_t s_owb0_rmt_cfg = OW0_RMT_CONFIG_DEFAULT;
static onewire_bus_config_t     s_owb0_bus_cfg = OW0_MASTER_CONFIG_DEFAULT;
static onewire_bus_handle_t     s_owb0_bus_hdl = NULL;

/**
 * @brief struct and enum definitions
 */



/**
 * @brief RS2E precipitation states enumerator.
 * 
    # precipitation type (16-bit int): register 12 (16-bit int) = 16-bit int
    #   types: rain | snow | rain + snow | hail | rain + snow + hail
    #
    #   b15 b14 b13 b12 b11 b10 b09 b08 b07 b06 b05 b04 b03 b02 b01 b00   precipitation type bit mask
    #    0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0    no  precip
    #    0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   1    rain
    #    0   0   0   0   0   0   0   0   0   0   0   0   0   0   1   0    snow
    #    0   0   0   0   0   0   0   0   0   0   0   0   0   0   1   1    rain + snow
    #    0   0   0   0   0   0   0   0   0   0   0   0   0   1   0   0    hail
    #    0   0   0   0   0   0   0   0   0   0   0   0   0   1   1   1    rain + snow + hail 
    #
 */
typedef enum rs2e_precip_types_tag {
    RS2E_PRECIP_TYPE_ERROR           = 10,
    RS2E_PRECIP_TYPE_NO_RAIN         = 0b0000000000000000,
    RS2E_PRECIP_TYPE_RAIN            = 0b0000000000000001,
    RS2E_PRECIP_TYPE_SNOW            = 0b0000000000000010,
    RS2E_PRECIP_TYPE_RAIN_SNOW       = 0b0000000000000011,
    RS2E_PRECIP_TYPE_HAIL            = 0b0000000000000100,
    RS2E_PRECIP_TYPE_RAIN_SNOW_HAIL  = 0b0000000000000111
} rs2e_precip_types_t;

/**
 * @brief Converts RS2E precipitation type to a string representation.
 * 
 * @param precip_type RS2E precipitation type to convert.
 * @return const char* RS2E precipitation type as a string.
 */
static inline const char* rs2e_precip_type_to_string(const rs2e_precip_types_t precip_type) {
    switch(precip_type) {
        case RS2E_PRECIP_TYPE_ERROR:
            return "Error";
        case RS2E_PRECIP_TYPE_NO_RAIN:
            return "No Rain";
        case RS2E_PRECIP_TYPE_RAIN:
            return "Rain";
        case RS2E_PRECIP_TYPE_SNOW:
            return "Snow";
        case RS2E_PRECIP_TYPE_RAIN_SNOW:
            return "Rain & Snow";
        case RS2E_PRECIP_TYPE_HAIL:
            return "Hail";
        case RS2E_PRECIP_TYPE_RAIN_SNOW_HAIL:
            return "Rain, Snow & Hail";
        default:
            return "Error";
    }
}

/**
 * @brief Prints the free heap size in bytes with consumed bytes stats as an
 * esp information log.  This is used to monitor consumed bytes for possible 
 * memory leak(s) in the application.
 * 
 * @param free_heap_size_last Last free heap size in bytes.
 * @return uint32_t Adjusted last free heap size in bytes.
 */
static inline uint32_t print_free_heap_size(const uint32_t free_heap_size_last) {
    uint32_t free_heap_size_start = free_heap_size_last; /* set free heap size start from last */
    uint32_t free_heap_size = esp_get_free_heap_size();  /* set free heap size */
    if(free_heap_size_start == 0) free_heap_size_start = free_heap_size;
    int32_t free_heap_size_delta = free_heap_size_start - free_heap_size;
    if(free_heap_size_delta < 0) { free_heap_size_start = free_heap_size; free_heap_size_delta = 0; }
    ESP_LOGW(TAG, "Free Memory:       %lu bytes (%li bytes Consumed)", free_heap_size, free_heap_size_delta);
    return free_heap_size_start;
}

/**
 * @brief Calculates dewpoint temperature from air temperature and relative humidity.
 *
 * @param[in] temperature Air temperature in degrees Celsius.
 * @param[in] humidity Relative humidity in percent.
 * @return float Calculated dewpoint temperature in degrees Celsius.
 */
static inline float rs2e_calculate_dewpoint(const float temperature, const float humidity) {
    // validate range of parameters
    if(temperature > 80 || temperature < -40) return NAN;
    if(humidity > 100 || humidity < 0) return NAN;
    
    // calculate dew-point temperature
    double H = (log10(humidity)-2)/0.4343 + (17.62*temperature)/(243.12+temperature);
    return 243.12*H/(17.62-H);
}

/**
 * @brief Task that samples a sensor over MODBUS. 
 * 
 * @param pvParameters Parameters for task.
 */
static void sample_sensor_task( void *pvParameters ) {
    // bmp280 i2c device handle and configuration
    const bmp280_config_t       bmp280_cfg = I2C_BMP280_CONFIG_DEFAULT;
    bmp280_handle_t             bmp280_hdl;

    // initialize owb device configuration
    ds18b20_config_t             ds18b20_cfg = OWB_DS18B20_CONFIG_DEFAULT;
    ds18b20_handle_t             ds18b20_hdl;
    onewire_device_iter_handle_t ds18b20_iter_hdl;
    onewire_device_t             ds18b20;

    // time-into-interval sampling handle and configuration
    time_into_interval_handle_t tii_sampling_hdl;
    const time_into_interval_config_t tii_sampling_cfg = {
        .name               = "tii_sampling",
        .interval_type      = TIME_INTO_INTERVAL_SEC,
        .interval_period    = 6,
        .interval_offset    = 0
    };

    // attempt to initialize a bmp280 device handle
    bmp280_init(s_i2c0_bus_hdl, &bmp280_cfg, &bmp280_hdl);
    if (bmp280_hdl == NULL) {
        ESP_LOGE(TAG, "Unable to initialize bmp280 device handle");
        esp_restart(); 
    }

    // instantiate 1-wire device iterator handle
    ESP_ERROR_CHECK( onewire_new_device_iter(s_owb0_bus_hdl, &ds18b20_iter_hdl) );
    
    // get next 1-wire device
    if (onewire_device_iter_get_next(ds18b20_iter_hdl, &ds18b20) == ESP_OK) {
        // check if the device is a ds18b20, if so, return the ds18b20 handle
        if (ds18b20_init(&ds18b20, &ds18b20_cfg, &ds18b20_hdl) == ESP_OK) {
            ESP_LOGI(TAG, "Found a ds18b20, address: %016llX", ds18b20.address);
        } else {
            ESP_LOGI(TAG, "Found an unknown device, address: %016llX", ds18b20.address);
        }
    }

    // free device iter handle
    ESP_ERROR_CHECK( onewire_del_device_iter(ds18b20_iter_hdl) );

    /* attempt to initialize a time-into-interval sampling handle - task system clock synchronization */
    time_into_interval_init(&tii_sampling_cfg, &tii_sampling_hdl);
    if (tii_sampling_hdl == NULL) {
        ESP_LOGE(TAG, "Unable to initialize time-into-interval handle");
        esp_restart(); 
    }

    /* attempt to initialize modbus master - device peripheral and objects */
    if(master_init() != ESP_OK) {
        ESP_LOGE(TAG, "Unable to initialize modbus master controller handle");
        esp_restart();
    }

    /* enter task loop */
    for ( ;; ) {
        /* time-into-interval task delay (6-sec sampling interval) */
        time_into_interval_delay(tii_sampling_hdl);

        ESP_LOGE(TAG, "################## KINCONY-S3 ##################");

        /* attempt to get temperature over modbus from rs2e sensor */
        float ta_sample;
        esp_err_t result = master_get_rs2e_temperature(&ta_sample);
        if(result != ESP_OK) {
            ta_sample = NAN;
            ESP_LOGE(TAG, "RS2E sensor read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(TAG, "RS2E Air Temperature:        %.2f°C", ta_sample);
        }

        /* settling delay between modbus query transactions with rtu */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* attempt to get humidity over modbus from rs2e sensor */
        float hr_sample;
        result = master_get_rs2e_humidity(&hr_sample);
        if(result != ESP_OK) {
            hr_sample = NAN;
            ESP_LOGE(TAG, "RS2E sensor read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(TAG, "RS2E Relative Humidity:      %.2f %%", hr_sample);
        }

        /* settling delay between modbus query transactions with rtu */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* attempt to calculate dewpoint temperature */
        float td_sample;
        if(isnan(ta_sample) || isnan(hr_sample)) {
            td_sample = NAN;
            ESP_LOGE(TAG, "RS2E sensor read failed (invalid samples)");
        } else {
            td_sample = rs2e_calculate_dewpoint(ta_sample, hr_sample);
            ESP_LOGI(TAG, "RS2E Dewpoint Temperature:   %.2f°C", td_sample);
        }

        /* settling delay between modbus query transactions with rtu */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* attempt to get precipitation rate over modbus from rs2e sensor */
        float pr_sample;
        result = master_get_rs2e_precipitation_rate(&pr_sample);
        if(result != ESP_OK) {
            pr_sample = NAN;

            ESP_LOGE(TAG, "RS2E sensor read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(TAG, "RS2E Precipitation Rate:     %.2f mm/h", pr_sample);
        }

        /* settling delay between modbus query transactions with rtu */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* attempt to get precipitation type over modbus from rs2e sensor */
        uint16_t pt_sample;
        result = master_get_rs2e_precipitation_type(&pt_sample);
        if(result != ESP_OK) {
            /* set precip state and situation to error values */
            pt_sample = RS2E_PRECIP_TYPE_ERROR;

            ESP_LOGE(TAG, "RS2E sensor read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(TAG, "RS2E Precipitation Type:     %s", rs2e_precip_type_to_string(pt_sample));
        }

        /* settling delay between modbus query transactions with rtu */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* attempt to get wind direction over modbus from wdc2e sensor */
        uint16_t wd_sample;
        result = master_get_wdc2e_wind_direction(&wd_sample);
        if(result != ESP_OK) {
            wd_sample = 361;

            ESP_LOGE(TAG, "WDC2E sensor read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(TAG, "WDC2E Wind Direction:        %u°", wd_sample);
        }

        /* settling delay between modbus query transactions with rtu */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* attempt to get wind speed over modbus from wdc2e sensor */
        float ws_sample;
        result = master_get_wdc2e_wind_speed(&ws_sample);
        if(result != ESP_OK) {
            ws_sample = NAN;

            ESP_LOGE(TAG, "WDC2E sensor read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(TAG, "WDC2E Wind Speed:            %.2f m/s", ws_sample);
        }

        /* attempt to get ground temperature over owb from ds18b20 sensor */
        float tg_sample;
        result = ds18b20_get_measurement(ds18b20_hdl, &tg_sample);
        if(result != ESP_OK) {
            tg_sample = NAN;

            ESP_LOGE(TAG, "DS18B20 device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(TAG, "DS18B20 Ground Temperature:  %.2f°C", tg_sample);
        }

        /* attempt to get atmospheric pressure over i2c from bmp280 sensor */
        float pa_sample;
        result = bmp280_get_pressure(bmp280_hdl, &pa_sample);
        if(result != ESP_OK) {
            pa_sample = NAN;
            ESP_LOGE(TAG, "BMP280 device read failed (%s)", esp_err_to_name(result));
        } else {
            pa_sample = pa_sample / 100;
            ESP_LOGI(TAG, "BMP280 Atmospheric Pressure: %.2f hPa", pa_sample);
        }

    }
    /* free resources */;
    mbc_master_destroy();
    bmp280_delete( bmp280_hdl );
    ds18b20_delete( ds18b20_hdl );
    vTaskDelete( NULL );
}



/**
 * @brief Task that prints memory usage.
 * 
 * @param pvParameters 
 */
static void heap_size_task( void *pvParameters ) {
    uint32_t free_heap_size_last = 0;
    /* time-into-interval handle and configuration - */
    time_into_interval_handle_t       tii_1min_hdl;
    const time_into_interval_config_t tii_1min_cfg = {
        .name               = "tii_1min",
        .interval_type      = TIME_INTO_INTERVAL_SEC,
        .interval_period    = 60,  // 60-second interval
        .interval_offset    = 10   // 10-seconds into the interval
    };

    /* attempt to initialize a time-into-interval handle - task system clock synchronization */
    time_into_interval_init(&tii_1min_cfg, &tii_1min_hdl);
    if (tii_1min_hdl == NULL) {
        ESP_LOGE(TAG, "Unable to initialize time-into-interval handle");
        esp_restart(); 
    }

    /* enter task loop */
    for ( ;; ) {
        /* time-into-interval task delay (1-min interval with 10-second offset into the interval) */
        time_into_interval_delay(tii_1min_hdl);

        /* monitor consumed bytes for possible memory leak */
        free_heap_size_last = print_free_heap_size(free_heap_size_last);

        ESP_LOGW(TAG, "Free Stack Memory: %lu bytes (heap_size_task)", uxTaskGetStackHighWaterMark2(NULL));

        if(s_sample_sensor_task_hdl != NULL) 
            ESP_LOGW(TAG, "Free Stack Memory: %lu bytes (sample_sensor_task)", uxTaskGetStackHighWaterMark2(s_sample_sensor_task_hdl));
    }
    vTaskDelete( NULL );
}

void app_main(void) {
    /* print general startup information */
    ESP_LOGI(TAG, "Startup..");
    ESP_LOGI(TAG, "Free Memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "IDF Version: %s", esp_get_idf_version());

    /* set log levels */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    /* attempt to initialize nvs flash */
    ESP_ERROR_CHECK( nvs_init() );

    /* attempt to start wifi services */
    ESP_ERROR_CHECK( wifi_start() );

    // attempt to set system date-time and timezone
    ESP_ERROR_CHECK( sntp_start("AST4ADT,M3.2.0,M11.1.0") ); // AST4ADT,M3.2.0,M11.1.0 Atlantic-Standard-Time (Alantic)

    // print ntp time server(s) and system time
    print_sntp_time_servers(); print_system_time(); 

    /* attempt to initialize a i2c 0 master bus handle */
    i2c_new_master_bus(&s_i2c0_bus_cfg, &s_i2c0_bus_hdl);
    if (s_i2c0_bus_hdl == NULL) {
        ESP_LOGE(TAG, "Unable to initialize i2c 0 master bus handle");
        esp_restart(); 
    }

    /* attempt to instantiate one wire master bus 0 */
    ESP_ERROR_CHECK( onewire_new_bus_rmt(&s_owb0_bus_cfg, &s_owb0_rmt_cfg, &s_owb0_bus_hdl) );
    

    /* attempt to start sensor sampling task */
    xTaskCreatePinnedToCore( 
        sample_sensor_task, 
        "smp_snr_tsk", 
        (MINIMAL_STACK_SIZE * 4), 
        NULL, 
        (tskIDLE_PRIORITY + 2), 
        &s_sample_sensor_task_hdl,
        APP_CPU_NUM );

    /* attempt to start memory usage task */
    xTaskCreatePinnedToCore( 
        heap_size_task, 
        "heap_size_tsk", 
        (MINIMAL_STACK_SIZE * 3), 
        NULL, 
        (tskIDLE_PRIORITY + 2), 
        NULL, 
        APP_CPU_NUM );

}