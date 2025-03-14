# Maxim-Integrated DS18B20 Sensor

[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Edited with VS Code](https://badgen.net/badge/icon/VS%20Code?icon=visualstudio&label=edited%20with)](https://visualstudio.microsoft.com)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/k0i05/library/esp_ds18b20.svg)](https://registry.platformio.org/libraries/k0i05/esp_ds18b20)
[![ESP Component Registry](https://components.espressif.com/components/k0i05/esp_ds18b20/badge.svg)](https://components.espressif.com/components/k0i05/esp_ds18b20)

This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Maxim-Integrated DS18B20 1-wire temperature sensor.  Information on features and functionality are documented and can be found in the `ds18b20.h` header file and in the `documentation` folder.

## Repository

The component is hosted on github and is located here: <https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/owb/esp_ds18b20>

## General Usage

To get started, simply copy the component to your project's `components` folder and reference the `ds18b20.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```text
components
└── esp_ds18b20
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── ds18b20_version.h
    │   └── ds18b20.h
    └── ds18b20.c
```

## Basic Example

Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings, prints DS18S20 sensors detected on the 1-wire bus, and makes a measurement request from the sensor at a user defined interval and prints the results.

```c
#include <ds18b20.h>

void owb0_ds18b20_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t                   last_wake_time = xTaskGetTickCount ();
    //
    // initialize owb device configuration
    ds18b20_config_t             dev_cfg = OWB_DS18B20_CONFIG_DEFAULT;
    ds18b20_handle_t             dev_hdl;
    onewire_device_iter_handle_t dev_iter_hdl;
    onewire_device_t             dev;
    // owb ds18b20 device detection
    onewire_device_t             devs[5];
    uint8_t                      devs_size = sizeof(devs) / sizeof(devs[0]);
    uint8_t                      devs_count = 0;
    //
    // detect ds18b20 devices on 1-wire bus
    esp_err_t ret = ds18b20_detect(owb0_bus_hdl, devs, devs_size, &devs_count);
    if(ret == ESP_OK) {
        ESP_LOGW(APP_TAG, "ds18b20 devices detected: %u", devs_count);
        for(uint8_t i = 0; i < devs_count; i++) {
            ESP_LOGI(APP_TAG, "ds18b20(%u), address: %016llX", i, devs[i].address);
        }
    } else {
        ESP_LOGE(APP_TAG, "ds18b20 device detect failed (%s)", esp_err_to_name(ret));
    }
    //
    // instantiate 1-wire device iterator handle
    ESP_ERROR_CHECK( onewire_new_device_iter(owb0_bus_hdl, &dev_iter_hdl) );
    //
    // get 1-wire device - assumes there is only one ds18b20 device on the bus
    if (onewire_device_iter_get_next(dev_iter_hdl, &dev) == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
        // check if the device is a ds18b20, if so, return the ds18b20 handle
        if (ds18b20_init(&dev, &dev_cfg, &dev_hdl) == ESP_OK) {
            ESP_LOGI(APP_TAG, "found a ds18b20, address: %016llX", dev.address);
        } else {
            ESP_LOGI(APP_TAG, "found an unknown device, address: %016llX", dev.address);
            assert(dev.address);
        }
    }
    //
    // free device iter handle
    ESP_ERROR_CHECK( onewire_del_device_iter(dev_iter_hdl) );
    //
    // get/set/get resolution
    ds18b20_resolutions_t dev_reso;
    ESP_ERROR_CHECK( ds18b20_get_resolution(dev_hdl, &dev_reso) );
    ESP_LOGW(APP_TAG, "ds18b20 get resolution: %d", dev_reso);
    ESP_ERROR_CHECK( ds18b20_set_resolution(dev_hdl, DS18B20_RESOLUTION_11BIT) );
    ESP_ERROR_CHECK( ds18b20_get_resolution(dev_hdl, &dev_reso) );
    ESP_LOGW(APP_TAG, "ds18b20 get resolution: %d", dev_reso);
    //
    // validate device handle
    if(dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ds18b20 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for( ;; ) {
        ESP_LOGI(APP_TAG, "######################## DS18B20 - START #########################");
        //
        // handle sensor
        float temperature;
        esp_err_t result = ds18b20_get_measurement(dev_hdl, &temperature);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ds18b20 device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "temperature:     %.2f°C", temperature);
        }
        //
        ESP_LOGI(APP_TAG, "######################## DS18B20 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, OWB0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    ds18b20_delete( dev_hdl );
    vTaskDelete( NULL );
}
```

Copyright (c) 2024 Eric Gionet (<gionet.c.eric@gmail.com>)
