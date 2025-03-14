# ESP Time-Into-Interval Scheduler

[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Edited with VS Code](https://badgen.net/badge/icon/VS%20Code?icon=visualstudio&label=edited%20with)](https://visualstudio.microsoft.com)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/k0i05/library/esp_time_into_interval.svg)](https://registry.platformio.org/libraries/k0i05/esp_time_into_interval)
[![ESP Component Registry](https://components.espressif.com/components/k0i05/esp_time_into_interval/badge.svg)](https://components.espressif.com/components/k0i05/esp_time_into_interval)

The ESP time-into-interval component synchronizes a FreeRTOS task with the system clock with user-defined time interval for temporal conditional scenarios.

## Repository

The component is hosted on github and is located here: <https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/schedule/esp_time_into_interval>

## General Usage

To get started, simply copy the component to your project's `components` folder and reference the `time_into_interval.h` header file as an include.

```text
components
└── esp_time_into_interval
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── include
    │   └── time_into_interval_version.h
    │   └── time_into_interval.h
    └── time_into_interval.c
```

## Time-Into-Interval Example

See **Data-Table Example**, declare a time-into-interval handle within the data-table sampling task sub-routine, and create a time-into-interval instance.  The time-into-interval instance interval is every 5-minutes with 0-minutes into the interval.

The next step is declaring a conditional statement leveraging the `time_into_interval` function.  This function returns true when the configured interval condition is valid, otherwise, it returns false.  In this example, the time-into-interval function will output the data-table in json format every 5-minutes with a 10-second offset into the interval (i.e. 12:00:10, 12:05:10, 12:10:10, etc.).

```c
static void dt_1min_smp_task( void *pvParameters ) {
    time_into_interval_handle_t dt_1min_tii_5min_hdl;
    time_into_interval_config_t dt_1min_tii_5min_cfg = {
        .interval_type      = TIME_INTO_INTERVAL_SEC,
        .interval_period    = 5 * 60,
        .interval_offset    = 10
    };

    // create a new time-into-interval handle - task system clock synchronization
    time_into_interval_new(&dt_1min_tii_5min_cfg, &dt_1min_tii_5min_hdl);
    if (dt_1min_tii_5min_hdl == NULL) ESP_LOGE(APP_TAG, "time_into_interval_new, new time-into-interval handle failed"); 
    

    for ( ;; ) {
        /* delay data-table sampling task until sampling interval has lapsed */
        datatable_sampling_task_delay(dt_1min_hdl);

        /* get measurement samples from sensors and set sensor variables (pa and ta)  */

        // push samples onto the data buffer stack for processing
        datatable_push_float_sample(dt_1min_hdl, dt_1min_pa_avg_col_index, pa_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_ta_avg_col_index, ta_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_ta_min_col_index, ta_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_ta_max_col_index, ta_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_td_avg_col_index, td_samples[samples_index]);

        // process data buffer stack samples (i.e. data-table's configured processing interval)
        datatable_process_samples(dt_1min_hdl);

        /* serialize data-table and output in json format every 5-minutes (i.e. 12:00:00, 12:05:00, 12:10:00, etc.) */
        if(time_into_interval(dt_1min_tii_5min_hdl)) {
            // create root object for data-table
            cJSON *dt_1min_json = cJSON_CreateObject();

            // convert the data-table to json object
            datatable_to_json(dt_1min_hdl, &dt_1min_json);

            // render json data-table object to text and print
            char *dt_1min_json_str = cJSON_Print(dt_1min_json);
            ESP_LOGI(APP_TAG, "JSON Data-Table:\n%s",dt_1min_json_str);

            // free-up json resources
            cJSON_free(dt_1min_json_str);
            cJSON_Delete(dt_1min_json);
        }
    }

    // free up task resources
    time_into_interval_del( dt_1min_tii_5min_hdl ); //delete time-into-interval handle
    vTaskDelete( NULL );
}
```

You can declare as many time-into-interval handles as needed, or as memory permits, to sychronize real-time events with the system clock.  See time-into-interval component and review documentation on features implemented to date.

Copyright (c) 2024 Eric Gionet (<gionet.c.eric@gmail.com>)

