# NVS Component Extension

[![K0I05](https://img.shields.io/badge/K0I05-a9a9a9?logo=data:image/svg%2bxml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIxODgiIGhlaWdodD0iMTg3Ij48cGF0aCBmaWxsPSIjNDU0QjU0IiBkPSJNMTU1LjU1NSAyMS45M2MxOS4yNzMgMTUuOTggMjkuNDcyIDM5LjM0NSAzMi4xNjggNjMuNzg5IDEuOTM3IDIyLjkxOC00LjU1MyA0Ni42Ni0xOC44NDggNjQuNzgxQTUwOS40NzggNTA5LjQ3OCAwIDAgMSAxNjUgMTU1bC0xLjQ4NCAxLjg4M2MtMTMuMTk2IDE2LjUzMS0zNS41NTUgMjcuMjE1LTU2LjMzOSAyOS45MDItMjguMzEyIDIuOC01Mi4yNTUtNC43MzctNzQuNzMyLTIxLjcxNUMxMy4xNzIgMTQ5LjA5IDIuOTczIDEyNS43MjUuMjc3IDEwMS4yODEtMS42NiA3OC4zNjMgNC44MyA1NC42MjEgMTkuMTI1IDM2LjVBNTA5LjQ3OCA1MDkuNDc4IDAgMCAxIDIzIDMybDEuNDg0LTEuODgzQzM3LjY4IDEzLjU4NiA2MC4wNCAyLjkwMiA4MC44MjMuMjE1YzI4LjMxMi0yLjggNTIuMjU1IDQuNzM3IDc0LjczMiAyMS43MTVaIi8+PHBhdGggZmlsbD0iI0ZERkRGRCIgZD0iTTExOS44NjcgNDUuMjdDMTI4LjkzMiA1Mi4yNiAxMzMuODIgNjMgMTM2IDc0Yy42MyA0Ljk3Mi44NDIgOS45NTMuOTUzIDE0Ljk2LjA0NCAxLjkxMS4xMjIgMy44MjIuMjAzIDUuNzMxLjM0IDEyLjIxLjM0IDEyLjIxLTMuMTU2IDE3LjMwOWE5NS42MDQgOTUuNjA0IDAgMCAxLTQuMTg4IDMuNjI1Yy00LjUgMy43MTctNi45NzQgNy42ODgtOS43MTcgMTIuODAzQzEwNi45NCAxNTIuNzkyIDEwNi45NCAxNTIuNzkyIDk3IDE1N2MtMy40MjMuNTkyLTUuODAxLjY4NS04Ljg3OS0xLjA3NC05LjgyNi03Ljg4LTE2LjAzNi0xOS41OS0yMS44NTgtMzAuNTEyLTIuNTM0LTQuNTc1LTUuMDA2LTcuMjEtOS40NjYtMTAuMDItMy43MTQtMi44ODItNS40NS02Ljk4Ni02Ljc5Ny0xMS4zOTQtLjU1LTQuODg5LS41NjEtOS4zMTYgMS0xNCAuMDkzLTEuNzYzLjE4Mi0zLjUyNy4yMzktNS4yOTIuNDkxLTEzLjg4NCAzLjg2Ni0yNy4wNTcgMTQuMTU2LTM3LjAyOCAxNy4yMTgtMTQuMzM2IDM1Ljg1OC0xNS4wNjYgNTQuNDcyLTIuNDFaIi8+PHBhdGggZmlsbD0iI0M2RDVFMCIgZD0iTTEwOSAzOWMxMS43MDMgNS4yNTUgMTkuMjA2IDEzLjE4NiAyNC4yOTMgMjUuMDA0IDIuODU3IDguMjQgMy40NyAxNi4zMTYgMy42NiAyNC45NTYuMDQ0IDEuOTExLjEyMiAzLjgyMi4yMDMgNS43MzEuMzQgMTIuMjEuMzQgMTIuMjEtMy4xNTYgMTcuMzA5YTk1LjYwNCA5NS42MDQgMCAwIDEtNC4xODggMy42MjVjLTQuNSAzLjcxNy02Ljk3NCA3LjY4OC05LjcxNyAxMi44MDNDMTA2LjgwNCAxNTMuMDQxIDEwNi44MDQgMTUzLjA0MSA5NyAxNTdjLTIuMzMyLjA3OC00LjY2OC4wOS03IDBsMi4xMjUtMS44NzVjNS40My01LjQ0NSA4Ljc0NC0xMi41NzcgMTEuNzU0LTE5LjU1OWEzNDkuNzc1IDM0OS43NzUgMCAwIDEgNC40OTYtOS44NzlsMS42NDgtMy41NWMyLjI0LTMuNTU1IDQuNDEtNC45OTYgNy45NzctNy4xMzcgMi4zMjMtMi42MSAyLjMyMy0yLjYxIDQtNWwtMyAxYy0yLjY4LjE0OC01LjMxOS4yMy04IC4yNWwtMi4xOTUuMDYzYy01LjI4Ny4wMzktNS4yODcuMDM5LTcuNzc4LTEuNjUzLTEuNjY2LTIuNjkyLTEuNDUzLTQuNTYtMS4wMjctNy42NiAyLjM5NS00LjM2MiA0LjkyNC04LjA0IDkuODI4LTkuNTcgMi4zNjQtLjQ2OCA0LjUxNC0uNTI4IDYuOTIyLS40OTNsMi40MjIuMDI4TDEyMSA5MmwtMS0yYTkyLjc1OCA5Mi43NTggMCAwIDEtLjM2LTQuNTg2QzExOC42IDY5LjYzMiAxMTYuNTE3IDU2LjA5NCAxMDQgNDVjLTUuOTA0LTQuNjY0LTExLjYtNi4wODgtMTktNyA3LjU5NC00LjI2NCAxNi4yMjMtMS44MSAyNCAxWiIvPjxwYXRoIGZpbGw9IiM0OTUwNTgiIGQ9Ik03NyA5MmM0LjYxMyAxLjY3MSA3LjI2IDMuOTQ1IDEwLjA2MyA3LjkzOCAxLjA3OCAzLjUyMy45NzYgNS41NDYtLjA2MyA5LjA2Mi0yLjk4NCAyLjk4NC02LjI1NiAyLjM2OC0xMC4yNSAyLjM3NWwtMi4yNzcuMDc0Yy01LjI5OC4wMjgtOC4yNTQtLjk4My0xMi40NzMtNC40NDktMi44MjYtMy41OTctMi40MTYtNy42MzQtMi0xMiA0LjUwMi00LjcyOCAxMC45OS0zLjc2IDE3LTNaIi8+PHBhdGggZmlsbD0iIzQ4NEY1NyIgZD0ibTExOCA5MS43NSAzLjEyNS0uMDc4YzMuMjU0LjM3MSA0LjU5NyAxLjAwMiA2Ljg3NSAzLjMyOC42MzkgNC4yMzEuMjkgNi40NDItMS42ODggMTAuMjUtMy40MjggNC4wNzgtNS44MjcgNS41OTgtMTEuMTk1IDYuMTQ4LTEuNDE0LjAwOC0yLjgyOCAwLTQuMjQyLS4wMjNsLTIuMTY4LjAzNWMtMi45OTgtLjAxNy01LjE1Ny0uMDMzLTcuNjcyLTEuNzU4LTEuNjgxLTIuNjg0LTEuNDYtNC41NTItMS4wMzUtNy42NTIgMi4zNzUtNC4zMjUgNC44OTQtOC4wMDkgOS43NS05LjU1OSAyLjc3Ny0uNTQ0IDUuNDItLjY0OSA4LjI1LS42OTFaIi8+PHBhdGggZmlsbD0iIzUyNTg2MCIgZD0iTTg2IDEzNGgxNmwxIDRjLTIgMi0yIDItNS4xODggMi4yNjZMOTQgMTQwLjI1bC0zLjgxMy4wMTZDODcgMTQwIDg3IDE0MCA4NSAxMzhsMS00WiIvPjwvc3ZnPg==)](https://github.com/K0I05)
[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Language](https://img.shields.io/badge/Language-C-navy.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-red.svg)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
[![Edited with VS Code](https://badgen.net/badge/icon/VS%20Code?icon=visualstudio&label=edited%20with)](https://code.visualstudio.com/)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/k0i05/library/esp_nvs_ext.svg)](https://registry.platformio.org/libraries/k0i05/esp_nvs_ext)
[![ESP Component Registry](https://components.espressif.com/components/k0i05/esp_nvs_ext/badge.svg)](https://components.espressif.com/components/k0i05/esp_nvs_ext)

The NVS extension simplifies the process of reading and writing information to non-volatile storage.  The NVS extension supports common data-types such as `uint8_t` to `uint64_t`, `int8_t` to `int64_t`, `float` and `double`, `string`, and `struct`.  The library can be used as is but you can implement wrappers if the information will be stored to a common key to reference the key-value pairs.

## Repository

The component is hosted on github and is located here: <https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/b34d78b383ce366b4954c946b7181bafb1c9ebb3/components/storage/esp_nvs_ext>

## General Usage

To get started, simply copy the component to your project's `components` folder and reference the `nvs_ext.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```text
components
└── esp_nvs_ext
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── nvs_ext_version.h
    │   └── nvs_ext.h
    └── nvs_ext.c
```

## Basic Example

Once the component is referenced as an include, the extended i2c functions should be visible and available for usage.  A wrapper example for a `struct` data-type is presented below.  The example uses a `struct` to preserve system state information that is updated on system boot-up.

```c
#include <nvs_ext.h>

/* System state structure */
typedef struct system_state_tag {
    uint16_t    reboot_counter;         /*!< number of times system has restarted */
    uint64_t    reboot_timestamp;       /*!< system restart unix epoch timestamp (UTC) in seconds */
    uint64_t    system_uptime;          /*!< up-time in seconds since system restart */
} system_state_t;

/* global variables */
static system_state_t  *s_system_state = NULL;
```

The `struct` wrapper functions are declared below using a common key named `system_state`.

```c
static inline esp_err_t nvs_write_system_state(system_state_t *system_state) {
    return nvs_write_struct("system_state", system_state, sizeof(system_state_t));
}

static inline esp_err_t nvs_read_system_state(system_state_t **system_state) {
    return nvs_read_struct("system_state", (void **)system_state, sizeof(system_state_t));
}
```

The system state `struct` is initialized and saved to NVS on system boot-up as shown below.

```c
static inline void init_system_state(void) {
    s_system_state = (system_state_t*)calloc(1, sizeof(system_state_t));

    /* attempt to read system state structure from nvs */
    if(nvs_read_system_state(&s_system_state) != ESP_OK) {
        /* assume system state structure doesn't exist - initialize attributes */
        s_system_state->reboot_counter  += 1;
        s_system_state->system_uptime    = time_into_interval_get_epoch_timestamp();
        s_system_state->reboot_timestamp = time_into_interval_get_epoch_timestamp();
    } else {
        /* assume system state structure exists - update attributes */
        s_system_state->reboot_counter  += 1;
        s_system_state->system_uptime    = time_into_interval_get_epoch_timestamp() - s_system_state->reboot_timestamp;
        s_system_state->reboot_timestamp = time_into_interval_get_epoch_timestamp();
    }

    /* attempt to write system state structure to nvs */
    nvs_write_system_state(s_system_state);

    /* print system state structure */
    ESP_LOGW(TAG, "Boot Count: %u", s_system_state->reboot_counter);
    ESP_LOGW(TAG, "Up-Time:    %llu", s_system_state->system_uptime);
    ESP_LOGW(TAG, "Timestamp:  %llu", s_system_state->reboot_timestamp);
}
```

Copyright (c) 2024 Eric Gionet (<gionet.c.eric@gmail.com>)
