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
 * @file modbus_connect.c
 *
 * MODBUS master connection to RS2E RTU libary
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <modbus_connect.h>
#include <modbus_params.h>  // for modbus parameters structures
#include <mbcontroller.h>

#include <esp_system.h>
#include <esp_event.h>
#include <esp_check.h>
#include <esp_log.h>
#include <esp_types.h>
#include <driver/gpio.h>

/**
 * @brief MODBUS controller UART definitions
 * 
 *
 * MODBUS controller UART configuration for RTU interface
 */
#define MB_PORT_NUM     (UART_NUM_1)   // Number of UART port used for Modbus connection
#define MB_UART_TXD     (GPIO_NUM_16)  // RS485 TXD GPIO number
#define MB_UART_RXD     (GPIO_NUM_15)  // RS485 RXD GPIO number
#define MB_UART_SPEED   (9600)         // The communication speed of the UART

/**
 * @brief macro definitions
 */

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))

// Discrete offset macro
#define DISCR_OFFSET(field) ((uint16_t)(offsetof(discrete_reg_params_t, field) + 1))

#define STR(fieldname) ((const char*)( fieldname ))

// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

static const char *TAG = "modbus_connect";

/**
 * @brief Enumeration of modbus device addresses accessed by master device.
 */
enum {
    MB_DEVICE_ADDR1 = 1,    // RS2E RTU
    MB_DEVICE_ADDR10 = 10   // WDC2E RTU
};

/**
 * @brief Enumeration of all supported CIDs for device (used in parameter definition table).
 */
enum {
    CID_RS2E_HOLD_DATA_0 = 0,   // RS2E temperature 32-bit float (register 5 & 6)
    CID_RS2E_HOLD_DATA_1,       // RS2E humidity 32-bit float (register 7 & 8)
    CID_RS2E_HOLD_DATA_2,       // RS2E precipitation type 16-bit uint (register 12)
    CID_RS2E_HOLD_DATA_3,       // RS2E precipitation intensity 32-bit float (register 13 & 14)
    CID_WDC2E_HOLD_DATA_4,      // WDC2E wind direction 16-bit uint (register 2)
    CID_WDC2E_HOLD_DATA_5       // WDC2E wind speed 32-bit float (register 3 & 4)
};

// Example Data (Object) Dictionary for Modbus parameters:
// The CID field in the table must be unique.
// Modbus Slave Addr field defines slave address of the device with correspond parameter.
// Modbus Reg Type - Type of Modbus register area (Holding register, Input Register and such).
// Reg Start field defines the start Modbus register number and Reg Size defines the number of registers for the characteristic accordingly.
// The Instance Offset defines offset in the appropriate parameter structure that will be used as instance to save parameter value.
// Data Type, Data Size specify type of the characteristic and its data size.
// Parameter Options field specifies the options that can be used to process parameter value (limits or masks).
// Access Mode - can be used to implement custom options for processing of characteristic (Read/Write restrictions, factory mode values and etc).
static const mb_parameter_descriptor_t s_device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}

    { CID_RS2E_HOLD_DATA_0, STR("Temperature"), STR("°C"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 4, 2,         // RS2E register 5-6
            HOLD_OFFSET(holding_data0), PARAM_TYPE_FLOAT, 4, OPTS( -40, 80, 1 ), PAR_PERMS_READ },
    { CID_RS2E_HOLD_DATA_1, STR("Humidity"), STR("%RH"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 6, 2,           // RS2E register 7-8
            HOLD_OFFSET(holding_data1), PARAM_TYPE_FLOAT, 4, OPTS( 0, 100, 1 ), PAR_PERMS_READ },
    { CID_RS2E_HOLD_DATA_2, STR("Precip_Type"), STR("CODE"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 11, 1,      // RS2E register 12
            HOLD_OFFSET(holding_data2), PARAM_TYPE_U16, 2, OPTS( -1, 7, 1 ), PAR_PERMS_READ },
    { CID_RS2E_HOLD_DATA_3, STR("Precip_Rate"), STR("MM/H"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 12, 2,      // RS2E register 13-14
            HOLD_OFFSET(holding_data3), PARAM_TYPE_FLOAT, 4, OPTS( 0, 65535, 1 ), PAR_PERMS_READ },
    
    { CID_WDC2E_HOLD_DATA_4, STR("Wind_Dir"), STR("°"), MB_DEVICE_ADDR10, MB_PARAM_HOLDING, 1, 1,           // WDC2E register 2
            HOLD_OFFSET(holding_data4), PARAM_TYPE_U16, 2, OPTS( -1, 361, 1 ), PAR_PERMS_READ },
    { CID_WDC2E_HOLD_DATA_5, STR("Wind_Speed"), STR("M/S"), MB_DEVICE_ADDR10, MB_PARAM_HOLDING, 2, 2,       // WDC2E register 3-4
            HOLD_OFFSET(holding_data5), PARAM_TYPE_FLOAT, 4, OPTS( -1, 61, 1 ), PAR_PERMS_READ },
};

// Number of parameters in the data object dictionary table
static const uint16_t s_num_device_parameters = (sizeof(s_device_parameters)/sizeof(s_device_parameters[0]));

/**
 * @brief Gets the pointer to parameter storage (instance) according to parameter description table.
 * 
 * @param param_descriptor Parameter descriptor from parameter description table.
 * @return void* Parameter storage instance.
 */
static inline void* master_get_param_data(const mb_parameter_descriptor_t* param_descriptor) {
    assert(param_descriptor != NULL);
    void* instance_ptr = NULL;
    if (param_descriptor->param_offset != 0) {
       switch(param_descriptor->mb_param_type)
       {
           case MB_PARAM_HOLDING:
               instance_ptr = ((void*)&holding_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_INPUT:
               instance_ptr = NULL;
               break;
           case MB_PARAM_COIL:
               instance_ptr = NULL;
               break;
           case MB_PARAM_DISCRETE:
               instance_ptr = NULL;
               break;
           default:
               instance_ptr = NULL;
               break;
       }
    } else {
        ESP_LOGE(TAG, "Wrong parameter offset for CID #%u", (unsigned)param_descriptor->cid);
        assert(instance_ptr != NULL);
    }
    return instance_ptr;
}

/**
 * @brief Gets CID data from parameter descriptor from device parameter table.
 * 
 * @note CID parameter type must be a holding register.
 * 
 * @param cid Index of the CID.
 * @param data_ptr CID data parameter instance.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t master_get_cid_data(const uint16_t cid, void** data_ptr) {
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    bool in_range = true; uint8_t type = 0;

    ESP_RETURN_ON_ERROR(mbc_master_get_cid_info(cid, &param_descriptor), TAG, "Unable to get cid information, master get cid data failed");
    ESP_RETURN_ON_FALSE(param_descriptor, ESP_ERR_INVALID_STATE, TAG, "Invalid param descriptor state, master get cid data failed");

    void* out_data_ptr = master_get_param_data(param_descriptor);
    ESP_RETURN_ON_FALSE(out_data_ptr, ESP_ERR_INVALID_STATE, TAG, "Invalid param data state, master get cid data failed");

    ESP_RETURN_ON_ERROR(mbc_master_get_parameter(cid, (char*)param_descriptor->param_key, (uint8_t*)out_data_ptr, &type),TAG, "Unable to get parameter, master get cid data failed");

    ESP_RETURN_ON_FALSE(param_descriptor->mb_param_type == MB_PARAM_HOLDING, ESP_ERR_INVALID_RESPONSE, TAG, "Invalid param type, master get cid data failed");

    float value = *(float*)out_data_ptr;

    if (((value > param_descriptor->param_opts.max) || (value < param_descriptor->param_opts.min))) {
        in_range = false;
    }

    ESP_RETURN_ON_FALSE(in_range, ESP_ERR_INVALID_SIZE, TAG, "Value is out of range, master get cid data failed");

    *data_ptr = out_data_ptr;

    return ESP_OK;
}

esp_err_t master_get_rs2e_temperature(float *value) {
    void* data_ptr;
    ESP_RETURN_ON_ERROR(master_get_cid_data(0, &data_ptr), TAG, "Unable to get cid data, master get temperature from RS2E failed");
    *value = *(float*)data_ptr;
    return ESP_OK;
}

esp_err_t master_get_rs2e_humidity(float *value) {
    void* data_ptr;
    ESP_RETURN_ON_ERROR(master_get_cid_data(1, &data_ptr), TAG, "Unable to get cid data, master get humidity from RS2E failed");
    *value = *(float*)data_ptr;
    return ESP_OK;
}

esp_err_t master_get_rs2e_precipitation_type(uint16_t *value) {
    void* data_ptr;
    ESP_RETURN_ON_ERROR(master_get_cid_data(2, &data_ptr), TAG, "Unable to get cid data, master get precipitation type from RS2E failed");
    *value = *(uint16_t*)data_ptr;
    return ESP_OK;
}

esp_err_t master_get_rs2e_precipitation_rate(float *value) {
    void* data_ptr;
    ESP_RETURN_ON_ERROR(master_get_cid_data(3, &data_ptr), TAG, "Unable to get cid data, master get precipitation rate from RS2E failed");
    *value = *(float*)data_ptr;
    return ESP_OK;
}

esp_err_t master_get_wdc2e_wind_direction(uint16_t *value) {
    void* data_ptr;
    ESP_RETURN_ON_ERROR(master_get_cid_data(4, &data_ptr), TAG, "Unable to get cid data, master get wind direction from WDC2E failed");
    *value = *(uint16_t*)data_ptr;
    return ESP_OK;
}

esp_err_t master_get_wdc2e_wind_speed(float *value) {
    void* data_ptr;
    ESP_RETURN_ON_ERROR(master_get_cid_data(5, &data_ptr), TAG, "Unable to get cid data, master get wind speed from WDC2E failed");
    *value = *(float*)data_ptr;
    return ESP_OK;
}

esp_err_t master_init(void) {
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
            .port =     MB_PORT_NUM,
            .mode =     MB_MODE_RTU,
            .baudrate = MB_UART_SPEED,
            .parity =   MB_PARITY_NONE
    };
    void* master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MB_RETURN_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, TAG,
                                "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller initialization fail, returns(0x%x).", (int)err);
    err = mbc_master_setup((void*)&comm);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller setup fail, returns(0x%x).", (int)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, MB_UART_TXD, MB_UART_RXD,
                              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
        "mb serial set pin failure, uart_set_pin() returned (0x%x).", (int)err);

    err = mbc_master_start();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller start fail, returned (0x%x).", (int)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (int)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&s_device_parameters[0], s_num_device_parameters);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                                "mb controller set descriptor fail, returns(0x%x).", (int)err);
    ESP_LOGI(TAG, "Modbus master stack initialized...");
    
    return err;
}