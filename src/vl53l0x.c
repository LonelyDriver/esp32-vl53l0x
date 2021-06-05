/**** Includes ***********************************/
#include <stdint.h>
#include <stdio.h>
#include "vl53l0x.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#define _FREE_RTOS_

#ifdef _FREE_RTOS_
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include "FreeRTOS/semphr.h"
#else
#error "Implementation only provided for freertos. Please set _FREE_RTOS_=1 if you compile for freertos."
#endif

/**** Typedefs ************************************/

/**** Defines *************************************/

/**** Static variables ****************************/
#ifdef _FREE_RTOS_
static const char* VL53L0x_DEBUG_TAG = "VL53l0x";

static VL53L0X_Dev_t dev = {0x00};
static int measurement_running = 0;
static SemaphoreHandle_t sem = {0x00};
#endif
/**** Static prototypes ***************************/
// freertos functions
#ifdef _FREE_RTOS_
static void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    ESP_LOGE(VL53L0x_DEBUG_TAG, "API Status: %i : %s\n", Status, buf);
}

static void set_debug_level(DebugLevel level) {
    if(sem == NULL) {
        ESP_LOGI(VL53L0x_DEBUG_TAG, "VL53L0x_SetDebugLevel: VL53L0x not initialized");
        return;
    }

    if(xSemaphoreTake( sem, ( TickType_t ) 200/portTICK_PERIOD_MS ) != pdTRUE) {
        ESP_LOGI(VL53L0x_DEBUG_TAG, "VL53L0x_SetDebugLevel: Could not get semaphore");
        return;
    }
    switch(level) {
        case DEBUG_ERROR:
            esp_log_level_set(VL53L0x_DEBUG_TAG, ESP_LOG_ERROR);
            break;
        case DEBUG_WARNING:
            esp_log_level_set(VL53L0x_DEBUG_TAG, ESP_LOG_WARN);
            break;
        case DEBUG_DEBUG:
            esp_log_level_set(VL53L0x_DEBUG_TAG, ESP_LOG_DEBUG);
            break;
        case DEBUG_INFO:
            esp_log_level_set(VL53L0x_DEBUG_TAG, ESP_LOG_INFO);
            break;
        case DEBUG_VERBOSE:
            esp_log_level_set(VL53L0x_DEBUG_TAG, ESP_LOG_VERBOSE);
            break;
        default:
            printf("Invalid DebugLevel (%d)\n", level);
    }
    xSemaphoreGive(sem);
    printf("Set DebugLevel to (%d)\n", level);
}

static int init_i2c(int i2c_port, int i2c_sda, int i2c_scl) {
    i2c_port_t i2c_master_port = i2c_port;
    i2c_config_t conf={0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = i2c_sda;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = i2c_scl;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 400000;

    esp_err_t esp_err = i2c_param_config(i2c_master_port, &conf); 
    ESP_ERROR_CHECK(esp_err);
    esp_err = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    ESP_ERROR_CHECK(esp_err);

    return 0;
}

static int init_vl53l0x(int i2c_port, int i2c_addr) {
    VL53L0X_Error status = {0x00};
    VL53L0X_DeviceInfo_t dev_info = {0x00};

    if((sem=xSemaphoreCreateMutex()) == NULL) {
        ESP_LOGE(VL53L0x_DEBUG_TAG, "Could not create semaphore\n");
        return -1;
    }

    xSemaphoreGive(sem);

    if(xSemaphoreTake( sem, ( TickType_t ) 200/portTICK_PERIOD_MS ) == pdTRUE) {

        if(dev.i2c_port_num == 0) {
            dev.i2c_address = i2c_addr;
            dev.i2c_port_num = i2c_port;
            ESP_LOGI(
                VL53L0x_DEBUG_TAG, 
                "VL53L0X initialized with i2c_port_num (%d) and i2c_address (%d)\n",
                dev.i2c_port_num, dev.i2c_address);

            if(status == VL53L0X_ERROR_NONE)
            {
                ESP_LOGI(VL53L0x_DEBUG_TAG, "Call of VL53L0X_DataInit\n");
                if((status = VL53L0X_DataInit(&dev)) != VL53L0X_ERROR_NONE) {
                    goto END;
                }
            }
            
            status = VL53L0X_GetDeviceInfo(&dev, &dev_info);
            
            if(status == VL53L0X_ERROR_NONE)
            {
                ESP_LOGI(VL53L0x_DEBUG_TAG, "VL53L0X_GetDeviceInfo:\n");
                ESP_LOGI(VL53L0x_DEBUG_TAG, "Device Name : %s\n", dev_info.Name);
                ESP_LOGI(VL53L0x_DEBUG_TAG, "Device Type : %s\n", dev_info.Type);
                ESP_LOGI(VL53L0x_DEBUG_TAG, "Device ID : %s\n", dev_info.ProductId);
                ESP_LOGI(VL53L0x_DEBUG_TAG, "ProductRevisionMajor : %d\n", dev_info.ProductRevisionMajor);
                ESP_LOGI(VL53L0x_DEBUG_TAG, "ProductRevisionMinor : %d\n", dev_info.ProductRevisionMinor);

                if ((dev_info.ProductRevisionMinor != 1) && (dev_info.ProductRevisionMinor != 1)) {
                    ESP_LOGE(VL53L0x_DEBUG_TAG, "Error expected cut 1.1 but found cut %d.%d\n",
                            dev_info.ProductRevisionMajor, dev_info.ProductRevisionMinor);
                    status = VL53L0X_ERROR_NOT_SUPPORTED;
                }
            } else {
                goto END;
            }
        } else {
            ESP_LOGI(VL53L0x_DEBUG_TAG,
                "Already initialized with port (%d) on address (%d)\n",
                dev.i2c_port_num, dev.i2c_address);
            xSemaphoreGive(sem);
        }
        xSemaphoreGive(sem);
    } else {
        ESP_LOGE(VL53L0x_DEBUG_TAG, "Could not get semaphore");
        return -1;
    }

    return 0;
    END:
        print_pal_error(status);
        xSemaphoreGive(sem);
        return -1;
}

static VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

static int get_data(int16_t* data, int16_t counts) {
    VL53L0X_Error status = {0x00};
    VL53L0X_RangingMeasurementData_t pRangingMeasurementData = {0x00};
    
    if(sem == NULL) {
        ESP_LOGW(VL53L0x_DEBUG_TAG, "VL53L0x_GetData: VL53L0x not initialized");
        return -1;
    }

    if(xSemaphoreTake( sem, ( TickType_t ) 200/portTICK_PERIOD_MS ) != pdTRUE) {
        ESP_LOGI(VL53L0x_DEBUG_TAG, "VL53L0x_GetData: Could not get semaphore");
        return -1;
    }

    if((status = VL53L0X_StaticInit(&dev)) != VL53L0X_ERROR_NONE) { // Device Initialization
        // StaticInit will set interrupt by default
        goto END;
    }

    if((status = VL53L0X_SetDeviceMode(&dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING)) != VL53L0X_ERROR_NONE) { // Setup in single ranging mode
        goto END;
    }

    if((status = VL53L0X_StartMeasurement(&dev)) != VL53L0X_ERROR_NONE) {
		goto END;
    }

    uint32_t measurement;

    for(measurement=0; measurement<counts; measurement++)
    {
        measurement_running = 1;
        status = WaitMeasurementDataReady(&dev);

        if(status == VL53L0X_ERROR_NONE)
        {
            status = VL53L0X_GetRangingMeasurementData(&dev, &pRangingMeasurementData);

            *(data + measurement) = pRangingMeasurementData.RangeMilliMeter;
            //printf("In loop measurement %d: %d\n", measurement, pRangingMeasurementData.RangeMilliMeter);

            // Clear the interrupt
            VL53L0X_ClearInterruptMask(&dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
            VL53L0X_PollingDelay(&dev);
        } else {
            goto END;
        }
    }

    measurement_running = 0;
    xSemaphoreGive(sem);
    return 0;
    END:
    print_pal_error(status);
    xSemaphoreGive(sem);
    return -1;
}
#endif
/**** Public interface ****************************/
int VL53L0x_SetDebugLevel(DebugLevel level) {
    set_debug_level(level);
    return 0;
}

/**
 * @brief Initialize VL53L0X Sensor
 * 
 * @param i2c_port Some Microcontrollers support multiple i2c Channels. Parameter is Controller dependent
 * @param i2c_sda SDA Pin for I2C communication
 * @param i2c_scl SCL Pin for I2C communication
 * @param clock_speed I2C clock speed, Default is 400000
 * @return int On success 0 is returned
 */
int VL53L0x_Init(int i2c_port, int i2c_sda, int i2c_scl) {
    ESP_LOGI(VL53L0x_DEBUG_TAG,
    "Initializing VL53L0X on I2C_Port (%d), I2C_SDA (%d), I2C_SCL (%d), I2C_CLK_Speed (%d)\n",
    i2c_port, i2c_sda, i2c_scl, 400000);
    
    init_i2c(i2c_port, i2c_sda, i2c_scl);
    ESP_LOGI(VL53L0x_DEBUG_TAG, "Init i2c successfull\n");

    init_vl53l0x(i2c_port, 0x29);
    ESP_LOGI(VL53L0x_DEBUG_TAG, "Init VL53L0X successfull\n");

    return 0;
}

int VL53L0x_GetData(int16_t* data, int16_t counts) {
    return get_data(data, counts);
}