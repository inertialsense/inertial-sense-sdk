#include <asf.h>
#include <string>
#include <stream_buffer.h>
#include <arm_math.h>

#include "sd_mmc_mem.h"
#include "wifi.h"
#include "xbee.h"
#include "globals.h"
#include "communications.h"
#include "user_interface.h"
#include "sd_card_logger.h"
#include "wheel_encoder.h"
#include "CAN.h"
#include "drivers/d_adc.h"

// RTOS Task Configuration
#define TASK_COMM_PERIOD_MS             1
#define TASK_LOGGER_PERIOD_MS           1
#define TASK_WIFI_PERIOD_MS             10
#define TASK_MAINT_PERIOD_MS            10
#define TASK_MAINT_SLOW_SEC_PERIOD_MS   1000

// #define TASK_COMM_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_COMM_STACK_SIZE            (8192/sizeof(portSTACK_TYPE))
#define TASK_MAINT_STACK_SIZE           (4096/sizeof(portSTACK_TYPE))
// #define TASK_LOGGER_STACK_SIZE          (4096/sizeof(portSTACK_TYPE))
#define TASK_LOGGER_STACK_SIZE          (8192/sizeof(portSTACK_TYPE))
#define TASK_WIFI_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))

#define TASK_COMM_PRIORITY			    (tskIDLE_PRIORITY + 4)  // Highest
#define TASK_LOGGER_PRIORITY            (tskIDLE_PRIORITY + 3)
#define TASK_WIFI_PRIORITY			    (tskIDLE_PRIORITY + 2)
#define TASK_MAINT_PRIORITY             (tskIDLE_PRIORITY + 1)

#undef printf
#define printf(...)
#define printf_mutex(...)


is_comm_instance_t& evbTaskCommInit(void *pvParameters);
void evbTaskComm(rtos_task_t &task, is_comm_instance_t &comm);

is_comm_instance_t& evbTaskLoggerInit(void *pvParameters);
void evbTaskLogger(rtos_task_t &task, is_comm_instance_t &comm);

void evbTaskMaintInit(void *pvParameters);
int evbTaskMaint(rtos_task_t &task);

void evbMainInitBoard(void);
void evbMainInitNvr(void);
void evbMainInitIO(void);
void evbMainInitComm(void);
void evbMainInitRTOS(pdTASK_CODE pxTaskComm,
				    pdTASK_CODE pxTaskLogg,
				    pdTASK_CODE pxTaskWifi,
				    pdTASK_CODE pxTaskMant );

int evbMain(void);