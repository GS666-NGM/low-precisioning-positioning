#include "FreeRTOS.h"
#include "task.h"
#include "Angle.h"
#include "usart.h"
#include "PID.h"
#include "Blueteeth.h"
#include "Position.h"

#define CAR_START_STACK                 128
#define CAR_START_PRIORITY              1
TaskHandle_t car_start_handle;
void Car_Start(void* pv);

#define DATA_TASK_STACK                 128
#define DATA_TASK_STACK_PRIORITY        4
TaskHandle_t data_task_handle;
void Data_Task(void* pv);

#define CONTROL_TASK_STACK              128
#define CONTROL_TASK_STACK_PRIORITY     3
TaskHandle_t contro_task_handle;
void Control_Task(void* pv);

#define SHOW_TASK_STACK                 128
#define SHOW_TASK_STACK_PRIORITY        2
TaskHandle_t show_task_handle;
void Show_Task(void* pv);

void Car_Init(void)
{
    xTaskCreate( (TaskFunction_t) Car_Start,
                (char *) "Car_Start", 
                (configSTACK_DEPTH_TYPE) CAR_START_STACK,
                (void *) NULL,
                (UBaseType_t) CAR_START_PRIORITY,
                (TaskHandle_t *) &car_start_handle );
    vTaskStartScheduler();
}

void Car_Start(void* pv)
{
    taskENTER_CRITICAL();
    
    xTaskCreate( (TaskFunction_t) Data_Task,
                (char *) "Data_Task", 
                (configSTACK_DEPTH_TYPE) DATA_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) DATA_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &data_task_handle );
                
    xTaskCreate( (TaskFunction_t) Control_Task,
                (char *) "Control_Task", 
                (configSTACK_DEPTH_TYPE) CONTROL_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) CONTROL_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &contro_task_handle );
                
    xTaskCreate( (TaskFunction_t) Show_Task,
                (char *) "Show_Task", 
                (configSTACK_DEPTH_TYPE) SHOW_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) SHOW_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &show_task_handle );
                
    vTaskDelete(NULL);
                
    taskEXIT_CRITICAL();
}

void Data_Task(void* pv)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while(1)
    {
        Get_Angle();
        Encoder_SpeedGet();
         if(startflag == 3)
         {
//             Get_Position();
             EKF_Position();
//             xTaskNotifyGive(contro_task_handle);
         }
            
        vTaskDelayUntil(&pxPreviousWakeTime, 10);
    }
}

void Control_Task(void* pv)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while(1)
    {
//        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(manipulationflag == 1)
            Velocity();
        Turn();
        if(navigationflag == 1)
            Distance();
        PWM_Get();
        vTaskDelayUntil(&pxPreviousWakeTime, 10);
    }
}

void Show_Task(void* pv)
{
    while(1)
    {
        rxdata_deal();
        if(positionfalg == 1)
        {
            printf("%.2f  %.2f  %.2f", px, py, yaw);
        }
        vTaskDelay(100);
    }
}