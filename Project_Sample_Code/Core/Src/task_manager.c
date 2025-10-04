/**
 * @file task_manager.c
 * @brief Task Manager - RTOS 기반 구현
 */

#include "task_manager.h"
#include "pmic_service.h"
#include "eeprom_service.h"
#include "can_service.h"
#include "dtc_manager.h"
#include <stdio.h>

// ========== RTOS 객체 핸들 ==========
osThreadId Task1msHandle;
osThreadId Task5msHandle;
osThreadId TaskPMICHandle;

osMutexId DTC_MutexHandle;
osMutexId EEPROM_MutexHandle;
osMutexId CAN_MutexHandle;

osEventFlagsId SystemEventsHandle;

// ========== 외부 하드웨어 핸들 참조 ==========
extern ADC_HandleTypeDef hadc1;

// ========== Task 함수 선언 (내부 사용) ==========
static void Task_1ms(void const *argument);
static void Task_5ms(void const *argument);
static void Task_PMIC_Monitor(void const *argument);

// ========== 공개 함수 구현 ==========

/**
 * @brief Task Manager 초기화
 */
bool task_manager_init(void)
{
    printf("[TASK_MGR] Initializing RTOS objects\n");
    
    // ========== Mutex 생성 ==========
    osMutexDef(DTC_Mutex);
    DTC_MutexHandle = osMutexCreate(osMutex(DTC_Mutex));
    if (DTC_MutexHandle == NULL) {
        printf("[TASK_MGR] ERROR: Failed to create DTC Mutex\n");
        return false;
    }
    
    osMutexDef(EEPROM_Mutex);
    EEPROM_MutexHandle = osMutexCreate(osMutex(EEPROM_Mutex));
    if (EEPROM_MutexHandle == NULL) {
        printf("[TASK_MGR] ERROR: Failed to create EEPROM Mutex\n");
        return false;
    }
    
    osMutexDef(CAN_Mutex);
    CAN_MutexHandle = osMutexCreate(osMutex(CAN_Mutex));
    if (CAN_MutexHandle == NULL) {
        printf("[TASK_MGR] ERROR: Failed to create CAN Mutex\n");
        return false;
    }
    
    // ========== Event Flags 생성 ==========
    osEventFlagsDef(SystemEvents);
    SystemEventsHandle = osEventFlagsCreate(osEventFlags(SystemEvents));
    if (SystemEventsHandle == NULL) {
        printf("[TASK_MGR] ERROR: Failed to create Event Flags\n");
        return false;
    }
    
    printf("[TASK_MGR] RTOS objects created successfully\n");
    return true;
}

/**
 * @brief Task 생성 및 시작
 */
bool task_manager_start(void)
{
    printf("[TASK_MGR] Creating tasks\n");
    
    // Task 1ms (우선순위: High)
    osThreadDef(Task1ms, Task_1ms, osPriorityHigh, 0, 256);
    Task1msHandle = osThreadCreate(osThread(Task1ms), NULL);
    if (Task1msHandle == NULL) {
        printf("[TASK_MGR] ERROR: Failed to create Task 1ms\n");
        return false;
    }
    
    // Task 5ms (우선순위: Normal)
    osThreadDef(Task5ms, Task_5ms, osPriorityNormal, 0, 512);
    Task5msHandle = osThreadCreate(osThread(Task5ms), NULL);
    if (Task5msHandle == NULL) {
        printf("[TASK_MGR] ERROR: Failed to create Task 5ms\n");
        return false;
    }
    
    // Task PMIC Monitor (우선순위: Realtime)
    osThreadDef(TaskPMIC, Task_PMIC_Monitor, osPriorityRealtime, 0, 256);
    TaskPMICHandle = osThreadCreate(osThread(TaskPMIC), NULL);
    if (TaskPMICHandle == NULL) {
        printf("[TASK_MGR] ERROR: Failed to create PMIC Task\n");
        return false;
    }
    
    printf("[TASK_MGR] All tasks created successfully\n");
    return true;
}

/**
 * @brief PMIC IRQ 이벤트 설정 (ISR에서 호출)
 */
void task_set_pmic_irq_event(void)
{
    osEventFlagsSet(SystemEventsHandle, EVENT_PMIC_IRQ);
}

/**
 * @brief Emergency 이벤트 설정
 */
void task_set_emergency_event(void)
{
    osEventFlagsSet(SystemEventsHandle, EVENT_EMERGENCY);
}

// ========== 내부 Task 함수 구현 ==========

/**
 * @brief 1ms Task - 빠른 샘플링
 */
static void Task_1ms(void const *argument)
{
    uint32_t tick_count = 0;
    
    printf("[TASK_1MS] Started\n");
    
    for(;;)
    {
        // 1. ADC 빠른 샘플링
        if (HAL_ADC_Start(&hadc1) == HAL_OK) {
            if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
                uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
                
                // 위험 전압 감지
                if (adc_value < 1000 || adc_value > 4000) {
                    task_set_emergency_event();
                }
            }
            HAL_ADC_Stop(&hadc1);
        }
        
        // 2. PMIC IRQ 핀 직접 체크
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET) {
            task_set_pmic_irq_event();
        }
        
        // 3. 1초마다 상태 출력
        tick_count++;
        if ((tick_count % 1000) == 0) {
            printf("[TASK_1MS] Count: %lu\n", tick_count);
        }
        
        osDelay(1);  // 1ms 대기
    }
}

/**
 * @brief 5ms Task - 주기적 작업
 */
static void Task_5ms(void const *argument)
{
    uint32_t tick_count = 0;
    
    printf("[TASK_5MS] Started\n");
    
    for(;;)
    {
        tick_count++;
        
        // 1. PMIC 진단 결과 확인
        if (pmic_service_get_state() == PMIC_SERVICE_COMPLETE) {
            pmic_status_data_t pmic_data;
            if (pmic_service_get_status(&pmic_data)) {
                // DTC 분석 (Mutex 보호)
                osMutexWait(DTC_MutexHandle, osWaitForever);
                uint8_t fault_count = pmic_service_analyze_faults(&pmic_data);
                osMutexRelease(DTC_MutexHandle);
                
                if (fault_count > 0) {
                    printf("[TASK_5MS] PMIC faults: %d\n", fault_count);
                }
            }
        }
        
        // 2. CAN 주기 전송 (100ms: 5ms * 20)
        if ((tick_count % 20) == 0) {
            brake_can_data_t brake_status = {0};
            brake_status.status.system_status = 0x01;
            
            osMutexWait(DTC_MutexHandle, osWaitForever);
            brake_status.status.active_dtc_count = dtc_get_active_count();
            osMutexRelease(DTC_MutexHandle);
            
            brake_status.status.brake_pressure = 850;
            brake_status.status.temperature = 65;
            brake_status.status.power_mode = 0x01;
            
            osMutexWait(CAN_MutexHandle, osWaitForever);
            can_service_send_brake_status(&brake_status);
            osMutexRelease(CAN_MutexHandle);
        }
        
        // 3. 시스템 상태 (1초: 5ms * 200)
        if ((tick_count % 200) == 0) {
            osMutexWait(DTC_MutexHandle, osWaitForever);
            uint8_t dtc_count = dtc_get_active_count();
            osMutexRelease(DTC_MutexHandle);
            
            printf("[TASK_5MS] DTCs: %d\n", dtc_count);
        }
        
        osDelay(5);  // 5ms 대기
    }
}

/**
 * @brief PMIC Monitor Task - 이벤트 대기
 */
static void Task_PMIC_Monitor(void const *argument)
{
    printf("[TASK_PMIC] Started\n");
    
    for(;;)
    {
        // 이벤트 대기
        uint32_t flags = osEventFlagsWait(SystemEventsHandle,
                                          EVENT_PMIC_IRQ | EVENT_EMERGENCY,
                                          osFlagsWaitAny,
                                          osWaitForever);
        
        // PMIC IRQ 처리
        if (flags & EVENT_PMIC_IRQ) {
            printf("[TASK_PMIC] IRQ detected\n");
            
            if (!pmic_service_start_diagnosis()) {
                printf("[TASK_PMIC] WARNING: Diagnosis failed\n");
            }
        }
        
        // 긴급 상황 처리
        if (flags & EVENT_EMERGENCY) {
            printf("[TASK_PMIC] Emergency!\n");
            
            osMutexWait(DTC_MutexHandle, osWaitForever);
            dtc_add_code(DTC_BRAKE_SYSTEM_FAULT);
            osMutexRelease(DTC_MutexHandle);
        }
    }
}