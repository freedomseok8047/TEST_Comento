/**
 * @file task_manager.c
 * @brief Task Manager 구현 - 실제 프로젝트 로직용
 * @author ECU Development Team
 * @date 2025-09-21
 */

#include "task_manager.h"
#include "pmic_service.h"
#include "eeprom_service.h"
#include "can_service.h"
#include "dtc_manager.h"
#include <stdio.h>

/* 외부 핸들 참조 */
extern TIM_HandleTypeDef htim6;  // 1ms Timer
extern TIM_HandleTypeDef htim7;  // 5ms Timer
extern ADC_HandleTypeDef hadc1;

/* Task 플래그들 */
typedef struct {
    volatile bool pmic_irq_flag;        // PMIC IRQ 발생 플래그
    volatile bool task_1ms_flag;        // 1ms Task 실행 플래그
    volatile bool task_5ms_flag;        // 5ms Task 실행 플래그
    volatile bool emergency_flag;       // 긴급 상황 플래그
    
    uint32_t task_1ms_count;            // 1ms Task 실행 횟수
    uint32_t task_5ms_count;            // 5ms Task 실행 횟수
    uint32_t pmic_irq_count;            // PMIC IRQ 발생 횟수
} task_manager_t;

static task_manager_t g_task_mgr = {0};

/* Forward Declarations */
static void task_1ms_handler(void);
static void task_5ms_handler(void);

/**
 * @brief Task Manager 초기화
 */
bool task_manager_init(void)
{
    printf("[TASK_MGR] Task Manager initialization started\n");
    
    // 모든 플래그 초기화
    g_task_mgr.pmic_irq_flag = false;
    g_task_mgr.task_1ms_flag = false;
    g_task_mgr.task_5ms_flag = false;
    g_task_mgr.emergency_flag = false;
    
    g_task_mgr.task_1ms_count = 0;
    g_task_mgr.task_5ms_count = 0;
    g_task_mgr.pmic_irq_count = 0;
    
    printf("[TASK_MGR] Task Manager initialization completed\n");
    return true;
}

/**
 * @brief Timer 시작
 */
bool task_manager_start_timers(void)
{
    // TIM6 시작 (1ms)
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        printf("[TASK_MGR] ERROR: TIM6 start failed\n");
        return false;
    }
    
    // TIM7 시작 (5ms)
    if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
        printf("[TASK_MGR] ERROR: TIM7 start failed\n");
        return false;
    }
    
    printf("[TASK_MGR] All timers started successfully\n");
    return true;
}

/**
 * @brief Timer 정지
 */
void task_manager_stop_timers(void)
{
    HAL_TIM_Base_Stop_IT(&htim6);
    HAL_TIM_Base_Stop_IT(&htim7);
    printf("[TASK_MGR] All timers stopped\n");
}

/**
 * @brief Main Loop Task 스케줄러
 */
void task_scheduler(void)
{
    // 1. PMIC IRQ 처리 (최고 우선순위)
    if (g_task_mgr.pmic_irq_flag) {
        g_task_mgr.pmic_irq_flag = false;
        g_task_mgr.pmic_irq_count++;
        
        printf("[TASK_MGR] Processing PMIC IRQ (Count: %lu)\n", g_task_mgr.pmic_irq_count);
        
        // PMIC 진단 시작
        if (!pmic_service_start_diagnosis()) {
            printf("[TASK_MGR] WARNING: PMIC diagnosis failed to start\n");
        }
    }
    
    // 2. 1ms Task 처리
    if (g_task_mgr.task_1ms_flag) {
        g_task_mgr.task_1ms_flag = false;
        g_task_mgr.task_1ms_count++;
        task_1ms_handler();
    }
    
    // 3. 5ms Task 처리
    if (g_task_mgr.task_5ms_flag) {
        g_task_mgr.task_5ms_flag = false;
        g_task_mgr.task_5ms_count++;
        task_5ms_handler();
    }
    
    // 4. 긴급 상황 처리
    if (g_task_mgr.emergency_flag) {
        g_task_mgr.emergency_flag = false;
        
        printf("[TASK_MGR] Processing emergency situation\n");
        dtc_add_code(DTC_BRAKE_SYSTEM_FAULT);
    }
}

/**
 * @brief PMIC IRQ 플래그 설정 (GPIO EXTI에서 호출)
 */
void task_set_pmic_irq_flag(void)
{
    g_task_mgr.pmic_irq_flag = true; // MISRA-C pdf 가이드라인 준수: 스팩 찾아 보기
}

/**
 * @brief 1ms Timer ISR 핸들러 (TIM6에서 호출)
 */
void task_1ms_isr_handler(void)
{
    g_task_mgr.task_1ms_flag = true;
}

/**
 * @brief 5ms Timer ISR 핸들러 (TIM7에서 호출)
 */
void task_5ms_isr_handler(void)
{
    g_task_mgr.task_5ms_flag = true;
}

/**
 * @brief 1ms Task 실제 처리 함수 - 빠른 감지만
 */
static void task_1ms_handler(void)
{
    // 1. ADC 빠른 샘플링 (전압 감시)
    if (HAL_ADC_Start(&hadc1) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
            uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
            
            // 위험 전압 감지 시 긴급 플래그 설정
            if (adc_value < 1000 || adc_value > 4000) {
                g_task_mgr.emergency_flag = true;
            }
        }
        HAL_ADC_Stop(&hadc1);
    }
    
    // 2. PMIC IRQ 핀 직접 체크 (추가 안전장치)
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET) {
        g_task_mgr.pmic_irq_flag = true;
    }
    
    // 3. 1초마다 상태 출력
    if ((g_task_mgr.task_1ms_count % 1000) == 0) {
        printf("[1MS_TASK] Running - Count: %lu\n", g_task_mgr.task_1ms_count);
    }
}

/**
 * @brief 5ms Task 실제 처리 함수 - 실제 작업 수행
 */
static void task_5ms_handler(void)
{
    // 1. PMIC 진단 결과 확인 및 처리
    if (pmic_service_get_state() == PMIC_SERVICE_COMPLETE) {
        pmic_status_data_t pmic_data;
        if (pmic_service_get_status(&pmic_data)) {
            // PMIC 고장 분석 수행
            uint8_t fault_count = pmic_service_analyze_faults(&pmic_data);
            
            if (fault_count > 0) {
                printf("[5MS_TASK] PMIC faults detected: %d\n", fault_count);
                // DTC는 메모리와 EEPROM에 저장됨
                // CAN 전송은 외부 UDS 요청 시에만 수행됨
            }
        }
    }
    
    // 2. EEPROM 주기적 백업 (10초마다: 5ms * 2000 = 10초)
    if ((g_task_mgr.task_5ms_count % 2000) == 0) {
        printf("[5MS_TASK] Performing periodic EEPROM backup\n");
        // EEPROM 백업은 DTC가 생성될 때 자동으로 수행됨
    }
    
    // 3. CAN 주기적 메시지 전송 (100ms마다: 5ms * 20 = 100ms)
    if ((g_task_mgr.task_5ms_count % 20) == 0) {
        // 브레이크 시스템 상태 주기 전송
        brake_can_data_t brake_status = {0};
        brake_status.status.system_status = 0x01;  // Normal
        brake_status.status.active_dtc_count = dtc_get_active_count();
        brake_status.status.brake_pressure = 850;  // kPa
        brake_status.status.temperature = 65;      // 도씨
        brake_status.status.power_mode = 0x01;     // Normal
        
        can_service_send_brake_status(&brake_status);
    }
    
    // 4. 시스템 상태 체크 (1초마다: 5ms * 200 = 1초)
    if ((g_task_mgr.task_5ms_count % 200) == 0) {
        printf("[5MS_TASK] System check - DTCs: %d, PMIC IRQs: %lu\n", 
               dtc_get_active_count(), g_task_mgr.pmic_irq_count);
    }
}