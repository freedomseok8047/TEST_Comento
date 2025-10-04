/**
 * @file task_manager.h
 * @brief Task Manager - RTOS 기반
 */

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

// ========== RTOS 객체 핸들 (외부 접근용) ==========
extern osThreadId Task1msHandle;
extern osThreadId Task5msHandle;
extern osThreadId TaskPMICHandle;

extern osMutexId DTC_MutexHandle;
extern osMutexId EEPROM_MutexHandle;
extern osMutexId CAN_MutexHandle;

extern osEventFlagsId SystemEventsHandle;

// Event Flags
#define EVENT_PMIC_IRQ      (1 << 0)
#define EVENT_EMERGENCY     (1 << 1)

// ========== 공개 함수 ==========

/**
 * @brief Task Manager 초기화 (RTOS 객체 생성)
 */
bool task_manager_init(void);

/**
 * @brief Task 생성 및 시작
 */
bool task_manager_start(void);

/**
 * @brief PMIC IRQ 이벤트 설정 (ISR에서 호출)
 */
void task_set_pmic_irq_event(void);

/**
 * @brief Emergency 이벤트 설정
 */
void task_set_emergency_event(void);

#endif /* TASK_MANAGER_H */