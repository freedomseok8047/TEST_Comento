/**
 * @file task_manager.h
 * @brief Task Manager 헤더 파일 - 실제 프로젝트 로직용
 * @author ECU Development Team
 * @date 2025-09-21
 */

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/**
 * @brief Task Manager 초기화
 * @return true 성공, false 실패
 */
bool task_manager_init(void);

/**
 * @brief Main Loop에서 호출되는 Task 스케줄러
 */
void task_scheduler(void);

/**
 * @brief PMIC IRQ 플래그 설정 (GPIO EXTI에서 호출)
 */
void task_set_pmic_irq_flag(void);

/**
 * @brief 1ms Timer ISR에서 호출되는 핸들러
 */
void task_1ms_isr_handler(void);

/**
 * @brief 5ms Timer ISR에서 호출되는 핸들러
 */
void task_5ms_isr_handler(void);

/**
 * @brief Timer 시작
 */
bool task_manager_start_timers(void);

/**
 * @brief Timer 정지
 */
void task_manager_stop_timers(void);

#endif /* TASK_MANAGER_H */