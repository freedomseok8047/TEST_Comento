/**
 * @file stm32f4xx_it.h
 * @brief Interrupt Service Routines 헤더 파일
 * @author ECU Development Team  
 * @date 2025-09-21
 */

#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Cortex-M4 Processor Interruption and Exception Handlers */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* STM32F4xx Peripheral Interrupt Handlers */

/**
 * @brief TIM6 global and DAC1&2 underrun error interrupts Handler
 * @note 1ms Task Timer 인터럽트
 */
void TIM6_DAC_IRQHandler(void);

/**
 * @brief TIM7 global interrupt Handler
 * @note 5ms Task Timer 인터럽트
 */
void TIM7_IRQHandler(void);

/**
 * @brief EXTI line3 interrupt Handler
 * @note PMIC IRQ 핀 인터럽트 (PB3)
 */
void EXTI3_IRQHandler(void);

/* DMA Interrupt Handlers */

/**
 * @brief DMA1 stream0 global interrupt Handler
 * @note I2C1 RX DMA
 */
void DMA1_Stream0_IRQHandler(void);

/**
 * @brief DMA1 stream2 global interrupt Handler
 * @note I2C2 RX DMA (예비용)
 */
void DMA1_Stream2_IRQHandler(void);

/**
 * @brief DMA1 stream3 global interrupt Handler
 * @note I2C2 TX DMA (예비용)
 */
void DMA1_Stream3_IRQHandler(void);

/**
 * @brief DMA1 stream4 global interrupt Handler
 * @note SPI2 RX DMA (예비용)
 */
void DMA1_Stream4_IRQHandler(void);

/**
 * @brief DMA1 stream6 global interrupt Handler
 * @note I2C1 TX DMA
 */
void DMA1_Stream6_IRQHandler(void);

/**
 * @brief DMA1 stream7 global interrupt Handler
 * @note SPI2 TX DMA (예비용)
 */
void DMA1_Stream7_IRQHandler(void);

/**
 * @brief DMA2 stream0 global interrupt Handler
 * @note SPI1 RX DMA (EEPROM)
 */
void DMA2_Stream0_IRQHandler(void);

/**
 * @brief DMA2 stream3 global interrupt Handler
 * @note SPI1 TX DMA (EEPROM)
 */
void DMA2_Stream3_IRQHandler(void);

/* CAN Interrupt Handlers (필요시 추가) */

/**
 * @brief CAN1 TX interrupts Handler
 */
void CAN1_TX_IRQHandler(void);

/**
 * @brief CAN1 RX0 interrupts Handler
 */
void CAN1_RX0_IRQHandler(void);

/**
 * @brief CAN1 RX1 interrupts Handler
 */
void CAN1_RX1_IRQHandler(void);

/**
 * @brief CAN1 SCE interrupt Handler
 */
void CAN1_SCE_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */