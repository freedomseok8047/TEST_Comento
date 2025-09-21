/**
 * @file stm32f4xx_it.c
 * @brief Interrupt Service Routines 구현 - 완전한 버전
 * @author ECU Development Team
 * @date 2025-09-21
 */

#include "main.h"
#include "stm32f4xx_it.h"
#include "task_manager.h"

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;    // 1ms Timer
extern TIM_HandleTypeDef htim7;    // 5ms Timer
extern I2C_HandleTypeDef hi2c1;    // PMIC I2C
extern I2C_HandleTypeDef hi2c2;    // 예비 I2C
extern SPI_HandleTypeDef hspi1;    // EEPROM SPI
extern SPI_HandleTypeDef hspi2;    // 예비 SPI
extern CAN_HandleTypeDef hcan1;    // CAN 통신
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
  printf("[CRITICAL] Hard Fault occurred! System halted.\n");
  while (1)
  {
  }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
  printf("[CRITICAL] Memory Management Fault! System halted.\n");
  while (1)
  {
  }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
  printf("[CRITICAL] Bus Fault occurred! System halted.\n");
  while (1)
  {
  }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
  printf("[CRITICAL] Usage Fault occurred! System halted.\n");
  while (1)
  {
  }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers - Task Manager용                   */
/******************************************************************************/

/**
 * @brief This function handles TIM6 global and DAC1&2 underrun error interrupts.
 * @note 1ms Task Timer 인터럽트 (가장 중요!)
 */
void TIM6_DAC_IRQHandler(void)
{
  // Timer Update 플래그 확인 및 클리어
  if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET) {
    if (__HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) != RESET) {
      __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
      
      // 1ms Task 플래그 설정 (task_manager.c)
      task_1ms_isr_handler();
    }
  }
}

/**
 * @brief This function handles TIM7 global interrupt.
 * @note 5ms Task Timer 인터럽트
 */
void TIM7_IRQHandler(void)
{
  // Timer Update 플래그 확인 및 클리어
  if (__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET) {
    if (__HAL_TIM_GET_IT_SOURCE(&htim7, TIM_IT_UPDATE) != RESET) {
      __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
      
      // 5ms Task 플래그 설정 (task_manager.c)
      task_5ms_isr_handler();
    }
  }
}

/**
 * @brief This function handles EXTI line3 interrupt.
 * @note PMIC IRQ 핀 인터럽트 (PB3)
 */
void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  // HAL_GPIO_EXTI_Callback()이 main.c에서 호출됨
}

/******************************************************************************/
/*                 DMA Interrupt Handlers (기존 프로젝트 유지)                  */
/******************************************************************************/

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 * @note I2C1 RX DMA (PMIC 통신)
 */
void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
}

/**
 * @brief This function handles DMA1 stream2 global interrupt.
 * @note I2C2 RX DMA (예비용)
 */
void DMA1_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
}

/**
 * @brief This function handles DMA1 stream3 global interrupt.
 * @note I2C2 TX DMA (예비용)
 */
void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
}

/**
 * @brief This function handles DMA1 stream4 global interrupt.
 * @note SPI2 RX DMA (예비용)
 */
void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
}

/**
 * @brief This function handles DMA1 stream6 global interrupt.
 * @note I2C1 TX DMA (PMIC 통신)
 */
void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
}

/**
 * @brief This function handles DMA1 stream7 global interrupt.
 * @note SPI2 TX DMA (예비용)
 */
void DMA1_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
}

/**
 * @brief This function handles DMA2 stream0 global interrupt.
 * @note SPI1 RX DMA (EEPROM 통신)
 */
void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

/**
 * @brief This function handles DMA2 stream3 global interrupt.
 * @note SPI1 TX DMA (EEPROM 통신)
 */
void DMA2_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

/******************************************************************************/
/*                 CAN Interrupt Handlers (기존 프로젝트용)                     */
/******************************************************************************/

/**
 * @brief This function handles CAN1 TX interrupts.
 * @note CAN 송신 완료 인터럽트
 */
void CAN1_TX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

/**
 * @brief This function handles CAN1 RX0 interrupts.
 * @note CAN RX FIFO 0 메시지 수신 인터럽트
 */
void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
  // HAL_CAN_RxFifo0MsgPendingCallback()이 can_service.c에서 호출됨
}

/**
 * @brief This function handles CAN1 RX1 interrupts.
 * @note CAN RX FIFO 1 메시지 수신 인터럽트
 */
void CAN1_RX1_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

/**
 * @brief This function handles CAN1 SCE interrupt.
 * @note CAN 상태 변경 및 에러 인터럽트
 */
void CAN1_SCE_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

/******************************************************************************/
/*                 UART Interrupt Handlers (필요시 추가)                       */
/******************************************************************************/

/**
 * @brief This function handles UART4 global interrupt.
 * @note 디버깅 UART (필요시 활성화)
 */
void UART4_IRQHandler(void)
{
  // UART 인터럽트 처리가 필요한 경우 활성화
  // HAL_UART_IRQHandler(&huart4);
}

/******************************************************************************/
/*                 ADC Interrupt Handlers (필요시 추가)                        */
/******************************************************************************/

/**
 * @brief This function handles ADC1 global interrupt.
 * @note ADC 변환 완료 인터럽트 (현재는 폴링 방식 사용)
 */
void ADC_IRQHandler(void)
{
  // ADC 인터럽트 방식 사용시 활성화
  // HAL_ADC_IRQHandler(&hadc1);
}