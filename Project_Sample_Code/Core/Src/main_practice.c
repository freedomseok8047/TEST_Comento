#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// 시스템 헤더 (나중에 추가될 예정)
// #include "mp5475gu.h"    // PMIC I2C
// #include "25lc256.h"     // EEPROM SPI  
// #include "tja1051.h"     // CAN Transceiver
// #include "uart_debug.h"  // UART Debug

/**
 * @brief 시스템 초기화 함수
 * @note 모든 하드웨어 초기화를 담당
 */
void system_init(void) {
	printf("[INIT] System initialization started ...\n");

	//TODO 하드웨어 초기화
	// I2C 초기화 (MP5475GU PMIC)
	// SPI 초기화 (25LC256 EEPROM)
	// CAN CHRLGHK (TJA1051)
	// UART 초기화 (Debug Log)

	printf("[INIT] System initialization completed!...\n");
}

/**
 * @brief I2C 통신 처리 함수 (PMIC 상태 확인)
 * @note MP5475GU와 I2C DMA Interrupt 방식으로 통신
 */
void system_init(void){
	printf("[I2C] Processing PMIC status check ...\n");

	//todo
	// PMIC 레지스터 읽기 (0x07, 0x08, 0x09)
	// 고장감지 (UV/OV/OV 체크)
	// PG 확인 레지스터 (0x06)
	// DTC 코드 생성 (0xC001 ~ 0xC006)

	printf("[I2C] PMIC status: Normal! ...\n");
}

/**
 * @brief SPI 통신 처리 함수 (EEPROM 데이터 처리)
 * @note 25LC256과 SPI DMA Interrupt 방식으로 통신
 */
void process_spi_step(void){
	printf("[SPI] Processing EEPROM data...\n");

	//todo
	// Write Enable (WREN 0x06)
	// DTC 저장/읽기 (Page Write 사용)
	// Status 확인 (WIP 비트 체크)

	printf("[SPI] EEPROM operation completed! \n");
}

/**
 * @brief CAN 통신 처리 함수 (진단 데이터 전송)
 * @note TJA1051을 통해 UDS 프로토콜로 Interrupt 방식 동작
 */
void process_can_step(void)
{
	printf("[CAN] Processing diagnostic data transmission...\n");

	//todo
	// CAN 프레임 준비 (UDS Format)
	// DTC 전송 (via TJA1051)
	// Clear 요청 처리 (Service 0x14)

	printf("[CAN] Diagnostic data transmitted! \n");
}

/**
 * @brief UART 통신 처리 함수 (로그 출력)
 * @note Polling 방식으로 시스템 상태 로그 출력
 */
void process_uart_step(void)
{
	printf("[UART] Processing log output...\n");

	//todo
	// 시스템 상태 로그
	// 정상 동작 메시지
	// 에러 발생시 상세 정보

	printf("[UART] System status : All systems operational \n");
}

/**
 * @brief 메인 함수
 * @return int 프로그램 종료 코드
 */
int main(void)
{
	printf("==== 브레이크 시스템 PMIC 제어 시작 === \n");
	printf("HW : MP5475GU + 25LC256 + TJA1051\n\n");

	//시스템 초기화
	system_init();

	printf("\n=== Main Loop 시작 ===\n");
	printf("동작 순서 : I2C -> SPI -> CAN -> UART \n\n");

	unit32_t loop_count = 0;

	while(1)
	{
		printf("--- Loop #%d --- \n", ++loop_count);

		// I2C -> SPI -> CAN -> UART 순서
		process_i2c_step();
		process_spi_step();
		process_can_step();
		process_uart_step();

		printf("[DELAY] 100ms delay...\n");

		HAL_Delay(100);

		if(loop_count >= 3){
			printf("==시뮬레이션 완료 ==");
			break;
		}

		printf("\n")

	}

	return 0;
}
























