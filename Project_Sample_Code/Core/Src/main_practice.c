#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h> 
#include "stm32f4xx_hal.h"
#include "mp5457gu.h"
#include "common_types.h"
#include "dtc_manager.h"
#include "pmic_service.h"
#include "uds_protocol.h"
#include "eeprom_service.h"

// ========== PMIC 설정 상수 ==========
// #define MP5475_I2C_ADDRESS          0x60
// #define MP5475_REG_READ_SIZE        1

// 외부 변수 참조
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern SPI_HandleTypeDef hspi1, hspi2;
extern UART_HandleTypeDef huart4;

// ========== I2C 상태 머신 ==========
// typedef enum {
// 	I2C_STATE_IDLE = 0,
// 	I2C_STATE_RX_BUSY,
// 	I2C_STATE_RX_COMPLETE,
// 	I2C_STATE_ERROR
// } i2c_state_t;

// ========== EEPROM 상태 머신 ==========
// typedef enum {
//   EEPROM_STATE_IDLE = 0,
//   EEPROM_STATE_WRITE_ENABLE,
//   EEPROM_STATE_WRITING,
//   EEPROM_STATE_READ_STATUS,
//   EEPROM_STATE_COMPLETE,
//   EEPROM_STATE_ERROR
// } eeprom_state_t;

 // ========== PMIC 레지스터 주소 정의 ==========
//  typedef enum {
// 	MP5475_REG_SYSTEM_STATUS = 0x05,
// 	MP5475_REG_POWER_GOOD = 0x06,
// 	MP5475_REG_UV_OV_FAULT = 0x07,
// 	MP5475_REG_OC_FAULT = 0x08,
// 	MP5475_REG_TEMP_FAULT = 0x09
// } mp5475_register_t;

 // ========== DTC 코드 정의 ==========
// typedef enum {
//     DTC_BRAKE_PMIC_UV     = 0xC001,  // 16비트
//     DTC_BRAKE_PMIC_OV     = 0xC002,  // 16비트
//     DTC_BRAKE_PMIC_OC     = 0xC003,  // 16비트
//     DTC_BRAKE_PMIC_TEMP   = 0xC004,  // 16비트
//     DTC_BRAKE_COMM_ERROR  = 0xC005,  // 16비트
//     DTC_BRAKE_SYSTEM_FAULT = 0xC006  // 16비트
// } brake_dtc_code_t;

// --- DTC 데이터 구조 정의 ---
#pragma pack(push, 1)  // 기존:56 Byte × 6 = 336 최적화: 53 Byte × 6 = 318 
typedef struct {
	uint16_t DTC_Code;              // 고장 코드 (예: C1234)
	char Description[50];           // 설명 문자열
	uint8_t active;                 // 활성화 상태 플래그
  } DTC_Table_t;
#pragma pack(pop) 

DTC_Table_t DTC_Table = { 0x1234, "Brake UV Fault", 0 };

 // ========== I2C 상태 관리 변수 ==========
// volatile i2c_state_t i2c_state = I2C_STATE_IDLE;  // 현재 I2C 작업 상태
// uint8_t current_reg_index = 0;              // 레지스터 배열 인덱스 (0부터 시작)
// uint8_t pmic_reg_address = 0;               // 현재 처리 중인 레지스터 주소
// uint8_t pmic_reg_data = 0;                  // DMA로 읽은 데이터 저장용
// uint8_t i2c_rx_buffer[1] = {0};             // DMA 수신 버퍼 (1바이트씩 읽음음 크기 1)

// ========== EEPROM 상태 관리 변수 ==========
// volatile eeprom_state_t eeprom_state = EEPROM_STATE_IDLE;
// static volatile bool eeprom_dma_busy = false;
// static volatile bool eeprom_write_in_progress = false;
// static uint8_t current_dtc_save_index = 0;

// // EEPROM DMA 버퍼
// static uint8_t eeprom_tx_buffer[67] __attribute__((aligned(4)));  // CMD(1) + ADDR(2) + DATA(64)
// static uint8_t eeprom_rx_buffer[67] __attribute__((aligned(4)));
// static uint16_t current_eeprom_address = LC256_AREA_DTC_CURRENT;
// static uint16_t dtc_log_count = 0;
// static bool eeprom_is_init_mode = false;

// EEPROM용 DTC 로그 구조체 (64바이트)
// typedef struct {
//     uint32_t timestamp;
//     uint16_t DTC_Code;
//     char Description[48];
//     uint8_t active;
//     uint8_t occurrence_count;
//     uint8_t status;
//     uint8_t reserved[9];
// } __attribute__((packed)) eeprom_dtc_log_t;

static eeprom_dtc_log_t current_dtc_log;

// GPIO 핀 정의 (EEPROM CS 추가)
// #define EEPROM_CS_PORT    GPIOB
// #define EEPROM_CS_PIN     GPIO_PIN_4

// 읽을 레지스터 주소 배열 (순서대로 처리)
// static const mp5475_register_t reg_addresses[] = {
// 	  MP5475_REG_SYSTEM_STATUS,	// 0x05
//     MP5475_REG_POWER_GOOD,      // 0x06
//     MP5475_REG_UV_OV_FAULT,     // 0x07
//     MP5475_REG_TEMP_FAULT       // 0x09
// };

// 읽은 레지스터 데이터 저장 배열
// static uint8_t pmic_status_data[4];

 // ========== DTC 마스터 테이블 ==========
static const DTC_Table_t dtc_master_table[] = {
    {DTC_BRAKE_PMIC_UV,     "Buck Undervoltage Fault",     0},
    {DTC_BRAKE_PMIC_OV,     "Buck Overvoltage Fault",      0},
    {DTC_BRAKE_PMIC_OC,     "Buck Overcurrent Fault",      0},
    {DTC_BRAKE_PMIC_TEMP,   "PMIC Temperature Fault",      0},
    {DTC_BRAKE_COMM_ERROR,  "I2C Communication Error",     0},
    {DTC_BRAKE_SYSTEM_FAULT,"System/Power Good Fault",     0}
};

 // ========== 감지된 DTC 저장 배열 ==========
// static DTC_Table_t detected_dtc_table[6];
// static uint8_t dtc_count = 0;

/* ========== 함수 프로토타입 ========== */
// static HAL_StatusTypeDef start_pmic_register_read(uint8_t reg_addr);
// static void process_pmic_register_data(uint8_t reg_addr, uint8_t reg_data);
// static void dtc_add_code(uint16_t dtc_code);
// static void dtc_print_summary(void);
// static void dtc_reset_diagnosis(void);

/* ========== EEPROM 함수 프로토타입 ========== */
// static void eeprom_cs_select(void);
// static void eeprom_cs_deselect(void);
// static HAL_StatusTypeDef eeprom_write_enable(void);
// static HAL_StatusTypeDef eeprom_wip_check(void);
// static HAL_StatusTypeDef eeprom_write_dtc_log(uint16_t address, eeprom_dtc_log_t* dtc_log);
// static bool eeprom_is_write_complete(void);
// static void save_dtc_to_eeprom(void);
// static void convert_dtc_to_log(DTC_Table_t* dtc, eeprom_dtc_log_t* log);
// static uint16_t get_next_eeprom_address(void);
// static HAL_StatusTypeDef eeprom_init(void);

// Init 프로토 타입 선언
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);

/*
1. save_dtc_to_eeprom() 호출 ↓ 
2. eeprom_write_enable() 호출 ↓ 
3. HAL_SPI_TxCpltCallback() → Write Enable 완료 감지 ↓
4. eeprom_write_dtc_log() 호출 (실제 데이터 쓰기) ↓
5. HAL_SPI_TxCpltCallback() → Page Write 완료 감지↓
6. eeprom_wip_check() 호출 ↓ 
7. HAL_SPI_TxRxCpltCallback() → Status 읽기 완료 ↓
8. WIP 비트 확인 → 완료되면 다음 DTC 또는 종료
*/

// ========== EEPROM CS 핀 제어 함수 ==========
// static void eeprom_cs_select(void)
// {
//     HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_RESET);
// }

// static void eeprom_cs_deselect(void)
// {
//   HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_SET);
// }

//========== EEPROM Write Enable 함수 ==========
/*
Role: 25LC256 EEPROM에 "이제 데이터를 쓸 준비를 해라"
why?
EEPROM은 데이터 보호를 위해 기본적으로 쓰기가 금지되어 있음
쓰기 전에 반드시 Write Enable 명령(0x06)을 보내야 함
데이터시트에서 요구하는 필수 절차
when?
DTC를 EEPROM에 저장하기 직전
save_dtc_to_eeprom() 함수에서 첫 번째로 호출
what?
DMA가 사용 중인지 확인
Write Enable 명령(0x06)을 송신 버퍼에 준비
CS 핀을 LOW로 설정 (EEPROM 선택)
DMA로 1바이트 전송 시작
완료는 HAL_SPI_TxCpltCallback()에서 처리
*/
// static HAL_StatusTypeDef eeprom_write_enable(void)
// {
//     if(eeprom_dma_busy) {
//         return HAL_BUSY;
//     }

//     printf("[EEPROM] Sending Write Enable command (0X06)\n");
    
//     eeprom_dma_busy = true;
//     eeprom_tx_buffer[0] = LC256_CMD_WREN;
    
//     eeprom_cs_select();
//     eeprom_state = EEPROM_STATE_WRITE_ENABLE;
//     //hspi: SPI 핸들 포인터 pData: 송신할 데이터 버퍼 Size: 송신할 바이트 수
//     HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi1, eeprom_tx_buffer, 1); 
//                               // -> HAL_SPI_TxCpltCallback

//     if(status != HAL_OK){
//         printf("[EEPROM] ERROR: Write Enable failed\n");
//         eeprom_cs_deselect();
//         eeprom_state = EEPROM_STATE_ERROR;
//         eeprom_dma_busy = false;
//     }
//     return status;
// }

//========== EEPROM CHECK WIP 읽기 함수 ==========
/*
Role : "쓰기 완료됐나 확인" [WIP 비트 체크]
why?
EEPROM은 데이터 쓰기에 최대 5ms가 소요됨
쓰기 중에는 다른 명령을 받을 수 없음
WIP(Write In Progress) 비트를 확인해서 완료 여부 판단
when?
Page Write 완료 후 -> HAL_SPI_TxCpltCallback()에서 WRITING 상태일 때 호출
what?
DMA가 사용 중인지 확인
Status Read 명령(0x05) + 더미바이트를 준비
CS 핀을 LOW로 설정
DMA로 2바이트 송수신 시작
완료는 HAL_SPI_TxRxCpltCallback()에서 처리
*/
// static HAL_StatusTypeDef eeprom_wip_check(void)
// {
//     if(eeprom_dma_busy){
//       return HAL_BUSY;
//     }
//     printf("[EEPROM] Reading status register\n");

//     eeprom_dma_busy = true;
//     eeprom_tx_buffer[0] = LC256_CMD_RDSR;
//     eeprom_tx_buffer[1] = 0x00;

//     eeprom_cs_select();
//     eeprom_state = EEPROM_STATE_READ_STATUS;
//     /*
//     hspi: SPI 핸들 포인터 (&hspi1)
//     pTxData: 송신할 데이터 버퍼 (eeprom_tx_buffer)
//     pRxData: 수신된 데이터 저장 버퍼 (eeprom_rx_buffer)
//     Size: 송수신할 바이트 수 (2)*/
//     HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(&hspi1,
//                                                             eeprom_tx_buffer,
//                                                             eeprom_rx_buffer,
//                                                             2); // -> HAL_SPI_TxRxCpltCallback()

//     if(status != HAL_OK){
//         printf("[EEPROM] ERROR: Status read failed\n");
//         eeprom_cs_deselect();
//         eeprom_state = EEPROM_STATE_ERROR;
//         eeprom_dma_busy = false;
//     }

//     return status;
// }

// ========== Write 완료 확인 함수 ==========
// static bool eeprom_is_write_complete(void)
// {
//     return !(eeprom_rx_buffer[1] & 0x01); 
//     // WIP = 1: 쓰기 진행 중 (Write In Progress)
//     // WIP = 0: 쓰기 완료됨 (Write Complete)
// }

// // ========== DTC 로그 변환 함수 ==========
// static void convert_dtc_to_log(DTC_Table_t* dtc, eeprom_dtc_log_t* log)
// {
//   /*
//     매개변수:
//     ptr: 채울 메모리 블록의 시작 주소
//     value: 채울 값 (0~255)
//     num: 채울 바이트 수

//     log: eeprom_dtc_log_t 구조체 주소소
//     0: 0x00 값으로 채움
//     sizeof(eeprom_dtc_log_t): 구조체 크기만큼 (64바이트)

//     결과:
//     구조체의 모든 바이트를 0으로 초기화
//     64바이트 모두 0x00으로 설정됨

//     왜 사용하나:
//     구조체 초기화 (쓰레기 값 제거)
//     패딩 바이트까지 모두 0으로 설정
//     안전한 초기 상태 보장
//   */
//   // eeprom_dtc_log_t 구조체 초기화
//   memset(log, 0, sizeof(eeprom_dtc_log_t));

//   log->timestamp = HAL_GetTick();
//   log->DTC_Code = dtc->DTC_Code;
//   strncpy(log->Description, dtc->Description, 47); 
//   //47의미: Description 필드의 크기 -1
//   log->active = dtc->active;
//   log->occurrence_count = 1;
//   log->status = 0x01;
// }

// ========== EEPROM DTC 쓰기 함수 ==========
// static HAL_StatusTypeDef eeprom_write_dtc_log(uint16_t address, eeprom_dtc_log_t* dtc_log)
// {
//     if(eeprom_dma_busy){
//         return HAL_BUSY;
//     }

//     printf("[EEPROM] Writing DTC log to address 0x%04X\n", address);

//     eeprom_dma_busy = true;
//     eeprom_write_in_progress = true;

//     eeprom_tx_buffer[0] = LC256_CMD_WRITE; // CMD 명령 : 바이트
//     eeprom_tx_buffer[1] = (address >> 8) & 0x7F; 
//     //25LC256은 15비트 주소 사용 (0x0000~0x7FFF)
//     // address >> 8 
//     // 0x7F = 127 = 0111 1111 address 상위 9비트 중 맨 앞비트 제외 
//     eeprom_tx_buffer[2] = address & 0xFF; // address 하우 8비트 [1],[2] 합쳐서 15비트 주소
    
//     //Memory Copy : memcpy(목적지, 소스, 복사할_바이트_수);
//     memcpy(&eeprom_tx_buffer[3], dtc_log, sizeof(eeprom_dtc_log_t));

//     eeprom_cs_select();
//     eeprom_state = EEPROM_STATE_WRITING;

//     HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi1,
//                                                     eeprom_tx_buffer,
//                                                     3 + sizeof(eeprom_dtc_log_t)); 
//                                                     // -> HAL_SPI_TxCpltCallback()
    
//     if (status != HAL_OK){
//         printf("[EEPROM] ERROR: DTC write failed\n");
//         eeprom_cs_deselect();
//         eeprom_state = EEPROM_STATE_ERROR;
//         eeprom_dma_busy = false;
//         eeprom_write_in_progress = false;
//     }
  
//     return status;
// }

// ========== 다음 주소 계산 함수 ==========
/*
"지속적으로 DTC를 저장"
"메모리 부족으로 저장 중단 X"
"가장 최근 64개 DTC를 유지"
*/
// static uint16_t get_next_eeprom_address(void)
// {
//     uint16_t next_addr = LC256_AREA_DTC_CURRENT + (dtc_log_count * sizeof(eeprom_dtc_log_t));

//     //DTC를 저장할 위치가 LC256_AREA_DTC_HISTORY 넘으면 다시 0x0000부터 저장 시작 (덮어쓰기)
//     if(next_addr + sizeof(eeprom_dtc_log_t) >= LC256_AREA_DTC_HISTORY)
//     {
//         next_addr = LC256_AREA_DTC_CURRENT;
//         dtc_log_count = 0;
//     }

//     return next_addr;
// }

// ========== DTC EEPROM 저장 시작 함수 (추가) ==========
// static void save_dtc_to_eeprom(void)
// {
//     if(dtc_count == 0){
//         printf("[EEPROM] NO DTCs to save\n");
//         return;
//     }

//     if (eeprom_state != EEPROM_STATE_IDLE){
//       printf("[EEPROM] EEPROM busy, cannot save DTCs\n");
//       return;
//     }

//     printf("[EEPROM] Starting DTC save process (%d DTCs)\n", dtc_count);

//     current_dtc_save_index = 0 ;
//     current_eeprom_address = get_next_eeprom_address();

//     convert_dtc_to_log(&detected_dtc_table[current_dtc_save_index], &current_dtc_log);

//     if(eeprom_write_enable() != HAL_OK){
//         printf("[EEPROM] ERROR: Failed to start DTC save\n");
//         eeprom_state = EEPROM_STATE_ERROR;
//     }
// }

// PMIC IRQ 발생 시 호출되는 콜백
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_3) {  // PMIC IRQ 핀
        printf("[IRQ] PMIC fault detected, starting diagnosis\n");
        
        // PMIC 서비스를 통해 진단 시작
        if (!pmic_service_start_diagnosis()) {
            printf("[WARNING] PMIC diagnosis failed to start\n");
        }

        // // I2C 상태가 유휴일 때만 진단 시작
        // if (i2c_state == I2C_STATE_IDLE) {
        //     dtc_reset_diagnosis();
        //     current_reg_index = 0;
        //     if (start_pmic_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
        //         i2c_state = I2C_STATE_ERROR;
        //     }
        // } else{
        //     printf("[WARNING] I2C bush, IRQ ignored\n");
        // }
    }
}

 // ========== I2C DMA 레지스터 읽기 시작 함수 ==========
  /**
  * @brief PMIC 레지스터 DMA 읽기 시작
  * @param reg_addr 읽을 레지스터 주소
  * @retval HAL_StatusTypeDef
  */
  // static HAL_StatusTypeDef start_pmic_register_read(uint8_t reg_addr)
  // { // 현재 레지스터 주소 저장
  //   pmic_reg_address = reg_addr;
  //   // 상태를 BUSY 대기로 변경
  //   i2c_state = I2C_STATE_RX_BUSY;
  //   // DMA 버퍼 초기화
  //   i2c_rx_buffer[0] = 0;
  //   printf("[I2C] Starting DMA read - Register: 0x%02X\n", reg_addr);
  //   // HAL_I2C_Mem_Read_DMA 호출
  //   /*HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c,
  //                 uint16_t DevAddress,
  //                 uint16_t MemAddress,
  //                 uint16_t MemAddSize,
  //                 uint8_t *pData,
  //                 uint16_t Size); */
  //   /* (MP5475_I2C_ADDRESS << 1)
  //   I2C 프로토콜 : 주소 7비트 + R/W 비트 1비트 = 총 8비트 전송.
  //   HAL 라이브러리가 R/W 비트를 자동으로 추가하므로,
  //   사용자는 7비트 주소를 1비트 왼쪽으로 시프트하여 공간을 만들어줘야 함함.
  //   */
  //   HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(&hi2c1,
  //                                                   (MP5475_I2C_ADDRESS << 1),
  //                                                   reg_addr,
  //                                                   I2C_MEMADD_SIZE_8BIT,
  //                                                   i2c_rx_buffer,
  //                                                   MP5475_REG_READ_SIZE);
  //   // READ 결과 확인
  //   if (status != HAL_OK){
  //     printf("[I2C] ERROR: Failed to start DMA read\n");
  //     i2c_state = I2C_STATE_ERROR;
  //     return status;
  //   }

  //   return HAL_OK;
  // }

 // ========== I2C DMA 수신 완료 콜백 함수 ==========
 /**
  * @brief I2C DMA 수신 완료 콜백
  * @param hi2c I2C 핸들러 포인터
  * @note DMA 인터럽트에서 자동 호출
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{	
    extern uint8_t i2c_rx_buffer[1];  // pmic_service.c의 버퍼 참조
    
    if(hi2c->Instance == I2C1){
        uint8_t reg_data = i2c_rx_buffer[0];
        printf("[I2C] DMA RX Complete - data: 0x%02X\n", reg_data);
        
        // PMIC 서비스에 데이터만 전달 (주소는 내부에서 관리)
        pmic_service_register_received(reg_data);

  //I2C1에서 발생한 콜백인지 확인
	// if(hi2c -> Instance == I2C1){
	
	// 	uint8_t pmic_reg_data = i2c_rx_buffer[0];  // 지역변수로 변경
	// 	printf("[I2C] DMA RX Complete - Reg: 0x%02X, data: 0x%02X\n", pmic_reg_address, pmic_reg_data);

		//DMA 버퍼에서 데이터 복사
		// pmic_reg_data = i2c_rx_buffer[0];
		// //상태를 수신완료로 변경
		// i2c_state = I2C_STATE_RX_COMPLETE;

		// printf("[I2C] DMA RX Complete - Reg: 0x%02X, data: 0x%02X\n", pmic_reg_address,pmic_reg_data);
		
		//받은 데이터 처리
		// process_pmic_register_data(pmic_reg_address, pmic_reg_data);//pmic_reg_address는 전역변수로 선언했고 start_pmic_register_read함수 첫줄에서 값 설정 완료
		
    // // 다음 레지스터로 이동
    // current_reg_index++; // 0x06 -> 0x07 -> 0x08 -> 0x09
    
    // if(current_reg_index < 4) {
    //   if(start_pmic_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
    //       i2c_state = I2C_STATE_ERROR;
    //   }
    // } else {
    //   // 모든 레지스터 읽기 완료
    //   printf("[I2C] All registers read, diagnosis complete\n");
    //   dtc_print_summary();
    //   i2c_state = I2C_STATE_IDLE;
    // }
	}
}

 // ========== I2C DMA 에러 콜백 함수 ==========
 /**
  * @brief I2C DMA 에러 콜백
  * @param hi2c I2C 핸들러 포인터
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c -> Instance == I2C1) {
		uint32_t error_code = HAL_I2C_GetError(hi2c);
		printf("[I2C] ERROR: DMA communication failed (Error: 0x%08X)\n",error_code);
		
    // PMIC 서비스에 에러 알림
		pmic_service_communication_error();

    // // 에러 상태로 변경
		// i2c_state = I2C_STATE_ERROR;
		// // 통신 에러 DTC 추가 
		// dtc_add_code(DTC_BRAKE_COMM_ERROR);
	}
}

// ========== SPI DMA 송신 완료 콜백 ==========
/*
  eeprom_write_enable() 내부 HAL_SPI_Transmit_DMA()의 완료 콜백 
  if (EEPROM_STATE_WRITE_ENABLE) : 데이터 쓸 준비 완
  if (EEPROM_STATE_WRITING) : 데이터 쓰는 시간 5ms 대기 , 데이터 쓰기 완 
*/
// ✅ 새로운 코드 (간단해짐)
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI1){
        // EEPROM 서비스에 전달
        eeprom_service_tx_complete_callback();
    }
}
// void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
// {
//     if(hspi -> Instance == SPI1){

//         eeprom_cs_deselect();
//         eeprom_dma_busy = false;

//         switch (eeprom_state) {
//             case EEPROM_STATE_WRITE_ENABLE:
//                 printf("[EEPROM] Write ENable completed\n");
//                 HAL_Delay(1);

//                 if (eeprom_write_dtc_log(current_eeprom_address, &current_dtc_log) != HAL_OK)
//                 {
//                   eeprom_state = EEPROM_STATE_ERROR;
//                 }
//                 break;
            
//             case EEPROM_STATE_WRITING:
//                 printf("[EEPROM] DTC write completed, cecking status\n");
//                 HAL_Delay(LC256_WRITE_CYCLE_TIME);

//                 if(eeprom_wip_check() != HAL_OK) {
//                     eeprom_state = EEPROM_STATE_ERROR;
//                 }
//                 break;

//             default:
//                 break;
//         }
//     }
// }

// ========== SPI DMA 송수신 완료 콜백 ==========
// ✅ 새로운 코드 (간단해짐)
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI1){
        // EEPROM 서비스에 전달
        eeprom_service_txrx_complete_callback();
    }
}

// void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
// {
//     if(hspi->Instance == SPI1){
//         eeprom_cs_deselect();
//         eeprom_dma_busy = false;

//         // 콜백에서 구분
//         if (eeprom_is_init_mode) {
//         // 초기화 모드: 단순히 상태만 확인
//         printf("[EEPROM] Init status check: 0x%02X\n", eeprom_rx_buffer[1]);
//         eeprom_is_init_mode = false;
//         return;
//         }

//         if(eeprom_state == EEPROM_STATE_READ_STATUS){
//             printf("[EEPROM] Status: 0x%02X\n", eeprom_rx_buffer[1]);
            
//             if (eeprom_is_write_complete()) {
//                 printf("[EEPROM] Write completed successfully\n");

//                 dtc_log_count++;
//                 current_dtc_save_index++;

//                 if(current_dtc_save_index < dtc_count) {
//                     printf("[EEPROM] Processing next DTC (%d/%d)\n", current_dtc_save_index + 1, dtc_count);

//                     current_eeprom_address = get_next_eeprom_address();
//                     convert_dtc_to_log(&detected_dtc_table[current_dtc_save_index], &current_dtc_log);

//                     if(eeprom_write_enable() != HAL_OK) {
//                         eeprom_state = EEPROM_STATE_ERROR;
//                     }
//                 } else {
//                     printf("[EEPROM] All DTCs saved successfully\n");
//                     eeprom_state = EEPROM_STATE_COMPLETE;
//                     eeprom_write_in_progress = false;
//                 }
//             } else {
//                 printf("[EEPROM] Write in progress, checking again...\n");
//                 HAL_Delay(1);
//                 if (eeprom_wip_check() != HAL_OK) {
//                     eeprom_state = EEPROM_STATE_ERROR;
//                 }
//             }
//         }
//     }
// }

// ========== SPI 에러 콜백 ==========
// ✅ 새로운 코드 (간단해짐)
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI1) {
        // EEPROM 서비스에 전달
        eeprom_service_error_callback();
    }
}
// void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
// {
//     if(hspi->Instance == SPI1) {
//         uint32_t error_code = HAL_SPI_GetError(hspi);
//         printf("[EEPROM] ERROR: SPI failed (Error: 0x%08lx)\n",error_code);

//         eeprom_cs_deselect();
//         eeprom_state = EEPROM_STATE_ERROR;
//         eeprom_dma_busy = false;
//         eeprom_write_in_progress = false;
//     }
// }
/*
0x01 = 00000001  // 비트 0 체크
0x02 = 00000010  // 비트 1 체크  
0x04 = 00000100  // 비트 2 체크
0x08 = 00001000  // 비트 3 체크
0x10 = 00010000  // 비트 4 체크
0x20 = 00100000  // 비트 5 체크
0x40 = 01000000  // 비트 6 체크
0x80 = 10000000  // 비트 7 체크
*/
 // ========== PMIC 레지스터 데이터 처리 함수 ==========
 /**
  * @brief 읽은 PMIC 레지스터 데이터 분석 및 DTC 매핑
  * @param reg_addr 레지스터 주소
  * @param reg_data 레지스터 데이터
  */
// static void process_pmic_register_data(uint8_t reg_addr, uint8_t reg_data)
// {
// 	printf("[DIAG] Processing register 0x%02X with data 0x%02X\n",reg_addr,reg_data);
// 	// 읽은 데이터를 배열에 저장
// 	pmic_status_data[current_reg_index] = reg_data;
// 	// 레지스터별 데이터 분석
// 	switch (reg_addr) {
// 		case MP5475_REG_POWER_GOOD: // 0x06
// 			// PG_FILT 비트만 체크 (비트 7-4: 1111XXXX)
// 			if((reg_data & 0xF0) != 0xF0){ //전체 비트와 0xF0=11110000과 &연산 결과가 0xF0와 다를때
// 				printf("[FAULT] Power Good FAIL - Data: 0x%02X\n", reg_data);
//         		dtc_add_code(DTC_BRAKE_SYSTEM_FAULT);
// 			}
// 			break;
// 		case MP5475_REG_UV_OV_FAULT: // 0x07
// 			// 비트 7: 시스템 에러 UV
// 			if (reg_data & 0x80 || reg_data & 0x40 || reg_data & 0x20 || reg_data & 0x10)
// 			{	// 7654비트 1일때
// 				printf("[FAULT] PMIC error UV detected- Data: 0x%02X\n", reg_data);
// 				dtc_add_code(DTC_BRAKE_PMIC_UV);
// 			}else if (reg_data & 0x08 || reg_data & 0x04 || reg_data & 0x02 || reg_data & 0x01)
// 			{	// 3210비트 1일때  OV
// 				printf("[FAULT] PMIC error OV detected- Data: 0x%02X\n", reg_data);
// 				dtc_add_code(DTC_BRAKE_PMIC_OV);
// 			}
// 			break;
// 		case MP5475_REG_OC_FAULT: //0x08
// 			if(reg_data & 0x80 || reg_data & 0x40 || reg_data & 0x20 || reg_data & 0x10){
// 				printf("[FAULT] PMIC error OC detected- Data: 0x%02X\n", reg_data);
// 				dtc_add_code(DTC_BRAKE_PMIC_OC);
// 			}
// 			if (reg_data & 0x08 || reg_data & 0x04 || reg_data & 0x02 || reg_data & 0x01)
// 			{	printf("[WARNING] PMIC OC WARNING - Data: 0x%02X\n", reg_data);
// 			}
// 			break;
// 		case MP5475_REG_TEMP_FAULT: //0x09
// 			if(reg_data & 0x01){
// 				printf("[FAULT] PMIC High Temperature Shutdown - Data: 0x%02X\n", reg_data);
// 				dtc_add_code(DTC_BRAKE_PMIC_TEMP);
// 			}
// 			if (reg_data & 0x02) {
// 				printf("[WARNING] PMIC High Temperature Warning - Data: 0x%02X\n", reg_data);
// 			}
// 			break;
// 	}
// }

/**
 * @brief 시스템 초기화 함수
 * @note 모든 하드웨어 초기화를 담당
 */
void system_init(void) 
{
	printf("[INIT] System initialization started ...\n");

	//TODO 하드웨어 초기화
	  MX_GPIO_Init();
	  MX_DMA_Init();
	  MX_ADC1_Init();
	  MX_CAN1_Init();
	  MX_I2C1_Init();
	  MX_I2C2_Init();
	  MX_SPI1_Init();
	  MX_SPI2_Init();
	  MX_UART4_Init();

    // EEPROM 초기화 추가
    if (eeprom_init() != HAL_OK) {
      printf("[INIT] ERROR: EEPROM initialization failed!\n");
      Error_Handler();
    }

	printf("[INIT] System initialization completed!...\n");
}

/**
 * @brief I2C 통신 처리 함수 (PMIC 상태 확인)
 * @note MP5475GU와 I2C DMA Interrupt 방식으로 통신
 */
// void process_i2c_step(void)
// {
// 	printf("[I2C] Processing PMIC status check ...\n");

// 	// PMIC 레지스터 읽기 (0x07, 0x08, 0x09)
// 	// 고장감지 (UV/OV/OV 체크)
// 	// PG 확인 레지스터 (0x06)
// 	// DTC 코드 생성 (0xC001 ~ 0xC006)
// 	switch (i2c_state) {
// 	    case I2C_STATE_IDLE:
// 	    	//새로운 진단 시작
// 				printf("[I2C] Starting PMIC diagnosis sequence\n");
// 				dtc_reset_diagnosis();
// 				current_reg_index = 0;
// 				if (start_pmic_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
// 								printf("[I2C] Failed to start PMIC register read\n");
// 								i2c_state = I2C_STATE_ERROR;
// 							}
// 	        break;
// 	    case I2C_STATE_TX_BUSY:
// 	        // 처리
// 	        break;
// 	    case I2C_STATE_TX_COMPLETE:
// 	        // 처리
// 	        break;
// 	    case I2C_STATE_RX_BUSY:
// 	        // 처리
// 	        break;
// 	    case I2C_STATE_RX_COMPLETE:
// 	        // 처리
// 	        break;
// 	    case I2C_STATE_ERROR:
// 	        // 처리
// 	        break;
// 			default:
//             printf("[I2C] Unknown state: %d\n", i2c_state);
//             i2c_state = I2C_STATE_IDLE;  // 안전한 상태로 복귀
//             break;`
// 	}
// }

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
	//시스템 초기화
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();  // HAL 라이브러리 초기화
    SystemClock_Config();  // 시스템 클럭 설정

    //모든 시스템 IC init
    system_init();

    // DTC 매니저 초기화 추가
    if (!dtc_manager_init()) {
        printf("[MAIN] ERROR: DTC Manager initialization failed!\n");
        Error_Handler();
    }

    // PMIC 서비스 초기화 추가
    if (!pmic_service_init(&hi2c1)) {
        printf("[MAIN] ERROR: PMIC Service initialization failed!\n");
        Error_Handler();
    }

    // UDS 프로토콜 초기화 추가
    if (!uds_protocol_init()) {
        printf("[MAIN] ERROR: UDS Protocol initialization failed!\n");
        Error_Handler();
    }
    
    // EEPROM 서비스 초기화 추가
    if (!eeprom_service_init(&hspi1)) {
        printf("[MAIN] ERROR: EEPROM Service initialization failed!\n");
        Error_Handler();
    }
    
	uint32_t loop_count = 0;

	while(1)
	{

		// I2C -> SPI -> CAN -> UART 순서
		// process_i2c_step();
		// process_spi_step();
		// process_can_step();
		// process_uart_step();

		HAL_Delay(10);
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_SET);    

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // PMIC IRQ 핀 설정 (예: PB3)
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // 하강 엣지 인터럽트
  GPIO_InitStruct.Pull = GPIO_PULLUP;           // 내부 풀업
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // EEPROM CS 핀 설정 (추가)
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // NVIC 인터럽트 활성화
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

// ========== EEPROM 초기화 함수 ==========
// static HAL_StatusTypeDef eeprom_init(void)
// {
//     printf("[EEPROM] Initializing 25LC256...\n");
      
//     // 1단계: 기본 설정
//     eeprom_is_init_mode = true;
//     eeprom_cs_deselect();
//     eeprom_state = EEPROM_STATE_IDLE;
//     eeprom_dma_busy = false;
//     eeprom_write_in_progress = false;
//     dtc_log_count = 0;
//     current_eeprom_address = LC256_AREA_DTC_CURRENT;
    
//     memset(eeprom_tx_buffer, 0, sizeof(eeprom_tx_buffer));
//     memset(eeprom_rx_buffer, 0, sizeof(eeprom_rx_buffer));
    
//     HAL_Delay(10);
    
//     // 2단계: EEPROM 연결 확인 (Status Register 읽기)
//     printf("[EEPROM] Checking EEPROM connection...\n");
    
//     if (eeprom_wip_check() != HAL_OK) {
//         printf("[EEPROM] ERROR: Status read failed - Communication error\n");
//         return HAL_ERROR;
//     }
    
//     // 3단계: DMA 완료 대기
//     uint32_t timeout = HAL_GetTick() + 100;  // 100ms 타임아웃
//     while (eeprom_dma_busy && (HAL_GetTick() < timeout)) {
//         HAL_Delay(1);
//     }
    
//     // 4단계: 결과 확인
//     if (HAL_GetTick() >= timeout) {
//         printf("[EEPROM] ERROR: Status read timeout\n");
//         eeprom_state = EEPROM_STATE_IDLE;
//         eeprom_dma_busy = false;
//         return HAL_ERROR;
//     }
    
//     if (eeprom_state == EEPROM_STATE_ERROR) {
//         printf("[EEPROM] ERROR: Communication failed\n");
//         eeprom_state = EEPROM_STATE_IDLE;
//         return HAL_ERROR;
//     }
    
//     // 5단계: 성공
//     printf("[EEPROM] 25LC256 initialized successfully\n");
//     printf("[EEPROM] Status Register: 0x%02X\n", eeprom_rx_buffer[1]);
    
//     return HAL_OK;
// }

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
 void Error_Handler(void)
 {
   /* USER CODE BEGIN Error_Handler_Debug */
   /* User can add his own implementation to report the HAL error return state */
   __disable_irq();
   while (1)
   {
   }
   /* USER CODE END Error_Handler_Debug */
 }
