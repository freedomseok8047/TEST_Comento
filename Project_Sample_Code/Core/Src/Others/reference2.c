
 #include <stdio.h>
 #include <stdint.h>
 #include <stdbool.h>
 #include <string.h>
 #include "stm32f4xx_hal.h"
 #include "mp5457gu.h"
 
 // ========== PMIC 설정 상수 ==========
 #define MP5475_I2C_ADDRESS          0x68
 #define MP5475_REG_READ_SIZE        1
 
 // ========== I2C 상태 머신 정의 ==========
 typedef enum {
     I2C_STATE_IDLE = 0,
     I2C_STATE_RX_BUSY,
     I2C_STATE_RX_COMPLETE,
     I2C_STATE_ERROR
 } i2c_state_t;
 
 // ========== PMIC 레지스터 주소 정의 ==========
 typedef enum {
     MP5475_REG_SYSTEM_STATUS = 0x05,
     MP5475_REG_POWER_GOOD = 0x06,
     MP5475_REG_UV_OV_FAULT = 0x07,
     MP5475_REG_TEMP_FAULT = 0x09
 } mp5475_register_t;
 
 // ========== DTC 코드 정의 ==========
 typedef enum {
     DTC_BRAKE_PMIC_UV     = 0xC001,
     DTC_BRAKE_PMIC_OV     = 0xC002,
     DTC_BRAKE_PMIC_OC     = 0xC003,
     DTC_BRAKE_PMIC_TEMP   = 0xC004,
     DTC_BRAKE_COMM_ERROR  = 0xC005,
     DTC_BRAKE_SYSTEM_FAULT = 0xC006
 } brake_dtc_code_t;
 
 // ========== DTC 테이블 구조체 정의 ==========
 typedef struct {
     uint16_t DTC_Code;
     char Description[50];
     uint8_t active;
 } DTC_Table_t;
 
 // ========== 기존 전역 변수들 ==========
 DTC_Table_t DTC_Table = { 0x1234, "Brake UV Fault", 0 };
 
 // ========== I2C 상태 관리 변수 ==========
 i2c_state_t i2c_state = I2C_STATE_IDLE;
 uint8_t current_reg_index = 0;
 uint8_t pmic_reg_address = 0;
 uint8_t pmic_reg_data = 0;
 uint8_t i2c_rx_buffer[1] = {0};
 
 // ========== 읽을 레지스터 배열 ==========
 static const mp5475_register_t reg_addresses[] = {
     MP5475_REG_SYSTEM_STATUS,
     MP5475_REG_POWER_GOOD,
     MP5475_REG_UV_OV_FAULT,
     MP5475_REG_TEMP_FAULT
 };
 
 // ========== 읽은 데이터 저장 배열 ==========
 static uint8_t pmic_status_data[4];
 
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
 static DTC_Table_t detected_dtc_table[6];
 static uint8_t dtc_count = 0;
 
 // ========== 함수 프로토타입 선언 ==========
 static HAL_StatusTypeDef start_pmic_register_read(uint8_t reg_addr);
 static void process_pmic_register_data(uint8_t reg_addr, uint8_t reg_data);
 static void add_dtc_code(uint16_t dtc_code);
 static void print_dtc_summary(void);
 static void reset_pmic_diagnosis(void);
 
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
 
 // ========== I2C DMA 레지스터 읽기 시작 함수 ==========
 /**
  * @brief PMIC 레지스터 DMA 읽기 시작
  * @param reg_addr 읽을 레지스터 주소
  * @retval HAL_StatusTypeDef
  */
 static HAL_StatusTypeDef start_pmic_register_read(uint8_t reg_addr)
 {
     // 현재 레지스터 주소 저장
     pmic_reg_address = reg_addr;
     
     // 상태를 수신 대기로 변경
     i2c_state = I2C_STATE_RX_BUSY;
     
     // DMA 버퍼 초기화
     i2c_rx_buffer[0] = 0;
     
     printf("[I2C] Starting DMA read - Register: 0x%02X\n", reg_addr);
     
     // HAL_I2C_Mem_Read_DMA 호출
     HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(&hi2c1,
                                                     (MP5475_I2C_ADDRESS << 1),
                                                     reg_addr,
                                                     I2C_MEMADD_SIZE_8BIT,
                                                     i2c_rx_buffer,
                                                     MP5475_REG_READ_SIZE);
     
     // DMA 시작 결과 확인
     if (status != HAL_OK) {
         printf("[I2C] ERROR: Failed to start DMA read\n");
         i2c_state = I2C_STATE_ERROR;
         return status;
     }
     
     return HAL_OK;
 }
 
 // ========== I2C DMA 수신 완료 콜백 함수 ==========
 /**
  * @brief I2C DMA 수신 완료 콜백
  * @param hi2c I2C 핸들러 포인터
  * @note DMA 인터럽트에서 자동 호출
  */
 void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
 {
     // I2C1에서 발생한 콜백인지 확인
     if (hi2c->Instance == I2C1) {
         
         // DMA 버퍼에서 데이터 복사
         pmic_reg_data = i2c_rx_buffer[0];
         
         // 상태를 수신 완료로 변경
         i2c_state = I2C_STATE_RX_COMPLETE;
         
         printf("[I2C] DMA RX Complete - Reg: 0x%02X, Data: 0x%02X\n", 
                pmic_reg_address, pmic_reg_data);
         
         // 받은 데이터 처리
         process_pmic_register_data(pmic_reg_address, pmic_reg_data);
     }
 }
 
 // ========== I2C DMA 에러 콜백 함수 ==========
 /**
  * @brief I2C DMA 에러 콜백
  * @param hi2c I2C 핸들러 포인터
  */
 void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
 {
     if (hi2c->Instance == I2C1) {
         uint32_t error_code = HAL_I2C_GetError(hi2c);
         printf("[I2C] ERROR: DMA communication failed (Error: 0x%08X)\n", error_code);
         
         // 에러 상태로 변경
         i2c_state = I2C_STATE_ERROR;
         
         // 통신 에러 DTC 추가
         add_dtc_code(DTC_BRAKE_COMM_ERROR);
     }
 }
 
 // ========== PMIC 레지스터 데이터 처리 함수 ==========
 /**
  * @brief 읽은 PMIC 레지스터 데이터 분석 및 DTC 매핑
  * @param reg_addr 레지스터 주소
  * @param reg_data 레지스터 데이터
  */
 static void process_pmic_register_data(uint8_t reg_addr, uint8_t reg_data)
 {
     printf("[DIAG] Processing register 0x%02X with data 0x%02X\n", reg_addr, reg_data);
     
     // 읽은 데이터를 배열에 저장
     pmic_status_data[current_reg_index] = reg_data;
     
     // 레지스터별 데이터 분석
     switch (reg_addr) {
         
         case MP5475_REG_SYSTEM_STATUS:  // 0x05
             // 비트 7: 시스템 에러
             if (reg_data & 0x80) {
                 printf("[FAULT] System error detected\n");
                 add_dtc_code(DTC_BRAKE_SYSTEM_FAULT);
             }
             // 비트 6: 온도 경고
             if (reg_data & 0x40) {
                 printf("[FAULT] Temperature warning\n");
                 add_dtc_code(DTC_BRAKE_PMIC_TEMP);
             }
             break;
             
         case MP5475_REG_POWER_GOOD:     // 0x06
             // 비트별 Power Good 확인
             for (int i = 0; i < 4; i++) {
                 if (!(reg_data & (1 << i))) {
                     printf("[FAULT] Buck %d Power Good FAIL\n", i+1);
                     add_dtc_code(DTC_BRAKE_SYSTEM_FAULT);
                 }
             }
             break;
             
         case MP5475_REG_UV_OV_FAULT:    // 0x07
             // 비트 7-6: Buck A UV/OV
             if (reg_data & 0x80) {
                 printf("[FAULT] Buck A Overvoltage\n");
                 add_dtc_code(DTC_BRAKE_PMIC_OV);
             }
             if (reg_data & 0x40) {
                 printf("[FAULT] Buck A Undervoltage\n");
                 add_dtc_code(DTC_BRAKE_PMIC_UV);
             }
             // 비트 5-4: Buck B UV/OV
             if (reg_data & 0x20) {
                 printf("[FAULT] Buck B Overvoltage\n");
                 add_dtc_code(DTC_BRAKE_PMIC_OV);
             }
             if (reg_data & 0x10) {
                 printf("[FAULT] Buck B Undervoltage\n");
                 add_dtc_code(DTC_BRAKE_PMIC_UV);
             }
             break;
             
         case MP5475_REG_TEMP_FAULT:     // 0x09
             // 비트 7: 과온도 차단
             if (reg_data & 0x80) {
                 printf("[FAULT] Over-temperature shutdown\n");
                 add_dtc_code(DTC_BRAKE_PMIC_TEMP);
             }
             // 비트 6: 과온도 경고
             if (reg_data & 0x40) {
                 printf("[WARNING] Over-temperature warning\n");
                 add_dtc_code(DTC_BRAKE_PMIC_TEMP);
             }
             break;
     }
 }
 
 // ========== DTC 코드 추가 함수 ==========
 /**
  * @brief DTC 코드를 테이블에 추가
  * @param dtc_code 추가할 DTC 코드
  */
 static void add_dtc_code(uint16_t dtc_code)
 {
     // 중복 DTC 확인
     for (uint8_t i = 0; i < dtc_count; i++) {
         if (detected_dtc_table[i].DTC_Code == dtc_code) {
             return;  // 이미 존재
         }
     }
     
     // 테이블 공간 확인
     if (dtc_count >= 6) {
         printf("[DTC] ERROR: DTC table full\n");
         return;
     }
     
     // 마스터 테이블에서 DTC 정보 찾기
     for (uint8_t i = 0; i < 6; i++) {
         if (dtc_master_table[i].DTC_Code == dtc_code) {
             // DTC 테이블에 추가
             detected_dtc_table[dtc_count].DTC_Code = dtc_code;
             strcpy(detected_dtc_table[dtc_count].Description, dtc_master_table[i].Description);
             detected_dtc_table[dtc_count].active = 1;
             
             printf("[DTC] Added: 0x%04X - %s\n", dtc_code, dtc_master_table[i].Description);
             dtc_count++;
             return;
         }
     }
 }
 
 // ========== 진단 상태 초기화 함수 ==========
 /**
  * @brief PMIC 진단 상태 초기화
  */
 static void reset_pmic_diagnosis(void)
 {
     current_reg_index = 0;
     dtc_count = 0;
     
     // 데이터 배열 초기화
     for (uint8_t i = 0; i < 4; i++) {
         pmic_status_data[i] = 0;
     }
     
     // DTC 테이블 초기화
     for (uint8_t i = 0; i < 6; i++) {
         detected_dtc_table[i].DTC_Code = 0;
         detected_dtc_table[i].Description[0] = '\0';
         detected_dtc_table[i].active = 0;
     }
     
     pmic_reg_address = 0;
     pmic_reg_data = 0;
 }
 
 // ========== DTC 요약 출력 함수 ==========
 /**
  * @brief 감지된 DTC 요약 출력
  */
 static void print_dtc_summary(void)
 {
     printf("\n========== DTC SUMMARY ==========\n");
     printf("Total detected DTCs: %d\n", dtc_count);
     
     if (dtc_count == 0) {
         printf("No faults detected - System OK\n");
     } else {
         for (uint8_t i = 0; i < dtc_count; i++) {
             printf("DTC[%d]: 0x%04X - %s\n",
                    i + 1,
                    detected_dtc_table[i].DTC_Code,
                    detected_dtc_table[i].Description);
         }
     }
     printf("================================\n\n");
 }
 
 // ========== 시스템 초기화 함수 ==========
 void system_init(void) 
 {
     printf("[INIT] System initialization started ...\n");
 
     MX_GPIO_Init();
     MX_DMA_Init();
     MX_ADC1_Init();
     MX_CAN1_Init();
     MX_I2C1_Init();
     MX_I2C2_Init();
     MX_SPI1_Init();
     MX_SPI2_Init();
     MX_UART4_Init();
 
     printf("[INIT] System initialization completed!...\n");
 }
 
 // ========== I2C 통신 처리 메인 함수 (기존 함수 대체) ==========
 /**
  * @brief I2C 통신 상태 머신 처리
  */
 void process_i2c_step(void)
 {
     printf("[I2C] Processing PMIC status check - State: %d\n", i2c_state);
     
     switch (i2c_state) {
         
         case I2C_STATE_IDLE:
             // 새로운 진단 시작
             printf("[I2C] Starting PMIC diagnosis\n");
             reset_pmic_diagnosis();
             
             current_reg_index = 0;
             if (start_pmic_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
                 i2c_state = I2C_STATE_ERROR;
             }
             break;
             
         case I2C_STATE_RX_BUSY:
             // DMA 수신 진행 중 - 대기
             printf("[I2C] DMA RX in progress...\n");
             break;
             
         case I2C_STATE_RX_COMPLETE:
             // 수신 완료 - 다음 레지스터 또는 완료
             current_reg_index++;
             
             if (current_reg_index >= 4) {
                 // 모든 레지스터 읽기 완료
                 printf("[I2C] All registers read, diagnosis complete\n");
                 print_dtc_summary();
                 i2c_state = I2C_STATE_IDLE;
             } else {
                 // 다음 레지스터 읽기
                 if (start_pmic_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
                     i2c_state = I2C_STATE_ERROR;
                 }
             }
             break;
             
         case I2C_STATE_ERROR:
             // 에러 처리 - I2C 재초기화
             printf("[I2C] Error recovery\n");
             HAL_I2C_DeInit(&hi2c1);
             MX_I2C1_Init();
             i2c_state = I2C_STATE_IDLE;
             break;
     }
 }
 
 // ========== SPI 통신 처리 함수 (기존 코드 유지) ==========
 void process_spi_step(void){
     printf("[SPI] Processing EEPROM data...\n");
     printf("[SPI] EEPROM operation completed! \n");
 }
 
 // ========== CAN 통신 처리 함수 (기존 코드 유지) ==========
 void process_can_step(void)
 {
     printf("[CAN] Processing diagnostic data transmission...\n");
     printf("[CAN] Diagnostic data transmitted! \n");
 }
 
 // ========== UART 통신 처리 함수 (기존 코드 유지) ==========
 void process_uart_step(void)
 {
     printf("[UART] Processing log output...\n");
     printf("[UART] System status : All systems operational \n");
 }
 
 // ========== 메인 함수 (기존 코드 유지) ==========
 int main(void)
 {
     printf("==== 브레이크 시스템 PMIC 제어 시작 === \n");
     printf("HW : MP5475GU + 25LC256 + TJA1051\n\n");
 
     HAL_Init();
     SystemClock_Config();
     system_init();
 
     printf("\n=== Main Loop 시작 ===\n");
     printf("동작 순서 : I2C -> SPI -> CAN -> UART \n\n");
 
     uint32_t loop_count = 0;
 
     while(1)
     {
         printf("--- Loop #%d --- \n", ++loop_count);
 
         process_i2c_step();
         process_spi_step();
         process_can_step();
         process_uart_step();
 
         printf("[DELAY] 100ms delay...\n");
         HAL_Delay(10);
 
         if(loop_count >= 3){
             printf("==시뮬레이션 완료 ==");
             break;
         }
 
         printf("\n");
     }
 }
 
 // ========== 기존 초기화 함수들 (STM32CubeMX 생성 코드들) ==========
 void SystemClock_Config(void)
 {
     // 기존 코드 유지
 }
 
 static void MX_ADC1_Init(void)
 {
     // 기존 코드 유지
 }
 
 static void MX_CAN1_Init(void)
 {
     // 기존 코드 유지
 }
 
 static void MX_I2C1_Init(void)
 {
     // 기존 코드 유지
 }
 
 static void MX_I2C2_Init(void)
 {
     // 기존 코드 유지
 }
 
 static void MX_SPI1_Init(void)
 {
     // 기존 코드 유지
 }
 
 static void MX_SPI2_Init(void)
 {
     // 기존 코드 유지
 }
 
 static void MX_UART4_Init(void)
 {
     // 기존 코드 유지
 }
 
 static void MX_DMA_Init(void)
 {
     // 기존 코드 유지
 }
 
 static void MX_GPIO_Init(void)
 {
     // 기존 코드 유지
 }