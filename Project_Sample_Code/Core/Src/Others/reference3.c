#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h> 
#include "stm32f4xx_hal.h"
#include "mp5457gu.h"
#include "25lc256.h"  // EEPROM 헤더 추가

// ========== PMIC 설정 상수 (기존 유지) ==========
#define MP5475_I2C_ADDRESS          0x60
#define MP5475_REG_READ_SIZE        1

// 외부 변수 참조
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern SPI_HandleTypeDef hspi1, hspi2;
extern UART_HandleTypeDef huart4;

// ========== I2C 상태 머신 (기존 유지) ==========
typedef enum {
	I2C_STATE_IDLE = 0,
	I2C_STATE_RX_BUSY,
	I2C_STATE_RX_COMPLETE,
	I2C_STATE_ERROR
} i2c_state_t;

// ========== EEPROM 상태 머신 (추가) ==========
typedef enum {
    EEPROM_STATE_IDLE = 0,
    EEPROM_STATE_WRITE_ENABLE,
    EEPROM_STATE_WRITING,
    EEPROM_STATE_READ_STATUS,
    EEPROM_STATE_COMPLETE,
    EEPROM_STATE_ERROR
} eeprom_state_t;

 // ========== PMIC 레지스터 주소 정의 (기존 유지) ==========
 typedef enum {
	MP5475_REG_SYSTEM_STATUS = 0x05,
	MP5475_REG_POWER_GOOD = 0x06,
	MP5475_REG_UV_OV_FAULT = 0x07,
	MP5475_REG_OC_FAULT = 0x08,
	MP5475_REG_TEMP_FAULT = 0x09
} mp5475_register_t;

 // ========== DTC 코드 정의 (기존 유지) ==========
typedef enum {
    DTC_BRAKE_PMIC_UV     = 0xC001,
    DTC_BRAKE_PMIC_OV     = 0xC002,
    DTC_BRAKE_PMIC_OC     = 0xC003,
    DTC_BRAKE_PMIC_TEMP   = 0xC004,
    DTC_BRAKE_COMM_ERROR  = 0xC005,
    DTC_BRAKE_SYSTEM_FAULT = 0xC006
} brake_dtc_code_t;

// --- DTC 데이터 구조 정의 (기존 유지) ---
#pragma pack(push, 1)
typedef struct {
	uint16_t DTC_Code;
	char Description[50];
	uint8_t active;
  } DTC_Table_t;
#pragma pack(pop) 

DTC_Table_t DTC_Table = { 0x1234, "Brake UV Fault", 0 };

 // ========== I2C 상태 관리 변수 (기존 유지) ==========
volatile i2c_state_t i2c_state = I2C_STATE_IDLE;
uint8_t current_reg_index = 0;
uint8_t pmic_reg_address = 0;
uint8_t pmic_reg_data = 0;
uint8_t i2c_rx_buffer[1] = {0};

// ========== EEPROM 상태 관리 변수 (추가) ==========
volatile eeprom_state_t eeprom_state = EEPROM_STATE_IDLE;
static volatile bool eeprom_dma_busy = false;
static volatile bool eeprom_write_in_progress = false;
static uint8_t current_dtc_save_index = 0;

// EEPROM DMA 버퍼 (추가)
static uint8_t eeprom_tx_buffer[67] __attribute__((aligned(4)));  // CMD(1) + ADDR(2) + DATA(64)
static uint8_t eeprom_rx_buffer[67] __attribute__((aligned(4)));
static uint16_t current_eeprom_address = LC256_AREA_DTC_CURRENT;
static uint16_t dtc_log_count = 0;

// EEPROM용 DTC 로그 구조체 (64바이트)
typedef struct {
    uint32_t timestamp;
    uint16_t DTC_Code;
    char Description[48];
    uint8_t active;
    uint8_t occurrence_count;
    uint8_t status;
    uint8_t reserved[9];
} __attribute__((packed)) eeprom_dtc_log_t;

static eeprom_dtc_log_t current_dtc_log;

// GPIO 핀 정의 (EEPROM CS 추가)
#define EEPROM_CS_PORT    GPIOB
#define EEPROM_CS_PIN     GPIO_PIN_4

// 읽을 레지스터 주소 배열 (기존 유지)
static const mp5475_register_t reg_addresses[] = {
	  MP5475_REG_SYSTEM_STATUS,
    MP5475_REG_POWER_GOOD,
    MP5475_REG_UV_OV_FAULT,
    MP5475_REG_TEMP_FAULT
};

// 읽은 레지스터 데이터 저장 배열 (기존 유지)
static uint8_t pmic_status_data[4];

 // ========== DTC 마스터 테이블 (기존 유지) ==========
static const DTC_Table_t dtc_master_table[] = {
    {DTC_BRAKE_PMIC_UV,     "Buck Undervoltage Fault",     0},
    {DTC_BRAKE_PMIC_OV,     "Buck Overvoltage Fault",      0},
    {DTC_BRAKE_PMIC_OC,     "Buck Overcurrent Fault",      0},
    {DTC_BRAKE_PMIC_TEMP,   "PMIC Temperature Fault",      0},
    {DTC_BRAKE_COMM_ERROR,  "I2C Communication Error",     0},
    {DTC_BRAKE_SYSTEM_FAULT,"System/Power Good Fault",     0}
};

 // ========== 감지된 DTC 저장 배열 (기존 유지) ==========
static DTC_Table_t detected_dtc_table[6];
static uint8_t dtc_count = 0;

/* ========== 함수 프로토타입 (기존 유지) ========== */
static HAL_StatusTypeDef start_pmic_register_read(uint8_t reg_addr);
static void process_pmic_register_data(uint8_t reg_addr, uint8_t reg_data);
static void add_dtc_code(uint16_t dtc_code);
static void print_dtc_summary(void);
static void reset_pmic_diagnosis(void);

/* ========== EEPROM 함수 프로토타입 (추가) ========== */
static void eeprom_cs_select(void);
static void eeprom_cs_deselect(void);
static HAL_StatusTypeDef eeprom_write_enable(void);
static HAL_StatusTypeDef eeprom_read_status(void);
static HAL_StatusTypeDef eeprom_write_dtc_log(uint16_t address, eeprom_dtc_log_t* dtc_log);
static bool eeprom_is_write_complete(void);
static void save_dtc_to_eeprom(void);
static void convert_dtc_to_log(DTC_Table_t* dtc, eeprom_dtc_log_t* log);
static uint16_t get_next_eeprom_address(void);
static HAL_StatusTypeDef eeprom_init(void);

// Init 프로토 타입 선언 (기존 유지)
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

// ========== EEPROM CS 핀 제어 함수 (추가) ==========
static void eeprom_cs_select(void)
{
    HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_RESET);
}

static void eeprom_cs_deselect(void)
{
    HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_SET);
}

// ========== EEPROM Write Enable 함수 (추가) ==========
static HAL_StatusTypeDef eeprom_write_enable(void)
{
    if (eeprom_dma_busy) {
        return HAL_BUSY;
    }
    
    printf("[EEPROM] Sending Write Enable command (0x06)\n");
    
    eeprom_dma_busy = true;
    eeprom_tx_buffer[0] = LC256_CMD_WREN;
    
    eeprom_cs_select();
    eeprom_state = EEPROM_STATE_WRITE_ENABLE;
    
    HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi1, eeprom_tx_buffer, 1);
    
    if (status != HAL_OK) {
        printf("[EEPROM] ERROR: Write Enable failed\n");
        eeprom_cs_deselect();
        eeprom_state = EEPROM_STATE_ERROR;
        eeprom_dma_busy = false;
    }
    
    return status;
}

// ========== EEPROM Status 읽기 함수 (추가) ==========
static HAL_StatusTypeDef eeprom_read_status(void)
{
    if (eeprom_dma_busy) {
        return HAL_BUSY;
    }
    
    printf("[EEPROM] Reading status register\n");
    
    eeprom_dma_busy = true;
    eeprom_tx_buffer[0] = LC256_CMD_RDSR;
    eeprom_tx_buffer[1] = 0x00;
    
    eeprom_cs_select();
    eeprom_state = EEPROM_STATE_READ_STATUS;
    
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(&hspi1, eeprom_tx_buffer, eeprom_rx_buffer, 2);
    
    if (status != HAL_OK) {
        printf("[EEPROM] ERROR: Status read failed\n");
        eeprom_cs_deselect();
        eeprom_state = EEPROM_STATE_ERROR;
        eeprom_dma_busy = false;
    }
    
    return status;
}

// ========== Write 완료 확인 함수 (추가) ==========
static bool eeprom_is_write_complete(void)
{
    return !(eeprom_rx_buffer[1] & 0x01);  // WIP 비트 체크
}

// ========== DTC 로그 변환 함수 (추가) ==========
static void convert_dtc_to_log(DTC_Table_t* dtc, eeprom_dtc_log_t* log)
{
    memset(log, 0, sizeof(eeprom_dtc_log_t));
    
    log->timestamp = HAL_GetTick();
    log->DTC_Code = dtc->DTC_Code;
    strncpy(log->Description, dtc->Description, 47);
    log->Description[47] = '\0';
    log->active = dtc->active;
    log->occurrence_count = 1;
    log->status = 0x01;
}

// ========== EEPROM DTC 쓰기 함수 (추가) ==========
static HAL_StatusTypeDef eeprom_write_dtc_log(uint16_t address, eeprom_dtc_log_t* dtc_log)
{
    if (eeprom_dma_busy) {
        return HAL_BUSY;
    }
    
    printf("[EEPROM] Writing DTC log to address 0x%04X\n", address);
    
    eeprom_dma_busy = true;
    eeprom_write_in_progress = true;
    
    eeprom_tx_buffer[0] = LC256_CMD_WRITE;
    eeprom_tx_buffer[1] = (address >> 8) & 0x7F;
    eeprom_tx_buffer[2] = address & 0xFF;
    
    memcpy(&eeprom_tx_buffer[3], dtc_log, sizeof(eeprom_dtc_log_t));
    
    eeprom_cs_select();
    eeprom_state = EEPROM_STATE_WRITING;
    
    HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi1, eeprom_tx_buffer, 3 + sizeof(eeprom_dtc_log_t));
    
    if (status != HAL_OK) {
        printf("[EEPROM] ERROR: DTC write failed\n");
        eeprom_cs_deselect();
        eeprom_state = EEPROM_STATE_ERROR;
        eeprom_dma_busy = false;
        eeprom_write_in_progress = false;
    }
    
    return status;
}

// ========== 다음 주소 계산 함수 (추가) ==========
static uint16_t get_next_eeprom_address(void)
{
    uint16_t next_addr = LC256_AREA_DTC_CURRENT + (dtc_log_count * sizeof(eeprom_dtc_log_t));
    
    if (next_addr + sizeof(eeprom_dtc_log_t) >= LC256_AREA_DTC_HISTORY) {
        next_addr = LC256_AREA_DTC_CURRENT;
        dtc_log_count = 0;
    }
    
    return next_addr;
}

// ========== EEPROM 초기화 함수 (추가) ==========
static HAL_StatusTypeDef eeprom_init(void)
{
    printf("[EEPROM] Initializing 25LC256...\n");
    
    eeprom_cs_deselect();
    eeprom_state = EEPROM_STATE_IDLE;
    eeprom_dma_busy = false;
    eeprom_write_in_progress = false;
    
    memset(eeprom_tx_buffer, 0, sizeof(eeprom_tx_buffer));
    memset(eeprom_rx_buffer, 0, sizeof(eeprom_rx_buffer));
    
    HAL_Delay(10);
    
    printf("[EEPROM] 25LC256 initialized successfully\n");
    return HAL_OK;
}

// ========== DTC EEPROM 저장 시작 함수 (추가) ==========
static void save_dtc_to_eeprom(void)
{
    if (dtc_count == 0) {
        printf("[EEPROM] No DTCs to save\n");
        return;
    }
    
    if (eeprom_state != EEPROM_STATE_IDLE) {
        printf("[EEPROM] EEPROM busy, cannot save DTCs\n");
        return;
    }
    
    printf("[EEPROM] Starting DTC save process (%d DTCs)\n", dtc_count);
    
    current_dtc_save_index = 0;
    current_eeprom_address = get_next_eeprom_address();
    
    convert_dtc_to_log(&detected_dtc_table[current_dtc_save_index], &current_dtc_log);
    
    if (eeprom_write_enable() != HAL_OK) {
        printf("[EEPROM] ERROR: Failed to start DTC save\n");
        eeprom_state = EEPROM_STATE_ERROR;
    }
}

// ========== PMIC IRQ 콜백 (기존 유지) ==========
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_3) {
        printf("[IRQ] PMIC fault detected, starting diagnosis\n");
        
        if (i2c_state == I2C_STATE_IDLE) {
            reset_pmic_diagnosis();
            current_reg_index = 0;
            if (start_pmic_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
                i2c_state = I2C_STATE_ERROR;
            }
        } else{
            printf("[WARNING] I2C busy, IRQ ignored\n");
        }
    }
}

// ========== I2C 레지스터 읽기 함수 (기존 유지) ==========
static HAL_StatusTypeDef start_pmic_register_read(uint8_t reg_addr)
{
    pmic_reg_address = reg_addr;
    i2c_state = I2C_STATE_RX_BUSY;
    i2c_rx_buffer[0] = 0;
    
    printf("[I2C] Starting DMA read - Register: 0x%02X\n", reg_addr);
    
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(&hi2c1,
                                                    (MP5475_I2C_ADDRESS << 1),
                                                    reg_addr,
                                                    I2C_MEMADD_SIZE_8BIT,
                                                    i2c_rx_buffer,
                                                    MP5475_REG_READ_SIZE);
    if (status != HAL_OK){
      printf("[I2C] ERROR: Failed to start DMA read\n");
      i2c_state = I2C_STATE_ERROR;
      return status;
    }

    return HAL_OK;
}

// ========== I2C DMA 수신 완료 콜백 (기존 유지) ==========
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c -> Instance == I2C1){
		pmic_reg_data = i2c_rx_buffer[0];
		i2c_state = I2C_STATE_RX_COMPLETE;

		printf("[I2C] DMA RX Complete - Reg: 0x%02X, data: 0x%02X\n", pmic_reg_address,pmic_reg_data);
		
		process_pmic_register_data(pmic_reg_address, pmic_reg_data);
		
        current_reg_index++;
        
        if(current_reg_index < 4) {
          if(start_pmic_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
              i2c_state = I2C_STATE_ERROR;
          }
        } else {
          printf("[I2C] All registers read, diagnosis complete\n");
          print_dtc_summary();
          i2c_state = I2C_STATE_IDLE;
        }
	}
}

// ========== I2C 에러 콜백 (기존 유지) ==========
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c -> Instance == I2C1) {
		uint32_t error_code = HAL_I2C_GetError(hi2c);
		printf("[I2C] ERROR: DMA communication failed (Error: 0x%08lX)\n",error_code);
		i2c_state = I2C_STATE_ERROR;
		add_dtc_code(DTC_BRAKE_COMM_ERROR);
	}
}

// ========== SPI DMA 송신 완료 콜백 (추가) ==========
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {  // EEPROM용 SPI1
        
        eeprom_cs_deselect();
        eeprom_dma_busy = false;
        
        switch (eeprom_state) {
            case EEPROM_STATE_WRITE_ENABLE:
                printf("[EEPROM] Write Enable completed\n");
                HAL_Delay(1);
                
                if (eeprom_write_dtc_log(current_eeprom_address, &current_dtc_log) != HAL_OK) {
                    eeprom_state = EEPROM_STATE_ERROR;
                }
                break;
                
            case EEPROM_STATE_WRITING:
                printf("[EEPROM] DTC write completed, checking status\n");
                HAL_Delay(LC256_WRITE_CYCLE_TIME);
                
                if (eeprom_read_status() != HAL_OK) {
                    eeprom_state = EEPROM_STATE_ERROR;
                }
                break;
                
            default:
                break;
        }
    }
}

// ========== SPI DMA 송수신 완료 콜백 (추가) ==========
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        
        eeprom_cs_deselect();
        eeprom_dma_busy = false;
        
        if (eeprom_state == EEPROM_STATE_READ_STATUS) {
            printf("[EEPROM] Status: 0x%02X\n", eeprom_rx_buffer[1]);
            
            if (eeprom_is_write_complete()) {
                printf("[EEPROM] Write completed successfully\n");
                
                dtc_log_count++;
                current_dtc_save_index++;
                
                if (current_dtc_save_index < dtc_count) {
                    printf("[EEPROM] Processing next DTC (%d/%d)\n", 
                           current_dtc_save_index + 1, dtc_count);
                    
                    current_eeprom_address = get_next_eeprom_address();
                    convert_dtc_to_log(&detected_dtc_table[current_dtc_save_index], &current_dtc_log);
                    
                    if (eeprom_write_enable() != HAL_OK) {
                        eeprom_state = EEPROM_STATE_ERROR;
                    }
                } else {
                    printf("[EEPROM] All DTCs saved successfully\n");
                    eeprom_state = EEPROM_STATE_COMPLETE;
                    eeprom_write_in_progress = false;
                }
                
            } else {
                printf("[EEPROM] Write in progress, checking again...\n");
                HAL_Delay(1);
                if (eeprom_read_status() != HAL_OK) {
                    eeprom_state = EEPROM_STATE_ERROR;
                }
            }
        }
    }
}

// ========== SPI 에러 콜백 (추가) ==========
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        uint32_t error_code = HAL_SPI_GetError(hspi);
        printf("[EEPROM] ERROR: SPI failed (Error: 0x%08lX)\n", error_code);
        
        eeprom_cs_deselect();
        eeprom_state = EEPROM_STATE_ERROR;
        eeprom_dma_busy = false;
        eeprom_write_in_progress = false;
    }
}

// ========== PMIC 데이터 처리 함수 (기존 유지) ==========
static void process_pmic_register_data(uint8_t reg_addr, uint8_t reg_data)
{
	printf("[DIAG] Processing register 0x%02X with data 0x%02X\n",reg_addr,reg_data);
	pmic_status_data[current_reg_index] = reg_data;
	
	switch (reg_addr) {
		case MP5475_REG_POWER_GOOD:
			if((reg_data & 0xF0) != 0xF0){
				printf("[FAULT] Power Good FAIL - Data: 0x%02X\n", reg_data);
        		add_dtc_code(DTC_BRAKE_SYSTEM_FAULT);
			}
			break;
		case MP5475_REG_UV_OV_FAULT:
			if (reg_data & 0x80 || reg_data & 0x40 || reg_data & 0x20 || reg_data & 0x10) {
				printf("[FAULT] PMIC UV detected - Data: 0x%02X\n", reg_data);
				add_dtc_code(DTC_BRAKE_PMIC_UV);
			}else if (reg_data & 0x08 || reg_data & 0x04 || reg_data & 0x02 || reg_data & 0x01) {
				printf("[FAULT] PMIC OV detected - Data: 0x%02X\n", reg_data);
				add_dtc_code(DTC_BRAKE_PMIC_OV);
			}
			break;
		case MP5475_REG_OC_FAULT:
			if(reg_data & 0x80 || reg_data & 0x40 || reg_data & 0x20 || reg_data & 0x10){
				printf("[FAULT] PMIC OC detected - Data: 0x%02X\n", reg_data);
				add_dtc_code(DTC_BRAKE_PMIC_OC);
			}
			if (reg_data & 0x08 || reg_data & 0x04 || reg_data & 0x02 || reg_data & 0x01) {
				printf("[WARNING] PMIC OC WARNING - Data: 0x%02X\n", reg_data);
			}
			break;
		case MP5475_REG_TEMP_FAULT:
			if(reg_data & 0x01){
				printf("[FAULT] PMIC High Temperature - Data: 0x%02X\n", reg_data);
				add_dtc_code(DTC_BRAKE_PMIC_TEMP);
			}
			if (reg_data & 0x02) {
				printf("[WARNING] PMIC High Temp Warning - Data: 0x%02X\n", reg_data);
			}
			break;
	}
}

// ========== DTC 추가 함수 (기존 유지) ==========
static void add_dtc_code(uint16_t dtc_code)
{
	for (uint8_t i = 0; i < dtc_count; i++ ){
		if(detected_dtc_table[i].DTC_Code == dtc_code){
			return;
		}
	}

	if (dtc_count >= 6){
		printf("[DTC] ERROR: DTC table full\n");
		return;
	}

	for (uint8_t i = 0 ; i < 6; i++){
		if (dtc_master_table[i].DTC_Code == dtc_code){
			detected_dtc_table[dtc_count].DTC_Code = dtc_code;
			strcpy(detected_dtc_table[dtc_count].Description, dtc_master_table[i].Description) ;
			detected_dtc_table[dtc_count].active = 1;
			printf("[DTC] Added: 0x%04X - %s\n", dtc_code, dtc_master_table[i].Description);
			dtc_count++;
			return;
		}
	}
}

// ========== 진단 초기화 함수 (기존 유지) ==========
static void reset_pmic_diagnosis(void)
{
	 current_reg_index = 0;
	 dtc_count = 0;
 
	 for (uint8_t i = 0; i < sizeof(pmic_status_data); i++) {
		 pmic_status_data[i] = 0;
	 }
 
	 for (uint8_t i = 0; i < sizeof(detected_dtc_table) / sizeof(detected_dtc_table[0]); i++) {
		 detected_dtc_table[i].DTC_Code = 0;
		 detected_dtc_table[i].Description[0] = '\0';
		 detected_dtc_table[i].active = 0;
	 }
 
	 pmic_reg_address = 0;
	 pmic_reg_data = 0;
 
	 printf("[DIAG] PMIC diagnosis state reset completed\n");
}

// ========== DTC 요약 출력 함수 (EEPROM 저장 추가) ==========
static void print_dtc_summary(void)
{
    printf("\n========= DTC SUMMARY ===========\n");
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
        
        // EEPROM에 DTC 저장 시작
        printf("\n[EEPROM] === Starting DTC save to EEPROM ===\n");
        save_dtc_to_eeprom();
    }
    printf("===================================\n\n");
}

// ========== 시스템 초기화 함수 (EEPROM 초기화 추가) ==========
void system_init(void) 
{
	printf("[INIT] System initialization started ...\n");

	// 하드웨어 초기화 (기존 유지)
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

	printf("[INIT] System initialization completed!\n");
}

// ========== SPI 처리 함수 (EEPROM 상태 관리 추가) ==========
void process_spi_step(void)
{
	// EEPROM 상태에 따른 처리
	switch (eeprom_state) {
		case EEPROM_STATE_IDLE:
			// 유휴 상태
			break;
			
		case EEPROM_STATE_COMPLETE:
			printf("[SPI] EEPROM write operation completed successfully\n");
			eeprom_state = EEPROM_STATE_IDLE;
			break;
			
		case EEPROM_STATE_ERROR:
			printf("[SPI] EEPROM error detected, attempting recovery\n");
			
			// SPI 재초기화
			HAL_SPI_DeInit(&hspi1);
			if (MX_SPI1_Init() == HAL_OK) {
				printf("[SPI] SPI1 reinitialized successfully\n");
			} else {
				printf("[SPI] ERROR: SPI1 reinitialization failed\n");
			}
			
			// 상태 초기화
			eeprom_state = EEPROM_STATE_IDLE;
			eeprom_dma_busy = false;
			eeprom_write_in_progress = false;
			
			printf("[SPI] Recovery completed\n");
			break;
			
		default:
			// DMA 콜백에서 처리되는 상태들
			break;
	}
}

// ========== CAN 통신 처리 함수 (기존 유지) ==========
void process_can_step(void)
{
	printf("[CAN] Processing diagnostic data transmission...\n");
	// TODO: CAN 프레임 준비, DTC 전송, Clear 요청 처리
	printf("[CAN] Diagnostic data transmitted!\n");
}

// ========== UART 통신 처리 함수 (기존 유지) ==========
void process_uart_step(void)
{
	printf("[UART] Processing log output...\n");
	// TODO: 시스템 상태 로그, 정상 동작 메시지, 에러 정보
	printf("[UART] System status: All systems operational\n");
}

// ========== 메인 함수 ==========
int main(void)
{
	// 시스템 초기화
    HAL_Init();
    SystemClock_Config();
    system_init();

	uint32_t loop_count = 0;
	uint32_t last_status_print = 0;

	printf("[MAIN] System started successfully!\n");
	printf("[MAIN] Waiting for PMIC IRQ events...\n");

	while(1)
	{
		// 상태 처리 함수들 호출
		process_spi_step();
		// process_can_step();  // 필요시 활성화
		// process_uart_step(); // 필요시 활성화

		// 주기적 상태 출력 (디버깅용)
		if (HAL_GetTick() - last_status_print > 10000) {  // 10초마다
			if (eeprom_state != EEPROM_STATE_IDLE || i2c_state != I2C_STATE_IDLE) {
				printf("[STATUS] I2C: %d, EEPROM: %d, DMA_BUSY: %s\n", 
				       i2c_state, eeprom_state, 
				       eeprom_dma_busy ? "Yes" : "No");
			}
			last_status_print = HAL_GetTick();
		}

		loop_count++;
		HAL_Delay(10);
	}
}

// ========== 시스템 클럭 설정 (기존 유지) ==========
void SystemClock_Config(void)
{
  // 기존 코드 유지
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
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

// ========== ADC1 초기화 (기존 유지) ==========
static void MX_ADC1_Init(void)
{
  // 기존 코드 유지
  ADC_ChannelConfTypeDef sConfig = {0};

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
  
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

// ========== CAN1 초기화 (기존 유지) ==========
static void MX_CAN1_Init(void)
{
  // 기존 코드 유지
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
}

// ========== I2C1 초기화 (기존 유지) ==========
static void MX_I2C1_Init(void)
{
  // 기존 코드 유지
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
}

// ========== I2C2 초기화 (기존 유지) ==========
static void MX_I2C2_Init(void)
{
  // 기존 코드 유지
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
}

// ========== SPI1 초기화 (EEPROM용, 데이터시트 기준 수정) ==========
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;     // CPOL = 0 (25LC256 Mode 0)
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;         // CPHA = 0 (25LC256 Mode 0)
  hspi1.Init.NSS = SPI_NSS_SOFT;                 // CS 핀 수동 제어
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  // 10MHz (VCC >= 4.5V)
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;        // MSB First
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

// ========== SPI2 초기화 (기존 유지) ==========
static void MX_SPI2_Init(void)
{
  // 기존 코드 유지
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
}

// ========== UART4 초기화 (기존 유지) ==========
static void MX_UART4_Init(void)
{
  // 기존 코드 유지
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
}

// ========== DMA 초기화 (기존 유지) ==========
static void MX_DMA_Init(void)
{
  // 기존 코드 유지
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  // DMA interrupt init
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

// ========== GPIO 초기화 (EEPROM CS 핀 추가) ==========
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // GPIO Ports Clock Enable (기존 유지)
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  // Configure GPIO pin Output Level (기존 유지)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_SET);

  // Configure GPIO pins : PB0 PB1 (기존 유지)
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure GPIO pin : PB2 (기존 유지)
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // PMIC IRQ 핀 설정 (기존 유지)
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // EEPROM CS 핀 설정 (추가)
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // NVIC 인터럽트 활성화 (기존 유지)
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

// ========== 에러 핸들러 (기존 유지) ==========
void Error_Handler(void)
{
   __disable_irq();
   while (1)
   {
   }
}