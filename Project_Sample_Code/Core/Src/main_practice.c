#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

// 시스템 헤더 (나중에 추가될 예정)
// #include "mp5475gu.h"    // PMIC I2C
// #include "25lc256.h"     // EEPROM SPI  
// #include "tja1051.h"     // CAN Transceiver
// #include "uart_debug.h"  // UART Debug

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

/**
 * @brief 시스템 초기화 함수
 * @note 모든 하드웨어 초기화를 담당
 */
void system_init(void) {
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

	printf("[INIT] System initialization completed!...\n");
}

// --- DTC 데이터 구조 정의 ---
typedef struct {
  uint16_t DTC_Code;              // 고장 코드 (예: C1234)
  char Description[50];           // 설명 문자열
  uint8_t active;                 // 활성화 상태 플래그
} DTC_Table_t;

DTC_Table_t DTC_Table = { 0x1234, "Brake UV Fault", 0 };

// I2C DMA 작업의 현재 상태
// 1. enum 정의 (기존 그대로)
typedef enum {
    I2C_STATE_IDLE,
    I2C_STATE_TX_BUSY,
    I2C_STATE_TX_COMPLETE,
    I2C_STATE_RX_BUSY,
    I2C_STATE_RX_COMPLETE,
    I2C_STATE_ERROR
} I2C_State_t;

// i2c_state 초기화
volatile I2C_State_t i2c_state = I2C_STATE_IDLE;  // 현재 I2C 작업 상태
static uint8_t current_reg_index = 0;    // 현재 처리 중인 레지스터 인덱스

//// 2. struct 정의 (enum을 멤버로 포함)
//typedef struct {
//    volatile I2C_State_t state;        // enum 상태
//    volatile bool dma_busy;
//    volatile bool dma_complete;
//    volatile bool dma_error;
//    mp5475_data_t current_data;
//    uint32_t error_count;
//    uint32_t success_count;
//} i2c_dma_state_t;
//
//// 3. 변수 선언
//static i2c_dma_state_t i2c_state = {
//    .state = I2C_STATE_IDLE,
//    .dma_busy = false,
//    .dma_complete = false,
//    .dma_error = false,
//    .current_data = {0},
//    .error_count = 0,
//    .success_count = 0
//};

/* ========== 함수 프로토타입 ========== */
static HAL_StatusTypeDef start_pmic_register_read(uint8_t reg_addr);
static void process_pmic_register_data(uint8_t reg_addr, uint8_t reg_data);
static void analyze_power_good_status(uint8_t pg_data);
static void analyze_buck1_uv_status(uint8_t uv_data);
static void analyze_buck2_uv_status(uint8_t uv_data);
static void analyze_buck3_ov_status(uint8_t ov_data);
static void add_dtc_code(uint16_t dtc_code);
static void print_dtc_summary(void);
static void reset_pmic_diagnosis(void);

typedef enum {
// 시스템 상태 레지스터 (읽기 전용)
MP5475_REG_SYSTEM_STATUS = 0x05,
MP5475_REG_POWER_GOOD = 0x06,
MP5475_REG_UV_OV_FAULT = 0x07,
MP5475_REG_TEMP_FAULT = 0x09,

// Buck A 제어 레지스터 (0x10 ~ 0x17)
MP5475_REG_BUCKA_CTRL1 = 0x10,
MP5475_REG_BUCKA_CTRL2 = 0x11,
MP5475_REG_BUCKA_CTRL3 = 0x12,
MP5475_REG_BUCKA_VOUT_SEL = 0x13,
MP5475_REG_BUCKA_VREF_LOW = 0x14,
MP5475_REG_BUCKA_RESERVED1 = 0x15,
MP5475_REG_BUCKA_RESERVED2 = 0x16,
MP5475_REG_BUCKA_AVP = 0x17,

// Buck B 제어 레지스터 (0x18 ~ 0x1E)
MP5475_REG_BUCKB_CTRL1 = 0x18,
MP5475_REG_BUCKB_CTRL2 = 0x19,
MP5475_REG_BUCKB_CTRL3 = 0x1A,
MP5475_REG_BUCKB_VOUT_SEL = 0x1B,
MP5475_REG_BUCKB_VREF_LOW = 0x1C,
MP5475_REG_BUCKB_RESERVED1 = 0x1D,
MP5475_REG_BUCKB_RESERVED2 = 0x1E,

// Buck C 제어 레지스터 (0x20~0x27)
MP5475_REG_BUCKC_CTRL1 = 0x20, // Dual Phase, Soft Start 설정
MP5475_REG_BUCKC_CTRL2 = 0x21, // Shutdown Delay, Transition Rate
MP5475_REG_BUCKC_CTRL3 = 0x22, // Mode, Current Limit, OVP 설정
MP5475_REG_BUCKC_VOUT_SEL = 0x23, // 출력 전압 선택
MP5475_REG_BUCKC_VREF_LOW = 0x24, // 기준 전압 하위 바이트
MP5475_REG_BUCKC_RESERVED1 = 0x25, // 예약됨
MP5475_REG_BUCKC_RESERVED2 = 0x26, // 예약됨
MP5475_REG_BUCKC_AVP = 0x27, // AVP 설정

// Buck D 제어 레지스터 (0x28~0x2E)
MP5475_REG_BUCKD_CTRL1 = 0x28, // Soft Start 설정
MP5475_REG_BUCKD_CTRL2 = 0x29, // Shutdown Delay, Transition Rate
MP5475_REG_BUCKD_CTRL3 = 0x2A, // Mode, Current Limit, OVP 설정
MP5475_REG_BUCKD_VOUT_SEL = 0x2B, // 출력 전압 선택
MP5475_REG_BUCKD_VREF_LOW = 0x2C, // 기준 전압 하위 바이트
MP5475_REG_BUCKD_RESERVED1 = 0x2D, // 예약됨
MP5475_REG_BUCKD_RESERVED2 = 0x2E, // 예약됨

// 시스템 제어 레지스터 (0x40~0x56)
MP5475_REG_SYSTEM_ENABLE = 0x40, // 시스템/Buck Enable 제어
MP5475_REG_SLAVE_ADDRESS = 0x41, // I2C Slave Address 설정
MP5475_REG_FREQUENCY = 0x42, // 스위칭 주파수 설정
MP5475_REG_SYSTEM_CONFIG = 0x43, // UVLO, Shutdown 설정
MP5475_REG_POWER_GOOD_CFG = 0x44, // Power Good 설정
MP5475_REG_FAULT_MASK = 0x45, // Fault Mask 설정

// MTP 관련 레지스터
MP5475_REG_MTP_CONFIG = 0x50, // MTP 설정 코드
MP5475_REG_MTP_REVISION = 0x51, // MTP 리비전 번호
MP5475_REG_MTP_PASSWORD = 0x52, // MTP 프로그램 패스워드
MP5475_REG_REVISION_ID = 0x54, // 리비전 ID
MP5475_REG_VENDOR_ID_BYTE0 = 0x55, // 벤더 ID 바이트 0
MP5475_REG_VENDOR_ID_BYTE1 = 0x56 // 벤더 ID 바이트 1
} mp5475_register_t;

// 읽을 레지스터 주소 배열 (순서대로 처리)
static const mp5475_register_t reg_addresses[] = {
	MP5475_REG_SYSTEM_STATUS,	// 0x05
    MP5475_REG_POWER_GOOD,      // 0x06
    MP5475_REG_UV_OV_FAULT,     // 0x07
    MP5475_REG_TEMP_FAULT       // 0x09
};
/**
 * @brief I2C 통신 처리 함수 (PMIC 상태 확인)
 * @note MP5475GU와 I2C DMA Interrupt 방식으로 통신
 */
void process_i2c_step(void){
	printf("[I2C] Processing PMIC status check ...\n");

	//todo
	// PMIC 레지스터 읽기 (0x07, 0x08, 0x09)
	// 고장감지 (UV/OV/OV 체크)
	// PG 확인 레지스터 (0x06)
	// DTC 코드 생성 (0xC001 ~ 0xC006)
	switch (i2c_state) {
	    case I2C_STATE_IDLE:
	    	//새로운 진단 시작
				printf("[I2C] Starting PMIC diagnosis sequence\n");
				reset_pmic_diagnosis();

				current_reg_index = 0;
				if (start_pmic_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
								printf("[I2C] Failed to start PMIC register read\n");
								i2c_state = I2C_STATE_ERROR;
							}
	        break;
	    case I2C_STATE_TX_BUSY:
	        // 처리
	        break;
	    case I2C_STATE_TX_COMPLETE:
	        // 처리
	        break;
	    case I2C_STATE_RX_BUSY:
	        // 처리
	        break;
	    case I2C_STATE_RX_COMPLETE:
	        // 처리
	        break;
	    case I2C_STATE_ERROR:
	        // 처리
	        break;
	}



	printf("[I2C] PMIC status: Normal! ...\n");
}


// 읽은 레지스터 데이터 저장 배열
static uint8_t pmic_status_data[4];

typedef enum {
    DTC_BRAKE_PMIC_UV     = 0xC001,  // 16비트
    DTC_BRAKE_PMIC_OV     = 0xC002,  // 16비트
    DTC_BRAKE_PMIC_OC     = 0xC003,  // 16비트
    DTC_BRAKE_PMIC_TEMP   = 0xC004,  // 16비트
    DTC_BRAKE_COMM_ERROR  = 0xC005,  // 16비트
    DTC_BRAKE_SYSTEM_FAULT = 0xC006  // 16비트
} brake_dtc_code_t;

static const DTC_Table_t dtc_master_table[] = {
    {DTC_BRAKE_PMIC_UV,     "Buck Undervoltage Fault",     0},
    {DTC_BRAKE_PMIC_OV,     "Buck Overvoltage Fault",      0},
    {DTC_BRAKE_PMIC_OC,     "Buck Overcurrent Fault",      0},
    {DTC_BRAKE_PMIC_TEMP,   "PMIC Temperature Fault",      0},
    {DTC_BRAKE_COMM_ERROR,  "I2C Communication Error",     0},
    {DTC_BRAKE_SYSTEM_FAULT,"System/Power Good Fault",     0}
};

// 활성 DTC 저장 배열
static DTC_Table_t detected_dtc_table[6];
static uint8_t dtc_count = 0;

/**
 * @brief PMIC 진단 상태 초기화 함수
 * @note 새로운 진단 사이클 시작 전 모든 변수 초기화
 */
static void reset_pmic_diagnosis(void)
{
    // 인덱스 및 카운터 초기화
    current_reg_index = 0;
    dtc_count = 0;

    // 데이터 배열 초기화
    for (uint8_t i = 0; i < sizeof(pmic_status_data); i++) {
    	pmic_status_data[i] = 0;
    }

    // DTC 테이블 초기화 (구조체 배열)
        for (uint8_t i = 0; i < sizeof(detected_dtc_table) / sizeof(detected_dtc_table[0]); i++) {
            detected_dtc_table[i].DTC_Code = 0;
            detected_dtc_table[i].Description[0] = '\0';  // 문자열 초기화
            detected_dtc_table[i].active = 0;
        }

    // 작업 변수 초기화
    pmic_reg_address = 0;
    pmic_reg_data = 0;

    printf("[DIAG] PMIC diagnosis state reset completed\n");
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
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();  // HAL 라이브러리 초기화
    SystemClock_Config();  // 시스템 클럭 설정

    //모든 시스템 IC init
    system_init();

	printf("\n=== Main Loop 시작 ===\n");
	printf("동작 순서 : I2C -> SPI -> CAN -> UART \n\n");

	uint32_t loop_count = 0;

	while(1)
	{
		printf("--- Loop #%d --- \n", ++loop_count);

		// I2C -> SPI -> CAN -> UART 순서
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
