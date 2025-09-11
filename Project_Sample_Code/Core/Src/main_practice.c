#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h> 
#include "stm32f4xx_hal.h"
#include "mp5457gu.h"

// ========== PMIC 설정 상수 ==========
#define MP5475_I2C_ADDRESS          0x60
#define MP5475_REG_READ_SIZE        1

// 외부 변수 참조
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern SPI_HandleTypeDef hspi1, hspi2;
extern UART_HandleTypeDef huart4;

// ========== I2C 상태 머신 ==========
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
	MP5475_REG_OC_FAULT = 0x08,
	MP5475_REG_TEMP_FAULT = 0x09
} mp5475_register_t;

 // ========== DTC 코드 정의 ==========
typedef enum {
    DTC_BRAKE_PMIC_UV     = 0xC001,  // 16비트
    DTC_BRAKE_PMIC_OV     = 0xC002,  // 16비트
    DTC_BRAKE_PMIC_OC     = 0xC003,  // 16비트
    DTC_BRAKE_PMIC_TEMP   = 0xC004,  // 16비트
    DTC_BRAKE_COMM_ERROR  = 0xC005,  // 16비트
    DTC_BRAKE_SYSTEM_FAULT = 0xC006  // 16비트
} brake_dtc_code_t;

// --- DTC 데이터 구조 정의 ---
#pragma pack(push, 1)  // 기존:56 Byte × 6 = 336 최적화: 53 Byte × 6 = 318 절약: 18 Byte (5% 절약) 
typedef struct {
	uint16_t DTC_Code;              // 고장 코드 (예: C1234)
	char Description[50];           // 설명 문자열
	uint8_t active;                 // 활성화 상태 플래그
  } DTC_Table_t;
#pragma pack(pop) 

DTC_Table_t DTC_Table = { 0x1234, "Brake UV Fault", 0 };

 // ========== I2C 상태 관리 변수 ==========
volatile i2c_state_t i2c_state = I2C_STATE_IDLE;  // 현재 I2C 작업 상태
uint8_t current_reg_index = 0;                // 레지스터 배열 인덱스 (0부터 시작)
uint8_t pmic_reg_address = 0;                 // 현재 처리 중인 레지스터 주소
uint8_t pmic_reg_data = 0;                    // DMA로 읽은 데이터 저장용
uint8_t i2c_rx_buffer[1] = {0};               // DMA 수신 버퍼 (1바이트씩 읽으므로 크기 1)

// 읽을 레지스터 주소 배열 (순서대로 처리)
static const mp5475_register_t reg_addresses[] = {
	  MP5475_REG_SYSTEM_STATUS,	// 0x05
    MP5475_REG_POWER_GOOD,      // 0x06
    MP5475_REG_UV_OV_FAULT,     // 0x07
    MP5475_REG_TEMP_FAULT       // 0x09
};

// 읽은 레지스터 데이터 저장 배열
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

/* ========== 함수 프로토타입 ========== */
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

// PMIC IRQ 발생 시 호출되는 콜백
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_3) {  // PMIC IRQ 핀
        printf("[IRQ] PMIC fault detected, starting diagnosis\n");
        
        // I2C 상태가 유휴일 때만 진단 시작
        if (i2c_state == I2C_STATE_IDLE) {
            reset_pmic_diagnosis();
            current_reg_index = 0;
            if (start_pmic_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
                i2c_state = I2C_STATE_ERROR;
            }
        } else{
            printf("[WARNING] I2C bush, IRQ ignored\n");
        }
    }
}

 // ========== I2C DMA 레지스터 읽기 시작 함수 ==========
  /**
  * @brief PMIC 레지스터 DMA 읽기 시작
  * @param reg_addr 읽을 레지스터 주소
  * @retval HAL_StatusTypeDef
  */
  static HAL_StatusTypeDef start_pmic_register_read(uint8_t reg_addr)
  { // 현재 레지스터 주소 저장
    pmic_reg_address = reg_addr;
    // 상태를 BUSY 대기로 변경
    i2c_state = I2C_STATE_RX_BUSY;
    // DMA 버퍼 초기화
    i2c_rx_buffer[0] = 0;
    printf("[I2C] Starting DMA read - Register: 0x%02X\n", reg_addr);
    // HAL_I2C_Mem_Read_DMA 호출
    /*HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c,
                  uint16_t DevAddress,
                  uint16_t MemAddress,
                  uint16_t MemAddSize,
                  uint8_t *pData,
                  uint16_t Size); */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(&hi2c1,
                                                    (MP5475_I2C_ADDRESS << 1),
                                                    reg_addr,
                                                    I2C_MEMADD_SIZE_8BIT,
                                                    i2c_rx_buffer,
                                                    MP5475_REG_READ_SIZE);
    // READ 결과 확인
    if (status != HAL_OK){
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
{	//I2C1에서 발생한 콜백인지 확인
	if(hi2c -> Instance == I2C1){
		//DMA 버퍼에서 데이터 복사
		pmic_reg_data = i2c_rx_buffer[0];
		//상태를 수신완료로 변경
		i2c_state = I2C_STATE_RX_COMPLETE;

		printf("[I2C] DMA RX Complete - Reg: 0x%02X, data: 0x%02X\n", pmic_reg_address,pmic_reg_data);
		
		//받은 데이터 처리
		process_pmic_register_data(pmic_reg_address, pmic_reg_data);//pmic_reg_address는 전역변수로 선언했고 start_pmic_register_read함수 첫줄에서 값 설정 완료
		
    // 다음 레지스터로 이동
    current_reg_index++; // 0x06 -> 0x07 -> 0x08 -> 0x09
    
    if(current_reg_index < 4) {
      if(start_pmic_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
          i2c_state = I2C_STATE_ERROR;
      }
    } else {
      // 모든 레지스터 읽기 완료
      printf("[I2C] All registers read, diagnosis complete\n");
      print_dtc_summary();
      i2c_state = I2C_STATE_IDLE;
    }
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
		// 에러 상태로 변경
		i2c_state = I2C_STATE_ERROR;
		// 통신 에러 DTC 추가 
		add_dtc_code(DTC_BRAKE_COMM_ERROR);
	}
}
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
static void process_pmic_register_data(uint8_t reg_addr, uint8_t reg_data)
{
	printf("[DIAG] Processing register 0x%02X with data 0x%02X\n",reg_addr,reg_data);
	// 읽은 데이터를 배열에 저장
	pmic_status_data[current_reg_index] = reg_data;
	// 레지스터별 데이터 분석
	switch (reg_addr) {
		case MP5475_REG_POWER_GOOD: // 0x06
			// PG_FILT 비트만 체크 (비트 7-4: 1111XXXX)
			if((reg_data & 0xF0) != 0xF0){ //전체 비트와 0xF0=11110000과 &연산 결과가 0xF0와 다를때
				printf("[FAULT] Power Good FAIL - Data: 0x%02X\n", reg_data);
        		add_dtc_code(DTC_BRAKE_SYSTEM_FAULT);
			}
			break;
		case MP5475_REG_UV_OV_FAULT: // 0x07
			// 비트 7: 시스템 에러 UV
			if (reg_data & 0x80 || reg_data & 0x40 || reg_data & 0x20 || reg_data & 0x10)
			{	// 7654비트 1일때
				printf("[FAULT] PMIC error UV detected- Data: 0x%02X\n", reg_data);
				add_dtc_code(DTC_BRAKE_PMIC_UV);
			}else if (reg_data & 0x08 || reg_data & 0x04 || reg_data & 0x02 || reg_data & 0x01)
			{	// 3210비트 1일때  OV
				printf("[FAULT] PMIC error OV detected- Data: 0x%02X\n", reg_data);
				add_dtc_code(DTC_BRAKE_PMIC_OV);
			}
			break;
		case MP5475_REG_OC_FAULT: //0x08
			if(reg_data & 0x80 || reg_data & 0x40 || reg_data & 0x20 || reg_data & 0x10){
				printf("[FAULT] PMIC error OC detected- Data: 0x%02X\n", reg_data);
				add_dtc_code(DTC_BRAKE_PMIC_OC);
			}
			if (reg_data & 0x08 || reg_data & 0x04 || reg_data & 0x02 || reg_data & 0x01)
			{	printf("[WARNING] PMIC OC WARNING - Data: 0x%02X\n", reg_data);
			}
			break;
		case MP5475_REG_TEMP_FAULT: //0x09
			if(reg_data & 0x01){
				printf("[FAULT] PMIC High Temperature Shutdown - Data: 0x%02X\n", reg_data);
				add_dtc_code(DTC_BRAKE_PMIC_TEMP);
			}
			if (reg_data & 0x02) {
				printf("[WARNING] PMIC High Temperature Warning - Data: 0x%02X\n", reg_data);
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
	// 중복 DTC 확인 : 같은 고장을 여러 번 저장하는 것을 방지
	for (uint8_t i = 0; i < dtc_count; i++ ){
		if(detected_dtc_table[i].DTC_Code == dtc_code){
			return;
		}
	}

	// DTC 테이블 공간 확인 
	if (dtc_count >= 6){
		printf("[DTC] ERROR: DTC table full\n");
		return;
	}

	// 마스터 테이블에서 DTC 정보 찾기 
	for (uint8_t i = 0 ; i < 6; i++){
		if (dtc_master_table[i].DTC_Code == dtc_code){// 마스터 테이블에 갑지한 DTC코드롸 일치하는것이 있으면
			//detected_dtc_code 테이블에 추가 
			detected_dtc_table[dtc_count].DTC_Code = dtc_code;
			//마스터 Description을 detected_dtc_table의 Descripttion에 추가
			strcpy(detected_dtc_table[dtc_count].Description, dtc_master_table[i].Description) ;
			// DTC 활성화 상태로 업뎃
			detected_dtc_table[dtc_count].active = 1;
			printf("[DTC] Added: 0x%04X - %s\n", dtc_code, dtc_master_table[i].Description);
			dtc_count++;
			return;
		}
	}
}

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
	 pmic_reg_data = 0;  // Union 초기화
 
	 printf("[DIAG] PMIC diagnosis state reset completed\n");
 }

 // ========== DTC 요약 출력 함수 ==========
 /**
  * @brief 감지된 DTC 요약 출력
  */
static void print_dtc_summary(void){

  printf("\n========= DTC SUMMARY ===========");
  printf("Total detected DTCs: %d\n", dtc_count);
  
  if (dtc_count == 0){
      printf("No faults detected - System OK\n");
  } else {
      for (uint8_t i = 0; i < dtc_count; i++) {
          printf("DTC[%d]: 0x%04X - %s\n",
                i + 1,
              detected_dtc_table[i].DTC_Code,
              detected_dtc_table[i].Description);
      }
  }
  printf("===================================\n\n");
}

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
// 				reset_pmic_diagnosis();

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
//             break;
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);

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

  // NVIC 인터럽트 활성화
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

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