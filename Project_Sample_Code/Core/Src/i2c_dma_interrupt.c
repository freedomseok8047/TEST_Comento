/**
 * I2C DMA Interrupt 기반 시리얼 통신 코드
 * STM32 HAL 라이브러리를 기반으로 작성됨
 * 
 * 주요 기능:
 * - I2C를 통한 DMA 기반 데이터 송수신
 * - 인터럽트를 통한 비동기 통신 처리
 * - 송신(TX) 및 수신(RX) 완료 콜백
 */

#include "stm32f4xx_hal.h"  // STM32 HAL 헤더 (사용하는 MCU에 맞게 수정)
#include <string.h>
#include <stdio.h>

/* ========== 전역 변수 정의 ========== */

// I2C 핸들러 구조체 - I2C 통신을 위한 설정을 저장
I2C_HandleTypeDef hi2c1;

// DMA 핸들러 구조체들 - 송신과 수신용 DMA 채널을 각각 관리
DMA_HandleTypeDef hdma_i2c1_tx;  // 송신용 DMA 핸들러
DMA_HandleTypeDef hdma_i2c1_rx;  // 수신용 DMA 핸들러

// 데이터 버퍼들
uint8_t tx_buffer[256];     // 송신할 데이터를 저장하는 버퍼
uint8_t rx_buffer[256];     // 수신된 데이터를 저장하는 버퍼

// 상태 플래그들
volatile uint8_t tx_complete = 0;   // 송신 완료 플래그 (1: 완료, 0: 진행중)
volatile uint8_t rx_complete = 0;   // 수신 완료 플래그 (1: 완료, 0: 진행중)
volatile uint8_t i2c_error = 0;     // 에러 발생 플래그 (1: 에러, 0: 정상)

// I2C 슬레이브 장치 주소 (7-bit 주소를 8-bit로 변환)
#define SLAVE_ADDRESS   (0x50 << 1)  // 예시: EEPROM 주소 0x50을 왼쪽으로 1비트 시프트

/* ========== 함수 프로토타입 선언 ========== */
void SystemClock_Config(void);
void I2C1_Init(void);
void DMA_Init(void);
void GPIO_Init(void);
void Error_Handler(void);

/* ========== 메인 함수 ========== */
int main(void)
{
    /* STM32 HAL 라이브러리 초기화 */
    HAL_Init();
    
    /* 시스템 클록 설정 */
    SystemClock_Config();
    
    /* GPIO 핀 설정 (I2C용 핀 설정) */
    GPIO_Init();
    
    /* DMA 초기화 (I2C와 연결하기 전에 먼저 설정) */
    DMA_Init();
    
    /* I2C 초기화 */
    I2C1_Init();
    
    /* 송신할 데이터 준비 */
    const char* test_data = "Hello I2C DMA!";
    strcpy((char*)tx_buffer, test_data);
    uint16_t data_length = strlen(test_data);
    
    printf("I2C DMA 통신 시작\n");
    
    while (1)
    {
        /* ========== 데이터 송신 ========== */
        
        // 송신 플래그 초기화
        tx_complete = 0; // 송신 완료 플래그 (1: 완료, 0: 진행중)
        i2c_error = 0;   // 에러 발생 플래그 (1: 에러, 0: 정상)
        
        printf("데이터 송신 시작: %s\n", tx_buffer);
        
        // I2C DMA를 통한 데이터 송신 시작
        // HAL_I2C_Master_Transmit_DMA: DMA를 사용한 마스터 모드 송신
        if (HAL_I2C_Master_Transmit_DMA(&hi2c1, SLAVE_ADDRESS, tx_buffer, data_length) != HAL_OK) //매개변수: I2C핸들러, 슬레이브주소, 보낼데이터, 데이터길이
        {
            printf("송신 시작 실패\n");
            Error_Handler();
        }
        
        // 송신 완료를 기다림 (논블로킹 방식)
        // 실제 애플리케이션에서는 타임아웃을 설정하는 것이 좋음
        uint32_t timeout = HAL_GetTick() + 5000;  //시스템이 시작된 후 경과 시간(밀리초) + 5초 타임아웃
        while (!tx_complete && !i2c_error && HAL_GetTick() < timeout)
        	/*!tx_complete: 송신이 완료되지 않았고
        	!i2c_error: 에러도 없고
        	HAL_GetTick() < timeout: 아직 타임아웃 시간이 안됐으면 계속 대기*/
        {
            // 다른 작업을 수행할 수 있음
            HAL_Delay(1);
        }
        
        if (i2c_error) // 에러 발생 플래그 (1: 에러, 0: 정상)
        {
            printf("송신 중 에러 발생\n");
        }
        else if (tx_complete)
        {
            printf("송신 완료\n");
        }
        else
        {
            printf("송신 타임아웃\n");
        }
        
        HAL_Delay(1000);  // 1초 대기
        
        /* ========== 데이터 수신 ========== */
        
        // 수신 플래그 초기화
        rx_complete = 0;
        i2c_error = 0;
        memset(rx_buffer, 0, sizeof(rx_buffer));  // 수신 버퍼 초기화
        
        printf("데이터 수신 시작\n");
        
        // I2C DMA를 통한 데이터 수신 시작
        // HAL_I2C_Master_Receive_DMA: DMA를 사용한 마스터 모드 수신
        if (HAL_I2C_Master_Receive_DMA(&hi2c1, SLAVE_ADDRESS, rx_buffer, data_length) != HAL_OK)
        {
            printf("수신 시작 실패\n");
            Error_Handler();
        }
        
        // 수신 완료를 기다림
        timeout = HAL_GetTick() + 5000;  // 5초 타임아웃
        while (!rx_complete && !i2c_error && HAL_GetTick() < timeout)
        {
            // 다른 작업을 수행할 수 있음
            HAL_Delay(1);
        }
        
        if (i2c_error)
        {
            printf("수신 중 에러 발생\n");
        }
        else if (rx_complete)
        {
            printf("수신 완료: %s\n", rx_buffer);
        }
        else
        {
            printf("수신 타임아웃\n");
        }
        
        HAL_Delay(2000);  // 2초 대기 후 다시 반복
    }
}

/* ========== I2C 초기화 함수 ========== */
void I2C1_Init(void)
{
    /* I2C1 기본 설정 */
    hi2c1.Instance = I2C1;                    // I2C1 사용
    hi2c1.Init.ClockSpeed = 100000;           // 클록 속도: 100kHz (Standard Mode)
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;   // 듀티 사이클: 2 (50%)
    hi2c1.Init.OwnAddress1 = 0;               // 자신의 주소 (마스터 모드에서는 사용안함)
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  // 7비트 주소 모드
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // 듀얼 주소 모드 비활성화
    hi2c1.Init.OwnAddress2 = 0;               // 두 번째 주소 (사용안함)
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // 일반 호출 모드 비활성화
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;     // 클록 스트레칭 활성화
    
    /* I2C 초기화 실행 */
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========== DMA 초기화 함수 ========== */
void DMA_Init(void)
{
    /* DMA 컨트롤러 클록 활성화 */
    __HAL_RCC_DMA1_CLK_ENABLE();
    
    /* ========== I2C TX용 DMA 설정 ========== */
    hdma_i2c1_tx.Instance = DMA1_Stream6;                    // DMA1 Stream6 사용 (I2C1_TX)
    hdma_i2c1_tx.Init.Channel = DMA_CHANNEL_1;               // 채널 1
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;      // 메모리 -> 주변장치
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;         // 주변장치 주소 증가 비활성화
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;             // 메모리 주소 증가 활성화
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  // 주변장치 데이터 크기: 바이트
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;     // 메모리 데이터 크기: 바이트
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;                     // 노멀 모드 (순환 모드 아님)
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;           // 우선순위: 낮음
    hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;       // FIFO 모드 비활성화
    
    if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* DMA를 I2C TX와 연결 */
    __HAL_LINKDMA(&hi2c1, hdmatx, hdma_i2c1_tx);
    
    /* ========== I2C RX용 DMA 설정 ========== */
    hdma_i2c1_rx.Instance = DMA1_Stream5;                    // DMA1 Stream5 사용 (I2C1_RX)
    hdma_i2c1_rx.Init.Channel = DMA_CHANNEL_1;               // 채널 1
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;      // 주변장치 -> 메모리
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;         // 주변장치 주소 증가 비활성화
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;             // 메모리 주소 증가 활성화
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  // 주변장치 데이터 크기: 바이트
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;     // 메모리 데이터 크기: 바이트
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;                     // 노멀 모드
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_HIGH;          // 우선순위: 높음
    hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;       // FIFO 모드 비활성화
    
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* DMA를 I2C RX와 연결 */
    __HAL_LINKDMA(&hi2c1, hdmarx, hdma_i2c1_rx);
    
    /* ========== DMA 인터럽트 우선순위 설정 ========== */
    // TX DMA 인터럽트 설정
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    
    // RX DMA 인터럽트 설정
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    
    /* ========== I2C 이벤트 및 에러 인터럽트 설정 ========== */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

/* ========== GPIO 초기화 함수 ========== */
void GPIO_Init(void) //GPIO (General Purpose Input/Output) 범용 입출력 핀: 마이크로컨트롤러의 다리(핀)들을 설정
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* I2C1 GPIO 클록 활성화 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /**
     * I2C1 GPIO 설정
     * PB8 ------> I2C1_SCL (시리얼 클록)
     * PB9 ------> I2C1_SDA (시리얼 데이터)
     */
    /*TODO 오픈드레인과 풀업 저항의 조합이 I2C 통신의 핵심
    	이 설정이 없으면 I2C 통신이 제대로 되지 않음 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;        // PB8, PB9 핀 선택
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;               // 대체 기능, 오픈 드레인 모드
    GPIO_InitStruct.Pull = GPIO_PULLUP;                   // 풀업 저항 활성화
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;    // 최고 속도
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;            // I2C1 대체 기능 선택
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* ========== 시스템 클록 설정 함수 ========== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** 전압 스케일링 설정 */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** HSI 오실레이터 초기화 및 PLL 설정 */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;   // HSI/8 = 2MHz
    RCC_OscInitStruct.PLL.PLLN = 180; // 2MHz * 180 = 360MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // 360MHz/2 = 180MHz
    RCC_OscInitStruct.PLL.PLLQ = 4;   // USB 클록용

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** 시스템 클록 설정 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   // HCLK = 180MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;    // PCLK1 = 45MHz (I2C 클록)
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;    // PCLK2 = 90MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========== 인터럽트 핸들러 함수들 ========== */

/**
 * DMA1 Stream6 인터럽트 핸들러 (I2C1 TX)
 * DMA로 데이터 송신이 완료되면 이 함수가 호출됨
 */
void DMA1_Stream6_IRQHandler(void) // 송신 완료 시 호출
{
    HAL_DMA_IRQHandler(&hdma_i2c1_tx);
}

/**
 * DMA1 Stream5 인터럽트 핸들러 (I2C1 RX)
 * DMA로 데이터 수신이 완료되면 이 함수가 호출됨
 */
void DMA1_Stream5_IRQHandler(void) // 수신 완료 시 호출
{
    HAL_DMA_IRQHandler(&hdma_i2c1_rx);
}

/**
 * I2C1 이벤트 인터럽트 핸들러
 * I2C 통신 중 정상적인 이벤트가 발생하면 호출됨
 */
void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

/**
 * I2C1 에러 인터럽트 핸들러
 * I2C 통신 중 에러가 발생하면 호출됨
 */
void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c1);
}

/* ========== 콜백 함수들 ========== */

/**
 * I2C 마스터 송신 완료 콜백 함수
 * HAL_I2C_Master_Transmit_DMA() 완료 시 자동 호출됨
 * 
 * @param hi2c: I2C 핸들러 포인터
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) //현재 I2C 핸들이 I2C1 하드웨어를 사용하고 있는가?
    {
        tx_complete = 1;  // 송신 완료 플래그 설정
        printf("송신 완료 콜백 호출됨\n");
    }
}

/**
 * I2C 마스터 수신 완료 콜백 함수
 * HAL_I2C_Master_Receive_DMA() 완료 시 자동 호출됨
 * 
 * @param hi2c: I2C 핸들러 포인터
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) //현재 I2C 핸들이 I2C1 하드웨어를 사용하고 있는가?
    {
        rx_complete = 1;  // 수신 완료 플래그 설정
        printf("수신 완료 콜백 호출됨\n");
    }
}

/**
 * I2C 에러 콜백 함수
 * I2C 통신 중 에러 발생 시 자동 호출됨
 * 
 * @param hi2c: I2C 핸들러 포인터
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        i2c_error = 1;  // 에러 플래그 설정
        
        // 에러 종류 확인 및 출력
        uint32_t error = HAL_I2C_GetError(hi2c);
        
        printf("I2C 에러 발생: ");
        if (error & HAL_I2C_ERROR_BERR)    printf("Bus Error ");
        if (error & HAL_I2C_ERROR_ARLO)    printf("Arbitration Lost ");
        if (error & HAL_I2C_ERROR_AF)      printf("Acknowledge Failure ");
        if (error & HAL_I2C_ERROR_OVR)     printf("Overrun/Underrun ");
        if (error & HAL_I2C_ERROR_DMA)     printf("DMA Error ");
        if (error & HAL_I2C_ERROR_TIMEOUT) printf("Timeout Error ");
        printf("\n");
    }
}

/**
 * I2C 중단 완료 콜백 함수 (선택사항)
 * 통신이 중단되었을 때 호출됨
 */
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        printf("I2C 통신 중단됨\n");
    }
}

/* ========== 에러 핸들러 ========== */
void Error_Handler(void)
{
    /* 에러 발생 시 LED를 켜거나 다른 표시를 할 수 있음 */
    printf("에러 발생! 시스템 정지\n");
    
    /* 무한 루프로 시스템 정지 */
    while (1)
    {
        // 디버깅을 위해 여기서 브레이크포인트를 걸 수 있음
    }
}

/* ========== HAL MSP (MCU Support Package) 함수들 ========== */

/**
 * I2C MSP 초기화 함수
 * HAL_I2C_Init()에서 자동으로 호출됨
 * 클록 활성화 및 GPIO 설정이 여기서 이루어짐
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        /* I2C1 클록 활성화 */
        __HAL_RCC_I2C1_CLK_ENABLE();
        
        /* 이미 GPIO_Init()에서 설정했으므로 추가 설정 없음 */
    }
}

/**
 * I2C MSP 해제 함수
 * HAL_I2C_DeInit()에서 자동으로 호출됨
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        /* I2C1 클록 비활성화 */
        __HAL_RCC_I2C1_CLK_DISABLE();
        
        /* GPIO 핀 해제 */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
        
        /* DMA 해제 */
        if (hi2c->hdmatx != NULL)
        {
            HAL_DMA_DeInit(hi2c->hdmatx);
        }
        if (hi2c->hdmarx != NULL)
        {
            HAL_DMA_DeInit(hi2c->hdmarx);
        }
        
        /* 인터럽트 비활성화 */
        HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
    }
}

/**
 * 프로그램 사용법 및 주의사항:
 * 
 * 1. 하드웨어 연결:
 *    - PB8 (SCL)과 PB9 (SDA)를 I2C 슬레이브 디바이스에 연결
 *    - 풀업 저항 (4.7kΩ) 연결 (보통 슬레이브 보드에 내장됨)
 * 
 * 2. 슬레이브 주소 설정:
 *    - SLAVE_ADDRESS 매크로를 실제 디바이스 주소로 변경
 * 
 * 3. 컴파일 환경:
 *    - STM32CubeIDE 또는 Keil MDK 사용 권장
 *    - HAL 라이브러리 포함 필요
 * 
 * 4. 디버깅:
 *    - printf 출력을 위해 UART나 SWO 설정 필요
 *    - 에러 발생 시 Error_Handler()에서 브레이크포인트 설정
 * 
 * 5. 성능 최적화:
 *    - DMA 사용으로 CPU 부하 최소화
 *    - 인터럽트 기반으로 논블로킹 동작
 *    - 타임아웃 설정으로 무한 대기 방지
 */
