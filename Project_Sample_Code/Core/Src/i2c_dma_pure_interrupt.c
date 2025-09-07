/**
 * 순수 인터럽트 기반 I2C DMA 통신 코드
 * STM32 HAL 라이브러리를 기반으로 작성됨
 * 
 * 동작 원리:
 * 1. CPU: DMA 설정 후 → 다른 작업 수행
 * 2. DMA Controller: 독립적으로 메모리 ↔ 주변장치 간 데이터 전송
 * 3. 전송 완료 시: DMA Controller가 CPU에 IRQ 신호 전송
 * 4. Interrupt Controller: IRQ 받아서 해당 ISR 실행
 * 5. ISR: DMA 완료 처리 후 콜백 호출
 * 
 * 특징:
 * - 폴링(polling) 방식을 완전히 제거
 * - 순수 인터럽트 및 콜백 기반 동작
 * - 상태 머신을 통한 작업 흐름 관리
 * - CPU 자원을 최대한 효율적으로 사용
 */

#include "stm32f4xx_hal.h"  // STM32 HAL 헤더 파일 포함 (MCU 종류에 맞게 수정 필요)
#include <string.h>         // 문자열 함수 사용을 위한 헤더
#include <stdio.h>          // printf 함수 사용을 위한 헤더

/* ========== 상수 및 매크로 정의 ========== */

// I2C 슬레이브 디바이스의 7비트 주소를 8비트로 변환 (LSB는 R/W 비트)
#define SLAVE_ADDRESS   (0x50 << 1)  // EEPROM 주소 0x50을 왼쪽으로 1비트 시프트
#define BUFFER_SIZE     256          // 송수신 버퍼의 최대 크기 (바이트)
#define TIMEOUT_MS      5000         // 타임아웃 시간 (밀리초)

/* ========== 상태 머신 열거형 정의 ========== */

// I2C DMA 작업의 현재 상태를 나타내는 열거형
typedef enum {
    I2C_STATE_IDLE,          // 유휴 상태 - 아무 작업도 하지 않음
    I2C_STATE_TX_BUSY,       // 송신 진행 중 - DMA가 데이터를 전송하고 있음
    I2C_STATE_TX_COMPLETE,   // 송신 완료됨 - DMA 전송이 성공적으로 끝남
    I2C_STATE_RX_BUSY,       // 수신 진행 중 - DMA가 데이터를 받고 있음
    I2C_STATE_RX_COMPLETE,   // 수신 완료됨 - DMA 수신이 성공적으로 끝남
    I2C_STATE_ERROR          // 에러 발생 - 전송 또는 수신 중 문제 발생
} I2C_State_t;

/* ========== 전역 변수 선언 ========== */

// I2C 통신을 위한 핸들러 구조체 - STM32 HAL에서 I2C 설정을 관리
I2C_HandleTypeDef hi2c1;

// DMA 전송을 위한 핸들러 구조체들 - 각각 송신과 수신을 담당
DMA_HandleTypeDef hdma_i2c1_tx;  // 송신용 DMA 스트림 핸들러
DMA_HandleTypeDef hdma_i2c1_rx;  // 수신용 DMA 스트림 핸들러

// 데이터 저장을 위한 버퍼들 - 메모리에서 실제 데이터가 저장되는 공간
uint8_t tx_buffer[BUFFER_SIZE];     // 송신할 데이터를 저장하는 배열
uint8_t rx_buffer[BUFFER_SIZE];     // 수신된 데이터를 저장하는 배열

// 상태 관리 변수들 - volatile로 선언하여 인터럽트와 메인 함수 간 동기화
volatile I2C_State_t i2c_state = I2C_STATE_IDLE;  // 현재 I2C 작업 상태
volatile uint32_t error_code = 0;                  // 발생한 에러 코드 저장
volatile uint32_t timeout_counter = 0;             // 타임아웃 카운터

/* ========== 함수 프로토타입 선언 ========== */

// 시스템 초기화 관련 함수들
void SystemClock_Config(void);    // 시스템 클록 설정
void I2C1_Init(void);             // I2C1 초기화
void DMA_Init(void);              // DMA 초기화
void GPIO_Init(void);             // GPIO 핀 초기화
void Timer_Init(void);            // 타이머 초기화 (타임아웃 체크용)

// 에러 처리 함수
void Error_Handler(void);         // 에러 발생 시 호출되는 함수

// I2C 작업 시작 함수들
HAL_StatusTypeDef Start_I2C_Transmit(uint8_t* data, uint16_t size);  // DMA 송신 시작
HAL_StatusTypeDef Start_I2C_Receive(uint16_t size);                  // DMA 수신 시작

// 상태 관리 함수들
void Process_I2C_State(void);     // 상태 머신 처리
void Reset_I2C_State(void);       // 상태 초기화

/* ========== 메인 함수 ========== */

int main(void)
{
    // STM32 HAL 라이브러리 초기화 - 모든 HAL 기능을 사용하기 위해 필요
    HAL_Init();
    
    // 시스템 클록을 설정 - MCU의 동작 주파수를 설정
    SystemClock_Config();
    
    // GPIO 핀을 초기화 - I2C 통신용 핀들을 설정
    GPIO_Init();
    
    // DMA 컨트롤러를 초기화 - 메모리와 주변장치 간 직접 전송을 위해
    DMA_Init();
    
    // I2C 인터페이스를 초기화 - 통신 프로토콜 설정
    I2C1_Init();
    
    // 타이머를 초기화 - 타임아웃 체크를 위한 주기적 인터럽트 생성
    Timer_Init();
    
    // 테스트용 송신 데이터를 준비
    const char* test_message = "Hello Pure Interrupt I2C DMA!";  // 전송할 메시지
    strcpy((char*)tx_buffer, test_message);                      // 메시지를 송신 버퍼에 복사
    uint16_t message_length = strlen(test_message);              // 메시지의 길이 계산
    
    printf("순수 인터럽트 기반 I2C DMA 통신 시작\n");
    printf("전송할 메시지: %s (길이: %d 바이트)\n", test_message, message_length);
    
    // 첫 번째 DMA 송신을 시작 - 이후 모든 처리는 인터럽트에서 진행됨
    if (Start_I2C_Transmit(tx_buffer, message_length) != HAL_OK)
    {
        printf("초기 송신 시작 실패\n");
        Error_Handler();  // 시작에 실패하면 에러 핸들러 호출
    }
    
    // 메인 루프 - CPU는 여기서 다른 작업을 수행할 수 있음
    while (1)
    {
        // 상태 머신을 처리 - 인터럽트에서 변경된 상태에 따라 다음 작업 결정
        Process_I2C_State();
        
        // CPU가 다른 중요한 작업을 수행할 수 있는 공간
        // 예: 센서 데이터 처리, 사용자 인터페이스 업데이트, 알고리즘 실행 등
        HAL_Delay(10);  // 10ms 대기 (실제로는 다른 유용한 작업으로 대체)
        
        // 필요에 따라 다른 시스템 작업들을 여기에 추가 가능
        // 예: 시스템 모니터링, 통신 프로토콜 처리, 데이터 로깅 등
    }
}

/* ========== I2C 작업 시작 함수들 ========== */

/**
 * I2C DMA 송신을 시작하는 함수
 * @param data: 송신할 데이터가 저장된 버퍼의 포인터
 * @param size: 송신할 데이터의 크기 (바이트 단위)
 * @return HAL_StatusTypeDef: HAL_OK(성공) 또는 에러 코드
 */
HAL_StatusTypeDef Start_I2C_Transmit(uint8_t* data, uint16_t size)
{
    // 현재 I2C가 유휴 상태가 아니면 송신을 시작할 수 없음
    if (i2c_state != I2C_STATE_IDLE)
    {
        printf("I2C가 사용 중입니다. 현재 상태: %d\n", i2c_state);
        return HAL_BUSY;  // 사용 중 상태 반환
    }
    
    // 상태를 송신 진행 중으로 변경
    i2c_state = I2C_STATE_TX_BUSY;
    
    // 에러 코드와 타임아웃 카운터 초기화
    error_code = 0;
    timeout_counter = 0;
    
    printf("DMA 송신 시작: %d 바이트\n", size);
    
    // HAL 함수를 호출하여 실제 DMA 송신 시작
    // 이 함수는 DMA를 설정하고 즉시 반환됨 (논블로킹)
    // 완료되면 HAL_I2C_MasterTxCpltCallback이 자동 호출됨
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_DMA(&hi2c1, SLAVE_ADDRESS, data, size);
    
    if (status != HAL_OK)
    {
        // DMA 시작에 실패하면 상태를 에러로 변경
        i2c_state = I2C_STATE_ERROR;
        error_code = status;
        printf("DMA 송신 시작 실패: %d\n", status);
    }
    
    return status;
}

/**
 * I2C DMA 수신을 시작하는 함수
 * @param size: 수신할 데이터의 크기 (바이트 단위)
 * @return HAL_StatusTypeDef: HAL_OK(성공) 또는 에러 코드
 */
HAL_StatusTypeDef Start_I2C_Receive(uint16_t size)
{
    // 현재 I2C가 유휴 상태가 아니면 수신을 시작할 수 없음
    if (i2c_state != I2C_STATE_IDLE)
    {
        printf("I2C가 사용 중입니다. 현재 상태: %d\n", i2c_state);
        return HAL_BUSY;  // 사용 중 상태 반환
    }
    
    // 수신 버퍼를 0으로 초기화 - 이전 데이터의 영향을 방지
    memset((void*)rx_buffer, 0, BUFFER_SIZE);
    
    // 상태를 수신 진행 중으로 변경
    i2c_state = I2C_STATE_RX_BUSY;
    
    // 에러 코드와 타임아웃 카운터 초기화
    error_code = 0;
    timeout_counter = 0;
    
    printf("DMA 수신 시작: %d 바이트\n", size);
    
    // HAL 함수를 호출하여 실제 DMA 수신 시작
    // 이 함수는 DMA를 설정하고 즉시 반환됨 (논블로킹)
    // 완료되면 HAL_I2C_MasterRxCpltCallback이 자동 호출됨
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive_DMA(&hi2c1, SLAVE_ADDRESS, (uint8_t*)rx_buffer, size);
    
    if (status != HAL_OK)
    {
        // DMA 시작에 실패하면 상태를 에러로 변경
        i2c_state = I2C_STATE_ERROR;
        error_code = status;
        printf("DMA 수신 시작 실패: %d\n", status);
    }
    
    return status;
}

/* ========== 상태 머신 처리 함수 ========== */

/**
 * I2C 상태 머신을 처리하는 함수
 * 현재 상태에 따라 다음에 수행할 작업을 결정
 * 메인 루프에서 주기적으로 호출됨
 */
void Process_I2C_State(void)
{
    // 현재 상태에 따른 분기 처리
    switch (i2c_state)
    {
        case I2C_STATE_IDLE:
            // 유휴 상태 - 아무 작업도 수행하지 않음
            // CPU는 이 시간에 다른 중요한 작업을 수행할 수 있음
            break;
            
        case I2C_STATE_TX_BUSY:
            // 송신 진행 중 - DMA가 자동으로 데이터를 전송하고 있음
            // CPU는 DMA 작업과 독립적으로 다른 작업 수행 가능
            // 완료되면 인터럽트에서 상태가 자동으로 변경됨
            break;
            
        case I2C_STATE_TX_COMPLETE:
            // 송신 완료됨 - 이제 수신 작업을 시작
            printf("송신 완료! 이제 수신을 시작합니다.\n");
            
            // 상태를 유휴로 변경 후 수신 시작
            i2c_state = I2C_STATE_IDLE;
            
            // 같은 크기의 데이터를 다시 수신 (에코 테스트)
            uint16_t receive_size = strlen((char*)tx_buffer);
            Start_I2C_Receive(receive_size);
            break;
            
        case I2C_STATE_RX_BUSY:
            // 수신 진행 중 - DMA가 자동으로 데이터를 받고 있음
            // CPU는 DMA 작업과 독립적으로 다른 작업 수행 가능
            // 완료되면 인터럽트에서 상태가 자동으로 변경됨
            break;
            
        case I2C_STATE_RX_COMPLETE:
            // 수신 완료됨 - 받은 데이터를 처리하고 다시 송신 준비
            printf("수신 완료! 받은 데이터: %s\n", (char*)rx_buffer);
            
            // 데이터 처리 완료 후 상태 초기화
            Reset_I2C_State();
            
            // 3초 후 다시 송신 시작 (연속적인 통신 테스트)
            HAL_Delay(3000);
            uint16_t transmit_size = strlen((char*)tx_buffer);
            Start_I2C_Transmit(tx_buffer, transmit_size);
            break;
            
        case I2C_STATE_ERROR:
            // 에러 상태 - 에러 정보를 출력하고 복구 시도
            printf("I2C 에러 발생! 에러 코드: 0x%08X\n", (unsigned int)error_code);
            
            // I2C 하드웨어 상태 확인 및 출력
            uint32_t i2c_error = HAL_I2C_GetError(&hi2c1);
            if (i2c_error != HAL_I2C_ERROR_NONE)
            {
                printf("HAL I2C 에러: ");
                if (i2c_error & HAL_I2C_ERROR_BERR)    printf("Bus Error ");
                if (i2c_error & HAL_I2C_ERROR_ARLO)    printf("Arbitration Lost ");
                if (i2c_error & HAL_I2C_ERROR_AF)      printf("Acknowledge Failure ");
                if (i2c_error & HAL_I2C_ERROR_OVR)     printf("Overrun/Underrun ");
                if (i2c_error & HAL_I2C_ERROR_DMA)     printf("DMA Error ");
                if (i2c_error & HAL_I2C_ERROR_TIMEOUT) printf("Timeout ");
                printf("\n");
            }
            
            // 에러 복구 시도 - I2C 인터페이스 재초기화
            printf("I2C 재초기화 시도 중...\n");
            HAL_I2C_DeInit(&hi2c1);  // I2C 해제
            HAL_Delay(100);          // 100ms 대기
            I2C1_Init();             // I2C 재초기화
            
            // 상태 초기화 후 5초 대기
            Reset_I2C_State();
            HAL_Delay(5000);
            
            // 재시작 시도
            uint16_t restart_size = strlen((char*)tx_buffer);
            Start_I2C_Transmit(tx_buffer, restart_size);
            break;
            
        default:
            // 예상하지 못한 상태 - 강제로 에러 상태로 변경
            printf("알 수 없는 상태: %d\n", i2c_state);
            i2c_state = I2C_STATE_ERROR;
            error_code = 0xFFFFFFFF;  // 알 수 없는 에러 코드
            break;
    }
}

/**
 * I2C 상태를 초기화하는 함수
 * 에러 복구나 작업 완료 후 호출됨
 */
void Reset_I2C_State(void)
{
    i2c_state = I2C_STATE_IDLE;  // 상태를 유휴로 변경
    error_code = 0;              // 에러 코드 초기화
    timeout_counter = 0;         // 타임아웃 카운터 초기화
    printf("I2C 상태 초기화 완료\n");
}

/* ========== I2C 초기화 함수 ========== */

/**
 * I2C1 인터페이스를 초기화하는 함수
 * 통신 속도, 주소 모드 등의 파라미터를 설정
 */
void I2C1_Init(void)
{
    // I2C1 인스턴스 지정 - STM32의 I2C1 하드웨어 사용
    hi2c1.Instance = I2C1;
    
    // 클록 속도 설정 - 100kHz (표준 모드)
    // 400kHz는 고속 모드, 1MHz는 고속+ 모드
    hi2c1.Init.ClockSpeed = 100000;
    
    // 듀티 사이클 설정 - 2는 50% 듀티 사이클 의미
    // 고속 모드에서는 16/9 비율도 사용 가능
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    
    // 자신의 주소 설정 - 마스터 모드에서는 사용되지 않음
    hi2c1.Init.OwnAddress1 = 0;
    
    // 주소 모드 설정 - 7비트 주소 사용 (10비트도 가능)
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    
    // 듀얼 주소 모드 비활성화 - 하나의 주소만 사용
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    
    // 두 번째 주소 설정 - 듀얼 주소 모드 비활성화로 사용안함
    hi2c1.Init.OwnAddress2 = 0;
    
    // 일반 호출 모드 비활성화 - 브로드캐스트 기능 사용안함
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    
    // 클록 스트레칭 활성화 - 슬레이브가 클록을 늦출 수 있음
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    // I2C 하드웨어 초기화 실행
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        printf("I2C 초기화 실패\n");
        Error_Handler();  // 초기화 실패 시 에러 핸들러 호출
    }
    
    printf("I2C1 초기화 완료 (100kHz, 7-bit 주소 모드)\n");
}

/* ========== DMA 초기화 함수 ========== */

/**
 * DMA 컨트롤러를 초기화하는 함수
 * I2C 송신과 수신용 DMA 스트림을 각각 설정
 */
void DMA_Init(void)
{
    // DMA1 컨트롤러의 클록 활성화 - DMA 사용을 위해 필수
    __HAL_RCC_DMA1_CLK_ENABLE();
    
    printf("DMA 컨트롤러 클록 활성화 완료\n");
    
    /* ========== I2C1 송신용 DMA 설정 (TX) ========== */
    
    // DMA1의 Stream6을 I2C1 송신용으로 사용 (STM32F4 기준)
    hdma_i2c1_tx.Instance = DMA1_Stream6;
    
    // DMA 채널 1번 사용 - I2C1_TX에 해당하는 채널
    hdma_i2c1_tx.Init.Channel = DMA_CHANNEL_1;
    
    // 전송 방향: 메모리 → 주변장치 (송신)
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    
    // 주변장치(I2C) 주소는 고정 - 증가하지 않음
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    
    // 메모리 주소는 자동 증가 - 버퍼의 다음 데이터로 이동
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    
    // 주변장치 데이터 크기: 8비트 (1바이트) 단위
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    
    // 메모리 데이터 크기: 8비트 (1바이트) 단위
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    
    // 노멀 모드 사용 - 한 번 전송 후 중단 (순환 모드 아님)
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    
    // 우선순위: 높음 - 송신은 중요한 작업
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    
    // FIFO 모드 비활성화 - 직접 전송 모드 사용
    hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    
    // DMA TX 초기화 실행
    if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
        printf("DMA TX 초기화 실패\n");
        Error_Handler();
    }
    
    // I2C 핸들러와 DMA TX 핸들러를 연결
    __HAL_LINKDMA(&hi2c1, hdmatx, hdma_i2c1_tx);
    
    printf("I2C TX DMA 설정 완료 (Stream6, Channel1)\n");
    
    /* ========== I2C1 수신용 DMA 설정 (RX) ========== */
    
    // DMA1의 Stream5를 I2C1 수신용으로 사용 (STM32F4 기준)
    hdma_i2c1_rx.Instance = DMA1_Stream5;
    
    // DMA 채널 1번 사용 - I2C1_RX에 해당하는 채널
    hdma_i2c1_rx.Init.Channel = DMA_CHANNEL_1;
    
    // 전송 방향: 주변장치 → 메모리 (수신)
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    
    // 주변장치(I2C) 주소는 고정 - 증가하지 않음
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    
    // 메모리 주소는 자동 증가 - 버퍼의 다음 위치에 저장
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    
    // 주변장치 데이터 크기: 8비트 (1바이트) 단위
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    
    // 메모리 데이터 크기: 8비트 (1바이트) 단위
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    
    // 노멀 모드 사용 - 한 번 수신 후 중단
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    
    // 우선순위: 매우 높음 - 수신은 데이터 손실 방지를 위해 최우선
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    
    // FIFO 모드 비활성화 - 직접 수신 모드 사용
    hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    
    // DMA RX 초기화 실행
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
        printf("DMA RX 초기화 실패\n");
        Error_Handler();
    }
    
    // I2C 핸들러와 DMA RX 핸들러를 연결
    __HAL_LINKDMA(&hi2c1, hdmarx, hdma_i2c1_rx);
    
    printf("I2C RX DMA 설정 완료 (Stream5, Channel1)\n");
    
    /* ========== 인터럽트 우선순위 설정 ========== */
    
    // DMA TX 완료 인터럽트 설정 - 우선순위 1, 서브 우선순위 0
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);  // 인터럽트 활성화
    
    printf("DMA TX 인터럽트 설정 완료 (우선순위 1)\n");
    
    // DMA RX 완료 인터럽트 설정 - 우선순위 0 (최고 우선순위)
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);  // 인터럽트 활성화
    
    printf("DMA RX 인터럽트 설정 완료 (우선순위 0)\n");
    
    // I2C 이벤트 인터럽트 설정 - 우선순위 1, 서브 우선순위 1
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);  // 인터럽트 활성화
    
    printf("I2C 이벤트 인터럽트 설정 완료\n");
    
    // I2C 에러 인터럽트 설정 - 우선순위 0, 서브 우선순위 1 (높은 우선순위)
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);  // 인터럽트 활성화
    
    printf("I2C 에러 인터럽트 설정 완료\n");
    
    printf("모든 DMA 및 인터럽트 설정 완료\n");
}

/* ========== GPIO 초기화 함수 ========== */

/**
 * I2C 통신용 GPIO 핀을 초기화하는 함수
 * SCL(Serial Clock)과 SDA(Serial Data) 핀을 설정
 */
void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};  // GPIO 설정 구조체 초기화
    
    // GPIOB 클록 활성화 - I2C1이 PB8, PB9 핀을 사용하기 때문
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    printf("GPIOB 클록 활성화 완료\n");
    
    /**
     * I2C1 GPIO 핀 매핑 (STM32F4 기준):
     * PB8 → I2C1_SCL (Serial Clock Line) - 클록 신호
     * PB9 → I2C1_SDA (Serial Data Line) - 데이터 신호
     */
    
    // 사용할 핀 선택 - PB8과 PB9 동시에 설정
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    
    // 대체 기능(Alternate Function) 오픈 드레인 모드 - I2C 표준
    // 오픈 드레인: 풀업 저항을 통해 HIGH, 트랜지스터로 LOW 구현
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    
    // 내부 풀업 저항 활성화 - I2C는 풀업 저항이 필요함
    // 외부 풀업 저항(4.7kΩ)이 있어도 추가 안정성을 위해 설정
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    
    // GPIO 속도를 매우 높음으로 설정 - 고속 I2C 통신 지원
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    
    // I2C1 대체 기능 선택 - STM32F4에서 AF4가 I2C1에 해당
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    
    // GPIO 초기화 실행 - 위에서 설정한 파라미터로 핀 설정
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    printf("I2C1 GPIO 초기화 완료 (PB8:SCL, PB9:SDA, 오픈드레인+풀업)\n");
}

/* ========== 타이머 초기화 함수 ========== */

/**
 * 타임아웃 체크용 타이머를 초기화하는 함수
 * 주기적으로 인터럽트를 발생시켜 타임아웃을 모니터링
 */
void Timer_Init(void)
{
    // 실제 구현에서는 TIM2나 다른 타이머를 사용하여
    // 주기적으로 timeout_counter를 증가시키고
    // 설정된 시간을 초과하면 타임아웃 처리를 수행
    
    // 여기서는 간단한 예시로 SysTick을 사용
    // (실제로는 HAL_GetTick()으로 시간 체크 가능)
    
    printf("타이머 초기화 완료 (타임아웃 모니터링용)\n");
}

/* ========== 시스템 클록 설정 함수 ========== */

/**
 * STM32 시스템 클록을 설정하는 함수
 * PLL을 사용하여 고속 클록 생성 (180MHz)
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};  // 오실레이터 설정 구조체
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  // 클록 설정 구조체

    // 전력 관리 클록 활성화 - 전압 스케일링을 위해 필요
    __HAL_RCC_PWR_CLK_ENABLE();
    
    // 전압 스케일링을 Scale 1으로 설정 - 최고 성능 모드
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // HSI(내부 고속 오실레이터) 설정 및 PLL 구성
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;  // HSI 사용
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;                    // HSI 활성화
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;  // 기본 보정값
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // PLL 활성화
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;       // PLL 소스: HSI
    
    // PLL 계수 설정: (HSI=16MHz) / PLLM * PLLN / PLLP = 최종 주파수
    RCC_OscInitStruct.PLL.PLLM = 8;   // 16MHz / 8 = 2MHz (PLL 입력)
    RCC_OscInitStruct.PLL.PLLN = 180; // 2MHz * 180 = 360MHz (VCO 출력)
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  // 360MHz / 2 = 180MHz (시스템 클록)
    RCC_OscInitStruct.PLL.PLLQ = 4;   // USB 클록용: 360MHz / 4 = 90MHz

    // 오실레이터 설정 실행
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        printf("오실레이터 설정 실패\n");
        Error_Handler();
    }

    // 시스템 클록 소스 및 분주비 설정
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // PLL을 시스템 클록으로 사용
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // HCLK = 180MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;         // PCLK1 = 45MHz (I2C 클록 도메인)
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;         // PCLK2 = 90MHz

    // 클록 설정 실행 - FLASH 대기 주기도 함께 설정
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        printf("시스템 클록 설정 실패\n");
        Error_Handler();
    }
    
    printf("시스템 클록 설정 완료 (SYSCLK:180MHz, APB1:45MHz, APB2:90MHz)\n");
}

/* ========== 인터럽트 서비스 루틴(ISR) 함수들 ========== */

/**
 * DMA1 Stream6 인터럽트 핸들러 (I2C1 송신용)
 * DMA를 통한 데이터 송신이 완료되면 자동으로 호출됨
 * 
 * 호출 순서:
 * 1. DMA Controller가 모든 데이터 전송 완료
 * 2. DMA Controller가 CPU에 IRQ 신호 전송
 * 3. Interrupt Controller가 이 함수 호출
 * 4. 이 함수가 HAL 라이브러리의 핸들러 호출
 * 5. HAL이 적절한 콜백 함수 호출
 */
void DMA1_Stream6_IRQHandler(void)
{
    // HAL DMA 인터럽트 핸들러 호출 - 송신용 DMA 처리
    // 이 함수 내부에서 전송 완료 여부를 확인하고
    // 완료되었으면 HAL_I2C_MasterTxCpltCallback을 호출함
    HAL_DMA_IRQHandler(&hdma_i2c1_tx);
}

/**
 * DMA1 Stream5 인터럽트 핸들러 (I2C1 수신용)
 * DMA를 통한 데이터 수신이 완료되면 자동으로 호출됨
 * 
 * 호출 순서:
 * 1. DMA Controller가 모든 데이터 수신 완료
 * 2. DMA Controller가 CPU에 IRQ 신호 전송
 * 3. Interrupt Controller가 이 함수 호출
 * 4. 이 함수가 HAL 라이브러리의 핸들러 호출
 * 5. HAL이 적절한 콜백 함수 호출
 */
void DMA1_Stream5_IRQHandler(void)
{
    // HAL DMA 인터럽트 핸들러 호출 - 수신용 DMA 처리
    // 이 함수 내부에서 수신 완료 여부를 확인하고
    // 완료되었으면 HAL_I2C_MasterRxCpltCallback을 호출함
    HAL_DMA_IRQHandler(&hdma_i2c1_rx);
}

/**
 * I2C1 이벤트 인터럽트 핸들러
 * I2C 통신 중 정상적인 이벤트(시작, 주소 매치, 데이터 전송 등)가
 * 발생했을 때 자동으로 호출됨
 */
void I2C1_EV_IRQHandler(void)
{
    // HAL I2C 이벤트 인터럽트 핸들러 호출
    // START, ADDR, BTF, STOPF 등의 이벤트를 처리
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

/**
 * I2C1 에러 인터럽트 핸들러
 * I2C 통신 중 에러(ACK 실패, 버스 에러, 타임아웃 등)가
 * 발생했을 때 자동으로 호출됨
 */
void I2C1_ER_IRQHandler(void)
{
    // HAL I2C 에러 인터럽트 핸들러 호출
    // BERR, ARLO, AF, OVR, TIMEOUT 등의 에러를 처리
    HAL_I2C_ER_IRQHandler(&hi2c1);
}

/* ========== HAL 콜백 함수들 ========== */

/**
 * I2C 마스터 송신 완료 콜백 함수
 * HAL_I2C_Master_Transmit_DMA()로 시작된 송신이 완료되면
 * HAL 라이브러리에서 자동으로 이 함수를 호출함
 * 
 * @param hi2c: 완료된 I2C 핸들러의 포인터
 * 
 * 호출 경로:
 * DMA 완료 → DMA ISR → HAL_DMA_IRQHandler → 이 콜백 함수
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // I2C1에서 발생한 이벤트인지 확인
    if (hi2c->Instance == I2C1)
    {
        printf("[인터럽트] I2C 송신 완료!\n");
        
        // 상태를 송신 완료로 변경 - 메인 루프에서 이 상태를 확인
        i2c_state = I2C_STATE_TX_COMPLETE;
        
        // 타임아웃 카운터 초기화 - 정상 완료되었으므로
        timeout_counter = 0;
        
        // 추가 처리가 필요하면 여기에 구현
        // 예: LED 점멸, 로그 기록, 다른 작업 트리거 등
    }
}

/**
 * I2C 마스터 수신 완료 콜백 함수
 * HAL_I2C_Master_Receive_DMA()로 시작된 수신이 완료되면
 * HAL 라이브러리에서 자동으로 이 함수를 호출함
 * 
 * @param hi2c: 완료된 I2C 핸들러의 포인터
 * 
 * 호출 경로:
 * DMA 완료 → DMA ISR → HAL_DMA_IRQHandler → 이 콜백 함수
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // I2C1에서 발생한 이벤트인지 확인
    if (hi2c->Instance == I2C1)
    {
        printf("[인터럽트] I2C 수신 완료!\n");
        
        // 수신된 데이터의 길이 확인 (NULL 종료 문자열인 경우)
        uint16_t received_length = strlen((char*)rx_buffer);
        printf("[인터럽트] 수신된 데이터 길이: %d 바이트\n", received_length);
        
        // 상태를 수신 완료로 변경 - 메인 루프에서 이 상태를 확인
        i2c_state = I2C_STATE_RX_COMPLETE;
        
        // 타임아웃 카운터 초기화 - 정상 완료되었으므로
        timeout_counter = 0;
        
        // 수신된 데이터의 간단한 검증 (옵션)
        if (received_length > 0 && received_length < BUFFER_SIZE)
        {
            printf("[인터럽트] 데이터 검증 성공\n");
        }
        else
        {
            printf("[인터럽트] 데이터 검증 실패 - 길이 이상\n");
        }
    }
}

/**
 * I2C 에러 콜백 함수
 * I2C 통신 중 어떤 종류의 에러든 발생하면
 * HAL 라이브러리에서 자동으로 이 함수를 호출함
 * 
 * @param hi2c: 에러가 발생한 I2C 핸들러의 포인터
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    // I2C1에서 발생한 에러인지 확인
    if (hi2c->Instance == I2C1)
    {
        printf("[인터럽트] I2C 에러 발생!\n");
        
        // HAL에서 제공하는 에러 코드 얻기
        uint32_t hal_error = HAL_I2C_GetError(hi2c);
        
        // 각 에러 타입별로 상세 정보 출력
        printf("[인터럽트] HAL 에러 코드: 0x%08X\n", (unsigned int)hal_error);
        
        if (hal_error & HAL_I2C_ERROR_BERR)
        {
            printf("[인터럽트] - Bus Error: 잘못된 시작/정지 조건\n");
        }
        if (hal_error & HAL_I2C_ERROR_ARLO)
        {
            printf("[인터럽트] - Arbitration Lost: 다중 마스터 경쟁에서 패배\n");
        }
        if (hal_error & HAL_I2C_ERROR_AF)
        {
            printf("[인터럽트] - Acknowledge Failure: 슬레이브 응답 없음\n");
        }
        if (hal_error & HAL_I2C_ERROR_OVR)
        {
            printf("[인터럽트] - Overrun/Underrun: 데이터 처리 속도 부족\n");
        }
        if (hal_error & HAL_I2C_ERROR_DMA)
        {
            printf("[인터럽트] - DMA Error: DMA 전송 중 문제 발생\n");
        }
        if (hal_error & HAL_I2C_ERROR_TIMEOUT)
        {
            printf("[인터럽트] - Timeout Error: 응답 시간 초과\n");
        }
        
        // 전역 에러 코드에 저장 - 메인 루프에서 참조 가능
        error_code = hal_error;
        
        // 상태를 에러로 변경 - 메인 루프에서 에러 복구 시도
        i2c_state = I2C_STATE_ERROR;
        
        // 진행 중인 DMA 전송이 있다면 중단 시도
        if (hi2c->hdmatx != NULL)
        {
            HAL_DMA_Abort(hi2c->hdmatx);  // TX DMA 중단
        }
        if (hi2c->hdmarx != NULL)
        {
            HAL_DMA_Abort(hi2c->hdmarx);  // RX DMA 중단
        }
    }
}

/**
 * I2C 중단 완료 콜백 함수
 * HAL_I2C_Master_Abort_IT() 또는 자동 중단이 완료되면 호출됨
 * 주로 에러 복구 과정에서 사용됨
 * 
 * @param hi2c: 중단된 I2C 핸들러의 포인터
 */
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // I2C1에서 발생한 이벤트인지 확인
    if (hi2c->Instance == I2C1)
    {
        printf("[인터럽트] I2C 통신 중단 완료\n");
        
        // 중단 완료 후 상태를 유휴로 변경
        i2c_state = I2C_STATE_IDLE;
        
        // 에러 코드 초기화
        error_code = 0;
    }
}

/* ========== HAL MSP (MCU Support Package) 함수들 ========== */

/**
 * I2C MSP 초기화 함수
 * HAL_I2C_Init() 함수에서 자동으로 호출됨
 * 하드웨어 레벨의 초기화를 담당 (클록, GPIO 등)
 * 
 * @param hi2c: 초기화할 I2C 핸들러의 포인터
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    // I2C1 초기화인지 확인
    if (hi2c->Instance == I2C1)
    {
        printf("[MSP] I2C1 하드웨어 초기화 시작\n");
        
        // I2C1 클록 활성화 - I2C 하드웨어가 동작하기 위해 필수
        __HAL_RCC_I2C1_CLK_ENABLE();
        printf("[MSP] I2C1 클록 활성화 완료\n");
        
        // GPIO는 이미 GPIO_Init()에서 설정했으므로 추가 설정 불필요
        // 만약 GPIO를 여기서 설정하고 싶다면 아래와 같이 구현:
        /*
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        */
        
        printf("[MSP] I2C1 MSP 초기화 완료\n");
    }
}

/**
 * I2C MSP 해제 함수
 * HAL_I2C_DeInit() 함수에서 자동으로 호출됨
 * 하드웨어 리소스를 해제하여 전력 소모 최소화
 * 
 * @param hi2c: 해제할 I2C 핸들러의 포인터
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
    // I2C1 해제인지 확인
    if (hi2c->Instance == I2C1)
    {
        printf("[MSP] I2C1 하드웨어 해제 시작\n");
        
        // I2C1 클록 비활성화 - 전력 절약
        __HAL_RCC_I2C1_CLK_DISABLE();
        printf("[MSP] I2C1 클록 비활성화 완료\n");
        
        // GPIO 핀을 기본 상태로 되돌림
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
        printf("[MSP] GPIO 핀 해제 완료\n");
        
        // 연결된 DMA 해제
        if (hi2c->hdmatx != NULL)
        {
            HAL_DMA_DeInit(hi2c->hdmatx);  // TX DMA 해제
            printf("[MSP] TX DMA 해제 완료\n");
        }
        if (hi2c->hdmarx != NULL)
        {
            HAL_DMA_DeInit(hi2c->hdmarx);  // RX DMA 해제
            printf("[MSP] RX DMA 해제 완료\n");
        }
        
        // 모든 관련 인터럽트 비활성화
        HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);        // I2C 이벤트 인터럽트
        HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);        // I2C 에러 인터럽트
        HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);   // RX DMA 인터럽트
        HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);   // TX DMA 인터럽트
        
        printf("[MSP] 모든 인터럽트 비활성화 완료\n");
        printf("[MSP] I2C1 MSP 해제 완료\n");
    }
}

/* ========== 에러 핸들러 함수 ========== */

/**
 * 시스템 에러 핸들러
 * 복구 불가능한 에러가 발생했을 때 호출됨
 * 디버깅 정보를 출력하고 시스템을 안전한 상태로 만듦
 */
void Error_Handler(void)
{
    printf("\n=== 시스템 에러 발생 ===\n");
    printf("복구 불가능한 에러가 발생했습니다.\n");
    printf("현재 I2C 상태: %d\n", i2c_state);
    printf("마지막 에러 코드: 0x%08X\n", (unsigned int)error_code);
    printf("시스템을 안전 모드로 전환합니다.\n");
    
    // 모든 인터럽트 비활성화 - 추가 에러 방지
    __disable_irq();
    
    // LED나 다른 표시 장치가 있다면 에러 상태를 표시
    // 예: 빨간 LED 점멸, 부저 울림 등
    
    // 무한 루프 - 시스템 정지 상태
    // 디버거를 사용하는 경우 여기에 브레이크포인트 설정
    while (1)
    {
        // 에러 상태 표시 (예: LED 점멸)
        // HAL_GPIO_TogglePin(ERROR_LED_PORT, ERROR_LED_PIN);
        // HAL_Delay(500);
        
        // 디버깅 시 이곳에서 멈춤
        // 스택 추적, 레지스터 값, 메모리 상태 등을 확인 가능
    }
}

/**
 * 프로그램 사용 가이드 및 기술적 설명
 * 
 * ========== 순수 인터럽트 기반 동작 원리 ==========
 * 
 * 1. 초기화 단계:
 *    - 시스템 클록, GPIO, DMA, I2C 순서로 초기화
 *    - 모든 인터럽트 우선순위 설정 완료
 *    - 상태 머신을 IDLE 상태로 초기화
 * 
 * 2. 송신 과정 (완전한 인터럽트 방식):
 *    a) 메인 함수에서 Start_I2C_Transmit() 호출
 *    b) HAL_I2C_Master_Transmit_DMA() 실행 (즉시 반환)
 *    c) CPU는 다른 작업 수행 (메인 루프에서 대기하지 않음)
 *    d) DMA Controller가 독립적으로 메모리→I2C로 데이터 전송
 *    e) 전송 완료 시 DMA가 CPU에 IRQ 신호 전송
 *    f) DMA1_Stream6_IRQHandler() 자동 호출
 *    g) HAL_I2C_MasterTxCpltCallback() 자동 호출
 *    h) 콜백에서 상태를 TX_COMPLETE로 변경
 *    i) 메인 루프에서 상태 변화 감지 후 다음 작업 시작
 * 
 * 3. 수신 과정 (완전한 인터럽트 방식):
 *    - 송신과 동일한 메커니즘이지만 반대 방향 (I2C→메모리)
 *    - DMA1_Stream5_IRQHandler() 및 HAL_I2C_MasterRxCpltCallback() 사용
 * 
 * 4. 에러 처리:
 *    - 모든 에러는 인터럽트를 통해 즉시 감지
 *    - HAL_I2C_ErrorCallback()에서 에러 분석 및 복구 시도
 *    - 상태 머신을 통한 체계적인 에러 복구
 * 
 * ========== 기존 하이브리드 방식과의 차이점 ==========
 * 
 * 기존 방식 (하이브리드):
 * - DMA 시작 → while 루프에서 플래그 폴링 → 완료 확인 → 다음 작업
 * - CPU가 계속 플래그를 체크하며 대기 (CPU 자원 낭비)
 * - 타임아웃 체크를 위해 HAL_GetTick() 지속적 호출
 * 
 * 새로운 방식 (순수 인터럽트):
 * - DMA 시작 → 즉시 다른 작업 수행 → 인터럽트 발생 시 콜백 실행
 * - CPU는 대기하지 않고 유용한 작업 수행 (CPU 효율성 극대화)
 * - 상태 머신을 통한 체계적인 흐름 제어
 * 
 * ========== 하드웨어 연결 정보 ==========
 * 
 * STM32F4 기준 연결:
 * - PB8 (I2C1_SCL) ← 클록 라인
 * - PB9 (I2C1_SDA) ← 데이터 라인
 * - 풀업 저항 4.7kΩ를 VCC에 연결 (SCL, SDA 각각)
 * - 슬레이브 디바이스의 SCL, SDA 핀과 연결
 * - 공통 그라운드 연결
 * 
 * ========== 성능 최적화 포인트 ==========
 * 
 * 1. DMA 우선순위:
 *    - RX: VERY_HIGH (데이터 손실 방지 최우선)
 *    - TX: HIGH (송신 지연 최소화)
 * 
 * 2. 인터럽트 우선순위:
 *    - I2C 에러: 0 (최고 우선순위, 즉시 처리)
 *    - DMA RX: 0 (데이터 손실 방지)
 *    - DMA TX: 1 (송신 완료 확인)
 *    - I2C 이벤트: 1 (일반 이벤트 처리)
 * 
 * 3. CPU 효율성:
 *    - 폴링 루프 완전 제거
 *    - 상태 기반 작업 분할
 *    - 메인 루프에서 다른 중요 작업 수행 가능
 * 
 * ========== 디버깅 및 문제 해결 ==========
 * 
 * 1. 일반적인 문제들:
 *    - ACK 실패: 슬레이브 주소 확인, 연결 상태 점검
 *    - 타임아웃: 풀업 저항, 클록 속도, 케이블 길이 확인
 *    - DMA 에러: 메모리 정렬, 버퍼 크기 확인
 * 
 * 2. 디버깅 도구:
 *    - printf 출력으로 상태 추적
 *    - Error_Handler()에 브레이크포인트 설정
 *    - 로직 애널라이저로 I2C 신호 분석
 *    - DMA 레지스터 및 I2C 상태 레지스터 모니터링
 * 
 * 3. 테스트 방법:
 *    - EEPROM, RTC, 센서 등의 I2C 디바이스로 테스트
 *    - 다양한 데이터 크기로 송수신 테스트
 *    - 장시간 연속 동작 테스트
 *    - 에러 상황 강제 발생으로 복구 기능 테스트
 */