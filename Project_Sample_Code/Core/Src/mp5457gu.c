/**
 * @file mp5475gu.c
 * @brief MP5475GU PMIC I2C DMA Interrupt 방식 구현
 * @author Brake System Team
 * @date 2025-09-05
 *
 * @note I2C DMA Interrupt 방식으로 PMIC 상태를 읽고 고장을 감지합니다.
 *       DMA를 사용하여 CPU 부하를 최소화하고 실시간 성능을 보장합니다.
 */

#include "mp5475gu.h"
#include <string.h>
#include <stdio.h>

//=============================================================================
// 전역 변수 (I2C DMA 동작을 위한 상태 관리)
//=============================================================================

/**
 * @brief I2C DMA 동작 상태 구조체
 */
typedef struct {
    volatile bool           dma_busy;           // DMA 전송 진행 중 플래그
    volatile bool           dma_complete;       // DMA 전송 완료 플래그
    volatile bool           dma_error;          // DMA 에러 발생 플래그
    mp5475_data_t          current_data;        // 현재 읽은 PMIC 데이터
    uint32_t               error_count;         // 누적 에러 카운터
    uint32_t               success_count;       // 성공 카운터
} i2c_dma_state_t;

// I2C DMA 상태 변수 (정적으로 선언하여 인터럽트에서 접근 가능)
static i2c_dma_state_t i2c_state = {0};

// DMA 수신용 버퍼 (8바이트 - 레지스터 0x05~0x09 + 추가)
static uint8_t i2c_rx_buffer[8] __attribute__((aligned(4)));

// DMA 송신용 버퍼 (레지스터 주소 + 데이터)
static uint8_t i2c_tx_buffer[2] __attribute__((aligned(4)));

//=============================================================================
// I2C DMA 초기화 함수
//=============================================================================

/**
 * @brief MP5475GU I2C DMA 초기화
 * @return true: 성공, false: 실패
 *
 * @note I2C 하드웨어, DMA 채널, 인터럽트를 설정합니다.
 *       실제 구현에서는 MCU별 HAL 라이브러리를 사용합니다.
 */
bool mp5475_init(void)
{
    printf("[I2C-DMA] MP5475GU 초기화 시작...\n");

    // 1단계: I2C 하드웨어 초기화
    // - I2C 클럭: 400kHz (고속 모드)
    // - 주소 모드: 7비트
    // - DMA 모드 활성화
    /*
    실제 코드 예시 (STM32 HAL):
    hi2c.Instance = I2C1;
    hi2c.Init.ClockSpeed = 400000;
    hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c.Init.OwnAddress1 = 0;
    hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    HAL_I2C_Init(&hi2c);
    */

    // 2단계: DMA 채널 초기화
    // - DMA 채널: I2C TX/RX 전용
    // - 메모리-페리페럴 모드
    // - 순환 모드 비활성화 (단발성 전송)
    /*
    실제 코드 예시:
    hdma_i2c_tx.Instance = DMA1_Channel6;
    hdma_i2c_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    HAL_DMA_Init(&hdma_i2c_tx);
    */

    // 3단계: 인터럽트 우선순위 설정
    // - I2C 인터럽트: 높은 우선순위 (실시간 성능)
    // - DMA 인터럽트: 중간 우선순위
    /*
    실제 코드 예시:
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    */

    // 4단계: 상태 변수 초기화
    memset(&i2c_state, 0, sizeof(i2c_state));
    memset(i2c_rx_buffer, 0, sizeof(i2c_rx_buffer));
    memset(i2c_tx_buffer, 0, sizeof(i2c_tx_buffer));

    // 5단계: PMIC 연결 확인 (Vendor ID 읽기)
    if (!mp5475_read_vendor_id(NULL)) {
        printf("[I2C-DMA] PMIC 연결 확인 실패!\n");
        return false;
    }

    printf("[I2C-DMA] MP5475GU 초기화 완료\n");
    return true;
}

//=============================================================================
// I2C DMA 읽기 함수 (비동기)
//=============================================================================

/**
 * @brief MP5475GU 상태 레지스터 일괄 읽기 (DMA 방식)
 * @param data 8바이트 데이터 구조체 포인터
 * @return true: 성공, false: 실패
 *
 * @note 레지스터 0x05~0x09를 DMA로 연속 읽기합니다.
 *       비동기 방식이므로 완료는 인터럽트에서 확인합니다.
 */
bool mp5475_read_status_registers(mp5475_data_t *data)
{
    // 1단계: DMA 사용 가능 여부 확인
    if (i2c_state.dma_busy) {
        printf("[I2C-DMA] DMA 사용 중, 대기 필요\n");
        return false;
    }

    // 2단계: DMA 플래그 초기화
    i2c_state.dma_busy = true;
    i2c_state.dma_complete = false;
    i2c_state.dma_error = false;

    // 3단계: 송신 버퍼 준비 (시작 레지스터 주소)
    i2c_tx_buffer[0] = MP5475_REG_SYSTEM_STATUS;  // 0x05부터 시작

    printf("[I2C-DMA] 상태 레지스터 읽기 시작 (0x05~0x09)\n");

    // 4단계: I2C DMA 송신 시작 (레지스터 주소 전송)
    /*
    실제 코드 예시 (STM32 HAL):
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_DMA(
        &hi2c,                          // I2C 핸들
        MP5475GU_I2C_ADDR_WRITE,       // 슬레이브 주소 (Write)
        i2c_tx_buffer,                  // 송신 데이터 (레지스터 주소)
        1,                              // 송신 바이트 수
    );

    if (status != HAL_OK) {
        i2c_state.dma_busy = false;
        i2c_state.dma_error = true;
        return false;
    }
    */

    // 5단계: 실제 구현에서는 여기서 함수 종료
    // DMA 송신 완료 인터럽트에서 수신을 시작합니다.

    // 시뮬레이션: 성공적으로 DMA 시작됨을 표시
    printf("[I2C-DMA] DMA 전송 시작됨 (비동기)\n");

    // 실제로는 인터럽트에서 처리되지만, 시뮬레이션을 위해 바로 완료 처리
    mp5475_simulate_dma_complete();

    return true;
}

//=============================================================================
// I2C DMA 완료 콜백 함수 (인터럽트에서 호출)
//=============================================================================

/**
 * @brief I2C DMA 송신 완료 콜백
 * @note 레지스터 주소 송신 완료 후 수신을 시작합니다.
 *       이 함수는 인터럽트 컨텍스트에서 호출됩니다.
 */
void mp5475_i2c_tx_dma_complete_callback(void)
{
    printf("[I2C-DMA] 송신 완료, 수신 시작\n");

    // 1단계: Restart 조건으로 수신 모드 전환
    /*
    실제 코드 예시:
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive_DMA(
        &hi2c,                          // I2C 핸들
        MP5475GU_I2C_ADDR_READ,        // 슬레이브 주소 (Read)
        i2c_rx_buffer,                  // 수신 버퍼
        5,                              // 수신 바이트 수 (0x05~0x09)
    );

    if (status != HAL_OK) {
        i2c_state.dma_error = true;
        i2c_state.dma_busy = false;
    }
    */
}

/**
 * @brief I2C DMA 수신 완료 콜백
 * @note 레지스터 데이터 수신 완료 후 데이터를 처리합니다.
 *       이 함수는 인터럽트 컨텍스트에서 호출됩니다.
 */
void mp5475_i2c_rx_dma_complete_callback(void)
{
    printf("[I2C-DMA] 수신 완료, 데이터 처리 중\n");

    // 1단계: 수신된 데이터를 구조체에 복사
    i2c_state.current_data.status_regs.system_status.raw = i2c_rx_buffer[0];  // 0x05
    i2c_state.current_data.status_regs.power_good.raw = i2c_rx_buffer[1];     // 0x06
    i2c_state.current_data.status_regs.uv_ov_fault.raw = i2c_rx_buffer[2];   // 0x07
    i2c_state.current_data.status_regs.oc_fault.raw = i2c_rx_buffer[3];      // 0x08
    i2c_state.current_data.status_regs.temp_fault.raw = i2c_rx_buffer[4];    // 0x09

    // 2단계: DMA 완료 플래그 설정
    i2c_state.dma_complete = true;
    i2c_state.dma_busy = false;
    i2c_state.success_count++;

    // 3단계: 고장 감지 및 DTC 생성 (인터럽트에서는 간단히 처리)
    brake_dtc_code_t dtc = mp5475_check_faults(&i2c_state.current_data);
    if (dtc != 0) {
        printf("[I2C-DMA] 고장 감지됨: DTC 0x%04X\n", dtc);
        // 실제로는 DTC 저장을 위해 SPI 모듈에 신호 전송
    }

    printf("[I2C-DMA] 데이터 처리 완료 (성공: %lu회)\n", i2c_state.success_count);
}

/**
 * @brief I2C DMA 에러 콜백
 * @note DMA 전송 중 에러 발생시 호출됩니다.
 */
void mp5475_i2c_error_callback(void)
{
    printf("[I2C-DMA] 에러 발생!\n");

    // 1단계: 에러 플래그 설정
    i2c_state.dma_error = true;
    i2c_state.dma_busy = false;
    i2c_state.error_count++;

    // 2단계: 에러 복구 시도 (실제로는 하드웨어 리셋 등)
    /*
    실제 코드 예시:
    HAL_I2C_DeInit(&hi2c);
    HAL_I2C_Init(&hi2c);
    */

    printf("[I2C-DMA] 에러 복구 시도 (에러: %lu회)\n", i2c_state.error_count);
}

//=============================================================================
// 고장 감지 및 DTC 생성 함수
//=============================================================================

/**
 * @brief PMIC 고장 상태 확인 및 DTC 생성
 * @param pmic_data PMIC 데이터 구조체 포인터
 * @return brake_dtc_code_t 감지된 DTC 코드 (정상시 0)
 *
 * @note 비트필드를 사용하여 각 고장 상태를 효율적으로 확인합니다.
 */
brake_dtc_code_t mp5475_check_faults(mp5475_data_t *pmic_data)
{
    // 1단계: Under/Over Voltage 확인 (레지스터 0x07)
    if (pmic_data->status_regs.uv_ov_fault.bucka_uv ||
        pmic_data->status_regs.uv_ov_fault.buckb_uv ||
        pmic_data->status_regs.uv_ov_fault.buckc_uv ||
        pmic_data->status_regs.uv_ov_fault.buckd_uv) {
        printf("[I2C-DMA] Under Voltage 감지\n");
        return DTC_BRAKE_PMIC_UV;
    }

    if (pmic_data->status_regs.uv_ov_fault.bucka_ov ||
        pmic_data->status_regs.uv_ov_fault.buckb_ov ||
        pmic_data->status_regs.uv_ov_fault.buckc_ov ||
        pmic_data->status_regs.uv_ov_fault.buckd_ov) {
        printf("[I2C-DMA] Over Voltage 감지\n");
        return DTC_BRAKE_PMIC_OV;
    }

    // 2단계: Over Current 확인 (레지스터 0x08)
    if (pmic_data->status_regs.oc_fault.bucka_oc ||
        pmic_data->status_regs.oc_fault.buckb_oc ||
        pmic_data->status_regs.oc_fault.buckc_oc ||
        pmic_data->status_regs.oc_fault.buckd_oc) {
        printf("[I2C-DMA] Over Current 감지\n");
        return DTC_BRAKE_PMIC_OC;
    }

    // 3단계: 온도 고장 확인 (레지스터 0x09)
    if (pmic_data->status_regs.temp_fault.pmic_temp_shutdown ||
        pmic_data->status_regs.temp_fault.pmic_temp_warning) {
        printf("[I2C-DMA] Over Temperature 감지\n");
        return DTC_BRAKE_PMIC_TEMP;
    }

    // 4단계: 통신 에러 확인
    if (i2c_state.error_count > 0) {
        printf("[I2C-DMA] 통신 에러 확인\n");
        return DTC_BRAKE_COMM_ERROR;
    }

    return 0; // 정상 상태
}

//=============================================================================
// 상태 확인 함수
//=============================================================================

/**
 * @brief Power Good 상태 확인
 * @param pmic_data PMIC 데이터 구조체 포인터
 * @return true: 정상, false: 문제 있음
 */
bool mp5475_is_power_good(mp5475_data_t *pmic_data)
{
    // 모든 Buck의 Power Good 상태 확인 (필터링된 값 사용)
    return (pmic_data->status_regs.power_good.bucka_pg_filt &&
            pmic_data->status_regs.power_good.buckb_pg_filt &&
            pmic_data->status_regs.power_good.buckc_pg_filt &&
            pmic_data->status_regs.power_good.buckd_pg_filt);
}

/**
 * @brief DMA 전송 완료 대기
 * @param timeout_ms 타임아웃 (밀리초)
 * @return true: 완료, false: 타임아웃
 */
bool mp5475_wait_dma_complete(uint32_t timeout_ms)
{
    uint32_t start_time = 0; // 실제로는 HAL_GetTick() 사용

    while (!i2c_state.dma_complete && !i2c_state.dma_error) {
        uint32_t current_time = 0; // 실제로는 HAL_GetTick() 사용
        if ((current_time - start_time) > timeout_ms) {
            printf("[I2C-DMA] 타임아웃 발생\n");
            return false;
        }
        // 실제로는 여기서 다른 작업 수행 가능 (비동기 장점)
    }

    return i2c_state.dma_complete;
}

//=============================================================================
// 시뮬레이션 함수 (실제 구현에서는 제거)
//=============================================================================

/**
 * @brief DMA 완료 시뮬레이션 (테스트용)
 * @note 실제 환경에서는 인터럽트가 이 역할을 담당합니다.
 */
void mp5475_simulate_dma_complete(void)
{
    // 시뮬레이션: 정상 데이터 생성
    i2c_rx_buffer[0] = 0x03;  // 시스템 상태: PMR_GOOD=1, FSM_STATE=1
    i2c_rx_buffer[1] = 0xFF;  // 모든 Buck Power Good
    i2c_rx_buffer[2] = 0x00;  // UV/OV 없음
    i2c_rx_buffer[3] = 0x00;  // OC 없음
    i2c_rx_buffer[4] = 0x00;  // 온도 정상

    // 송신 완료 -> 수신 완료 시뮬레이션
    mp5475_i2c_tx_dma_complete_callback();
    mp5475_i2c_rx_dma_complete_callback();
}

/**
 * @brief Vendor ID 읽기 (연결 확인용)
 * @param vendor_id Vendor ID 저장 포인터 (NULL 가능)
 * @return true: 성공, false: 실패
 */
bool mp5475_read_vendor_id(uint16_t *vendor_id)
{
    // 시뮬레이션: 정상적인 Vendor ID 반환
    printf("[I2C-DMA] Vendor ID 확인 중...\n");

    if (vendor_id != NULL) {
        *vendor_id = 0x1234; // 예시 Vendor ID
    }

    printf("[I2C-DMA] Vendor ID: 0x1234 (정상)\n");
    return true;
}

/**
 * @brief 현재 상태 출력 (디버깅용)
 */
void mp5475_print_status(void)
{
    printf("\n=== MP5475GU I2C DMA 상태 ===\n");
    printf("DMA 사용 중: %s\n", i2c_state.dma_busy ? "예" : "아니오");
    printf("성공 횟수: %lu\n", i2c_state.success_count);
    printf("에러 횟수: %lu\n", i2c_state.error_count);
    printf("Power Good: %s\n", mp5475_is_power_good(&i2c_state.current_data) ? "정상" : "이상");
    printf("=============================\n\n");
}
