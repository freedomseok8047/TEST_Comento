/**
 * MP5475GU PMIC I2C DMA 통신 구현
 * 순수 인터럽트 기반으로 레지스터 읽기 및 고장 진단
 */

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* ========== MP5475GU 레지스터 정의 ========== */
#define MP5475GU_I2C_ADDRESS    (0x69 << 1)  // 7bit 주소를 8bit로 변환

// PMIC 레지스터 주소
#define REG_PG_STATUS           0x06  // Power Good 상태 레지스터
#define REG_BUCK1_UV_STATUS     0x07  // Buck1 Undervoltage 상태
#define REG_BUCK2_UV_STATUS     0x08  // Buck2 Undervoltage 상태
#define REG_BUCK3_OV_STATUS     0x09  // Buck3 Overvoltage 상태

// 상태 비트 마스크
#define PG_BUCK1_MASK           0x01  // Buck1 Power Good 비트
#define PG_BUCK2_MASK           0x02  // Buck2 Power Good 비트
#define PG_BUCK3_MASK           0x04  // Buck3 Power Good 비트

#define UV_FAULT_MASK           0x80  // Undervoltage 고장 비트
#define OV_FAULT_MASK           0x80  // Overvoltage 고장 비트

// DTC (Diagnostic Trouble Code) 정의
#define DTC_BUCK1_UV            0xC001  // Buck1 Undervoltage 고장
#define DTC_BUCK1_OV            0xC002  // Buck1 Overvoltage 고장
#define DTC_BUCK2_UV            0xC003  // Buck2 Undervoltage 고장
#define DTC_BUCK2_OV            0xC004  // Buck2 Overvoltage 고장
#define DTC_BUCK3_UV            0xC005  // Buck3 Undervoltage 고장
#define DTC_BUCK3_OV            0xC006  // Buck3 Overvoltage 고장

/* ========== 외부 변수 선언 ========== */
extern I2C_HandleTypeDef hi2c1;          // I2C1 핸들러
extern volatile I2C_State_t i2c_state;   // I2C 상태

/* ========== 내부 변수 ========== */
static uint8_t pmic_reg_address;         // 현재 읽을 레지스터 주소
static uint8_t pmic_reg_data;            // 읽은 레지스터 데이터 저장
static uint8_t current_reg_index = 0;    // 현재 처리 중인 레지스터 인덱스

// 읽을 레지스터 주소 배열 (순서대로 처리)
static const uint8_t reg_addresses[] = {
    REG_PG_STATUS,       // 0x06 - Power Good 상태
    REG_BUCK1_UV_STATUS, // 0x07 - Buck1 UV 상태
    REG_BUCK2_UV_STATUS, // 0x08 - Buck2 UV 상태
    REG_BUCK3_OV_STATUS  // 0x09 - Buck3 OV 상태
};

// 읽은 레지스터 데이터 저장 배열
static uint8_t pmic_status_data[4];

// 검출된 DTC 코드 저장 배열
static uint16_t detected_dtc_codes[6];
static uint8_t dtc_count = 0;

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

/**
 * @brief PMIC I2C 통신 처리 메인 함수
 * @note 상태 머신 기반으로 순차적 레지스터 읽기 수행
 */
void process_i2c_step(void)
{
    printf("[I2C] Processing PMIC status check...\n");

    switch (i2c_state) {
        case I2C_STATE_IDLE:
            // 유휴 상태 - 새로운 진단 시작
            printf("[I2C] Starting PMIC diagnosis sequence\n");
            reset_pmic_diagnosis();

            // 첫 번째 레지스터(PG_STATUS) 읽기 시작
            current_reg_index = 0;
            if (start_pmic_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
                printf("[I2C] Failed to start PMIC register read\n");
                i2c_state = I2C_STATE_ERROR;
            }
            break;

        case I2C_STATE_TX_BUSY:
            // 레지스터 주소 송신 중 - DMA가 처리 중
            printf("[I2C] Transmitting register address: 0x%02X\n", reg_addresses[current_reg_index]);
            break;

        case I2C_STATE_TX_COMPLETE:
            // 레지스터 주소 송신 완료 - 이제 데이터 수신 시작
            printf("[I2C] Register address sent, starting data receive\n");
            i2c_state = I2C_STATE_IDLE;  // 상태 초기화 후 수신 시작

            if (HAL_I2C_Master_Receive_DMA(&hi2c1, MP5475GU_I2C_ADDRESS, &pmic_reg_data, 1) != HAL_OK) {
                printf("[I2C] Failed to start register data receive\n");
                i2c_state = I2C_STATE_ERROR;
            } else {
                i2c_state = I2C_STATE_RX_BUSY;
            }
            break;

        case I2C_STATE_RX_BUSY:
            // 레지스터 데이터 수신 중 - DMA가 처리 중
            printf("[I2C] Receiving register data from 0x%02X\n", reg_addresses[current_reg_index]);
            break;

        case I2C_STATE_RX_COMPLETE:
            // 레지스터 데이터 수신 완료 - 데이터 분석 및 다음 레지스터 처리
            printf("[I2C] Register 0x%02X data received: 0x%02X\n",
                   reg_addresses[current_reg_index], pmic_reg_data);

            // 현재 레지스터 데이터 처리
            process_pmic_register_data(reg_addresses[current_reg_index], pmic_reg_data);

            // 다음 레지스터로 이동
            current_reg_index++;

            if (current_reg_index < sizeof(reg_addresses)) {
                // 아직 읽을 레지스터가 남아있음 - 다음 레지스터 읽기
                i2c_state = I2C_STATE_IDLE;
                if (start_pmic_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
                    printf("[I2C] Failed to read next register\n");
                    i2c_state = I2C_STATE_ERROR;
                }
            } else {
                // 모든 레지스터 읽기 완료 - 최종 결과 출력
                printf("[I2C] All PMIC registers read completed\n");
                print_dtc_summary();
                i2c_state = I2C_STATE_IDLE;
            }
            break;

        case I2C_STATE_ERROR:
            // I2C 통신 에러 - 재시도 또는 에러 처리
            printf("[I2C] PMIC communication error - attempting recovery\n");

            // I2C 하드웨어 재초기화 시도
            HAL_I2C_DeInit(&hi2c1);
            HAL_Delay(10);
            MX_I2C1_Init();  // I2C 재초기화 함수 호출

            // 상태 초기화 후 재시도
            reset_pmic_diagnosis();
            i2c_state = I2C_STATE_IDLE;
            break;

        default:
            printf("[I2C] Unknown I2C state: %d\n", i2c_state);
            i2c_state = I2C_STATE_ERROR;
            break;
    }
}

/**
 * @brief PMIC 레지스터 읽기 시작 함수
 * @param reg_addr 읽을 레지스터 주소
 * @return HAL_StatusTypeDef 실행 결과
 * @note DMA를 사용하여 레지스터 주소를 먼저 송신
 */
static HAL_StatusTypeDef start_pmic_register_read(uint8_t reg_addr)
{
    if (i2c_state != I2C_STATE_IDLE) {
        printf("[I2C] I2C busy, cannot start new operation\n");
        return HAL_BUSY;
    }

    // 읽을 레지스터 주소 설정
    pmic_reg_address = reg_addr;
    pmic_reg_data = 0;  // 수신 버퍼 초기화

    printf("[I2C] Starting register read: 0x%02X\n", reg_addr);

    // 상태를 송신 진행 중으로 변경
    i2c_state = I2C_STATE_TX_BUSY;

    // DMA를 사용하여 레지스터 주소 송신 (Write 후 Read 방식)
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_DMA(&hi2c1, MP5475GU_I2C_ADDRESS,
                                                           &pmic_reg_address, 1);

    if (status != HAL_OK) {
        printf("[I2C] Failed to start register address transmission: %d\n", status);
        i2c_state = I2C_STATE_ERROR;
    }

    return status;
}

/**
 * @brief 읽은 PMIC 레지스터 데이터 처리 함수
 * @param reg_addr 레지스터 주소
 * @param reg_data 읽은 레지스터 데이터
 * @note 레지스터별로 적절한 분석 함수 호출
 */
static void process_pmic_register_data(uint8_t reg_addr, uint8_t reg_data)
{
    // 읽은 데이터를 배열에 저장 (인덱스별로)
    pmic_status_data[current_reg_index] = reg_data;

    printf("[I2C] Processing register 0x%02X data: 0x%02X\n", reg_addr, reg_data);

    // 레지스터 주소에 따른 데이터 분석
    switch (reg_addr) {
        case REG_PG_STATUS:
            analyze_power_good_status(reg_data);
            break;

        case REG_BUCK1_UV_STATUS:
            analyze_buck1_uv_status(reg_data);
            break;

        case REG_BUCK2_UV_STATUS:
            analyze_buck2_uv_status(reg_data);
            break;

        case REG_BUCK3_OV_STATUS:
            analyze_buck3_ov_status(reg_data);
            break;

        default:
            printf("[I2C] Unknown register address: 0x%02X\n", reg_addr);
            break;
    }
}

/**
 * @brief Power Good 상태 분석 함수
 * @param pg_data 0x06 레지스터에서 읽은 데이터
 * @note Buck1, Buck2, Buck3의 Power Good 상태 확인
 */
static void analyze_power_good_status(uint8_t pg_data)
{
    printf("[DIAG] Analyzing Power Good Status: 0x%02X\n", pg_data);

    // Buck1 Power Good 상태 확인
    if (!(pg_data & PG_BUCK1_MASK)) {
        printf("[DIAG] Buck1 Power Good FAIL detected\n");
        add_dtc_code(DTC_BUCK1_UV);  // PG 실패시 UV로 간주
    } else {
        printf("[DIAG] Buck1 Power Good OK\n");
    }

    // Buck2 Power Good 상태 확인
    if (!(pg_data & PG_BUCK2_MASK)) {
        printf("[DIAG] Buck2 Power Good FAIL detected\n");
        add_dtc_code(DTC_BUCK2_UV);  // PG 실패시 UV로 간주
    } else {
        printf("[DIAG] Buck2 Power Good OK\n");
    }

    // Buck3 Power Good 상태 확인
    if (!(pg_data & PG_BUCK3_MASK)) {
        printf("[DIAG] Buck3 Power Good FAIL detected\n");
        add_dtc_code(DTC_BUCK3_UV);  // PG 실패시 UV로 간주
    } else {
        printf("[DIAG] Buck3 Power Good OK\n");
    }
}

/**
 * @brief Buck1 Undervoltage 상태 분석 함수
 * @param uv_data 0x07 레지스터에서 읽은 데이터
 * @note Buck1의 undervoltage 고장 여부 확인
 */
static void analyze_buck1_uv_status(uint8_t uv_data)
{
    printf("[DIAG] Analyzing Buck1 UV Status: 0x%02X\n", uv_data);

    // Undervoltage 고장 비트 확인
    if (uv_data & UV_FAULT_MASK) {
        printf("[DIAG] Buck1 Undervoltage FAULT detected\n");
        add_dtc_code(DTC_BUCK1_UV);
    } else {
        printf("[DIAG] Buck1 Undervoltage status Normal\n");
    }

    // 추가 상태 비트들 분석 (필요시)
    if (uv_data & 0x40) {  // 예시: Warning 레벨
        printf("[DIAG] Buck1 UV Warning level\n");
    }
    if (uv_data & 0x20) {  // 예시: Threshold 근접
        printf("[DIAG] Buck1 approaching UV threshold\n");
    }
}

/**
 * @brief Buck2 Undervoltage 상태 분석 함수
 * @param uv_data 0x08 레지스터에서 읽은 데이터
 * @note Buck2의 undervoltage 고장 여부 확인
 */
static void analyze_buck2_uv_status(uint8_t uv_data)
{
    printf("[DIAG] Analyzing Buck2 UV Status: 0x%02X\n", uv_data);

    // Undervoltage 고장 비트 확인
    if (uv_data & UV_FAULT_MASK) {
        printf("[DIAG] Buck2 Undervoltage FAULT detected\n");
        add_dtc_code(DTC_BUCK2_UV);
    } else {
        printf("[DIAG] Buck2 Undervoltage status Normal\n");
    }

    // 추가 상태 비트들 분석
    if (uv_data & 0x40) {
        printf("[DIAG] Buck2 UV Warning level\n");
    }
    if (uv_data & 0x20) {
        printf("[DIAG] Buck2 approaching UV threshold\n");
    }
}

/**
 * @brief Buck3 Overvoltage 상태 분석 함수
 * @param ov_data 0x09 레지스터에서 읽은 데이터
 * @note Buck3의 overvoltage 고장 여부 확인
 */
static void analyze_buck3_ov_status(uint8_t ov_data)
{
    printf("[DIAG] Analyzing Buck3 OV Status: 0x%02X\n", ov_data);

    // Overvoltage 고장 비트 확인
    if (ov_data & OV_FAULT_MASK) {
        printf("[DIAG] Buck3 Overvoltage FAULT detected\n");
        add_dtc_code(DTC_BUCK3_OV);
    } else {
        printf("[DIAG] Buck3 Overvoltage status Normal\n");
    }

    // 추가 상태 비트들 분석
    if (ov_data & 0x40) {
        printf("[DIAG] Buck3 OV Warning level\n");
    }
    if (ov_data & 0x20) {
        printf("[DIAG] Buck3 approaching OV threshold\n");
    }

    // Buck3는 overvoltage만 체크하지만, undervoltage도 가능하다면
    if (ov_data & 0x10) {  // 예시: UV 비트가 있다면
        printf("[DIAG] Buck3 Undervoltage also detected\n");
        add_dtc_code(DTC_BUCK3_UV);
    }
}

/**
 * @brief DTC 코드 추가 함수
 * @param dtc_code 추가할 DTC 코드
 * @note 중복 DTC 방지 및 배열 관리
 */
static void add_dtc_code(uint16_t dtc_code)
{
    // 이미 등록된 DTC인지 확인 (중복 방지)
    for (uint8_t i = 0; i < dtc_count; i++) {
        if (detected_dtc_codes[i] == dtc_code) {
            printf("[DIAG] DTC 0x%04X already exists, skipping\n", dtc_code);
            return;
        }
    }

    // 배열 공간 확인 후 추가
    if (dtc_count < sizeof(detected_dtc_codes) / sizeof(detected_dtc_codes[0])) {
        detected_dtc_codes[dtc_count] = dtc_code;
        dtc_count++;
        printf("[DIAG] DTC 0x%04X added (total: %d)\n", dtc_code, dtc_count);
    } else {
        printf("[DIAG] DTC buffer full, cannot add 0x%04X\n", dtc_code);
    }
}

/**
 * @brief DTC 요약 출력 함수
 * @note 진단 완료 후 검출된 모든 DTC 코드 출력
 */
static void print_dtc_summary(void)
{
    printf("\n[DIAG] ===== PMIC Diagnosis Summary =====\n");
    printf("[DIAG] Total DTC codes detected: %d\n", dtc_count);

    if (dtc_count == 0) {
        printf("[DIAG] PMIC status: All systems NORMAL\n");
    } else {
        printf("[DIAG] PMIC status: FAULTS detected\n");

        for (uint8_t i = 0; i < dtc_count; i++) {
            printf("[DIAG] DTC[%d]: 0x%04X - ", i + 1, detected_dtc_codes[i]);

            // DTC 코드별 설명 출력
            switch (detected_dtc_codes[i]) {
                case DTC_BUCK1_UV:
                    printf("Buck1 Undervoltage Fault");
                    break;
                case DTC_BUCK1_OV:
                    printf("Buck1 Overvoltage Fault");
                    break;
                case DTC_BUCK2_UV:
                    printf("Buck2 Undervoltage Fault");
                    break;
                case DTC_BUCK2_OV:
                    printf("Buck2 Overvoltage Fault");
                    break;
                case DTC_BUCK3_UV:
                    printf("Buck3 Undervoltage Fault");
                    break;
                case DTC_BUCK3_OV:
                    printf("Buck3 Overvoltage Fault");
                    break;
                default:
                    printf("Unknown Fault");
                    break;
            }
            printf("\n");
        }
    }

    // 레지스터 원시 데이터 출력
    printf("[DIAG] Raw register data:\n");
    printf("[DIAG]   PG_STATUS (0x06): 0x%02X\n", pmic_status_data[0]);
    printf("[DIAG]   BUCK1_UV (0x07):  0x%02X\n", pmic_status_data[1]);
    printf("[DIAG]   BUCK2_UV (0x08):  0x%02X\n", pmic_status_data[2]);
    printf("[DIAG]   BUCK3_OV (0x09):  0x%02X\n", pmic_status_data[3]);
    printf("[DIAG] ================================\n\n");
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

    for (uint8_t i = 0; i < sizeof(detected_dtc_codes) / sizeof(detected_dtc_codes[0]); i++) {
        detected_dtc_codes[i] = 0;
    }

    // 작업 변수 초기화
    pmic_reg_address = 0;
    pmic_reg_data = 0;

    printf("[DIAG] PMIC diagnosis state reset completed\n");
}

/**
 * @brief PMIC DTC 코드 조회 함수
 * @return uint8_t 현재 검출된 DTC 개수
 * @note 외부에서 DTC 상태를 확인할 때 사용
 */
uint8_t get_pmic_dtc_count(void)
{
    return dtc_count;
}

/**
 * @brief 특정 DTC 코드 확인 함수
 * @param dtc_code 확인할 DTC 코드
 * @return bool DTC 존재 여부 (true: 존재, false: 없음)
 * @note 특정 고장 상태를 외부에서 확인할 때 사용
 */
bool is_dtc_present(uint16_t dtc_code)
{
    for (uint8_t i = 0; i < dtc_count; i++) {
        if (detected_dtc_codes[i] == dtc_code) {
            return true;
        }
    }
    return false;
}

/**
 * @brief PMIC 전체 상태 확인 함수
 * @return bool PMIC 정상 여부 (true: 정상, false: 고장)
 * @note 시스템 레벨에서 PMIC 상태를 빠르게 확인할 때 사용
 */
bool is_pmic_healthy(void)
{
    return (dtc_count == 0);
}
