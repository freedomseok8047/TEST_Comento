/**
 * @file pmic_service.h
 * @brief PMIC(MP5475GU) 서비스 모듈 - Union 기반 구현
 * @author Brake System Team
 * @date 2025-09-15
 * 
 * @note 멘토님 피드백 반영: Union을 사용한 레지스터 처리
 *       main_practice.c에서 PMIC 관련 기능 분리
 */

#ifndef PMIC_SERVICE_H
#define PMIC_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "common_types.h"
#include "stm32f4xx_hal.h"

// ========== PMIC 설정 상수 (main_practice.c에서 이동) ==========
#define MP5475_I2C_ADDRESS          0x60
#define MP5475_REG_READ_SIZE        1

// ========== PMIC 레지스터 주소 정의 (main_practice.c에서 이동) ==========
typedef enum {
    MP5475_REG_SYSTEM_STATUS = 0x05,
    MP5475_REG_POWER_GOOD = 0x06,
    MP5475_REG_UV_OV_FAULT = 0x07,
    MP5475_REG_OC_FAULT = 0x08,
    MP5475_REG_TEMP_FAULT = 0x09
} mp5475_register_t;

// ========== Union 기반 레지스터 비트필드 정의 ==========

/**
 * @brief 시스템 상태 레지스터 (0x05) Union
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t pmr_good_raw    : 1;    // [0] PMR Good Raw
        uint8_t pmr_good_filt   : 1;    // [1] PMR Good Filtered
        uint8_t fsm_state       : 2;    // [3:2] FSM State
        uint8_t reserved        : 4;    // [7:4] Reserved
    } bits;
} pmic_system_status_reg_t;

/**
 * @brief Power Good 레지스터 (0x06) Union
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t buckd_pg_raw    : 1;    // [0] Buck D Power Good Raw
        uint8_t buckc_pg_raw    : 1;    // [1] Buck C Power Good Raw
        uint8_t buckb_pg_raw    : 1;    // [2] Buck B Power Good Raw
        uint8_t bucka_pg_raw    : 1;    // [3] Buck A Power Good Raw
        uint8_t buckd_pg_filt   : 1;    // [4] Buck D Power Good Filtered
        uint8_t buckc_pg_filt   : 1;    // [5] Buck C Power Good Filtered
        uint8_t buckb_pg_filt   : 1;    // [6] Buck B Power Good Filtered
        uint8_t bucka_pg_filt   : 1;    // [7] Buck A Power Good Filtered
    } bits;
} pmic_power_good_reg_t;

/**
 * @brief UV/OV 고장 레지스터 (0x07) Union
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t bucka_ov        : 1;    // [0] Buck A Over Voltage
        uint8_t buckb_ov        : 1;    // [1] Buck B Over Voltage
        uint8_t buckc_ov        : 1;    // [2] Buck C Over Voltage
        uint8_t buckd_ov        : 1;    // [3] Buck D Over Voltage
        uint8_t bucka_uv        : 1;    // [4] Buck A Under Voltage
        uint8_t buckb_uv        : 1;    // [5] Buck B Under Voltage
        uint8_t buckc_uv        : 1;    // [6] Buck C Under Voltage
        uint8_t buckd_uv        : 1;    // [7] Buck D Under Voltage
    } bits;
} pmic_uv_ov_fault_reg_t;

/**
 * @brief OC 고장 레지스터 (0x08) Union
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t bucka_oc_warn   : 1;    // [0] Buck A OC Warning
        uint8_t buckb_oc_warn   : 1;    // [1] Buck B OC Warning
        uint8_t buckc_oc_warn   : 1;    // [2] Buck C OC Warning
        uint8_t buckd_oc_warn   : 1;    // [3] Buck D OC Warning
        uint8_t bucka_oc        : 1;    // [4] Buck A Over Current
        uint8_t buckb_oc        : 1;    // [5] Buck B Over Current
        uint8_t buckc_oc        : 1;    // [6] Buck C Over Current
        uint8_t buckd_oc        : 1;    // [7] Buck D Over Current
    } bits;
} pmic_oc_fault_reg_t;

/**
 * @brief 온도 고장 레지스터 (0x09) Union
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t pmic_temp_shutdown  : 1;    // [0] PMIC High Temperature Shutdown
        uint8_t pmic_temp_warning   : 1;    // [1] PMIC High Temperature Warning
        uint8_t vdrv_ov             : 1;    // [2] VDRV Over Voltage
        uint8_t vbulk_ov            : 1;    // [3] VBULK Over Voltage
        uint8_t vr_fault            : 1;    // [4] VR Fault
        uint8_t ldo_1v1_fault       : 1;    // [5] 1.1V LDO Fault
        uint8_t ldo_1v8_fault       : 1;    // [6] 1.8V LDO Fault
        uint8_t reserved            : 1;    // [7] Reserved
    } bits;
} pmic_temp_fault_reg_t;

// ========== PMIC 전체 상태 Union (8바이트 요구사항 반영) ==========

/**
 * @brief PMIC 전체 상태 데이터 Union (멘토님 요구사항)
 * @note Union과 비트필드를 활용하여 8바이트 데이터를 효율적으로 처리
 */
typedef union {
    // 8바이트 원시 데이터
    uint8_t raw_data[8];
    
    // 레지스터별 구조화된 접근
    struct {
        pmic_system_status_reg_t    system_status;      // 0x05
        pmic_power_good_reg_t       power_good;         // 0x06
        pmic_uv_ov_fault_reg_t      uv_ov_fault;        // 0x07
        pmic_oc_fault_reg_t         oc_fault;           // 0x08
        pmic_temp_fault_reg_t       temp_fault;         // 0x09
        uint8_t                     reserved[3];        // 예약된 바이트
    } regs;
    
    // 16비트 워드 단위 접근
    uint16_t word_data[4];
    
    // 32비트 더블워드 단위 접근
    uint32_t dword_data[2];
    
    // 64비트 전체 데이터
    uint64_t full_data;
    
} pmic_status_data_t;

// ========== PMIC 서비스 상태 ==========
typedef enum {
    PMIC_SERVICE_IDLE = 0,
    PMIC_SERVICE_READING,
    PMIC_SERVICE_COMPLETE,
    PMIC_SERVICE_ERROR
} pmic_service_state_t;

// ========== 공개 함수 선언 ==========

/**
 * @brief PMIC 서비스 초기화
 * @param hi2c I2C 핸들 포인터
 * @return true: 성공, false: 실패
 */
bool pmic_service_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief PMIC 진단 시작 (main_practice.c의 start_pmic_register_read 대체)
 * @return true: 성공, false: 실패
 */
bool pmic_service_start_diagnosis(void);

/**
 * @brief PMIC 상태 데이터 가져오기
 * @param status_data 상태 데이터 저장 포인터
 * @return true: 성공, false: 실패
 */
bool pmic_service_get_status(pmic_status_data_t *status_data);

/**
 * @brief PMIC 서비스 상태 조회
 * @return 현재 상태
 */
pmic_service_state_t pmic_service_get_state(void);

/**
 * @brief Power Good 상태 확인 (Union 비트필드 활용)
 * @param status_data PMIC 상태 데이터
 * @return true: 정상, false: 문제 있음
 */
bool pmic_service_is_power_good(const pmic_status_data_t *status_data);

/**
 * @brief PMIC 고장 분석 및 DTC 생성 (main_practice.c의 process_pmic_register_data 개선)
 * @param status_data PMIC 상태 데이터
 * @return 생성된 DTC 개수
 */
uint8_t pmic_service_analyze_faults(const pmic_status_data_t *status_data);

/**
 * @brief 개별 레지스터 값 출력 (디버깅용)
 * @param status_data PMIC 상태 데이터
 */
void pmic_service_print_registers(const pmic_status_data_t *status_data);

/**
 * @brief PMIC 상태 리셋
 */
void pmic_service_reset_state(void);

// ========== 콜백 함수 (main_practice.c에서 호출) ==========

/**
 * @brief I2C DMA 수신 완료 콜백 처리
 * @param reg_data 레지스터 데이터
 */
void pmic_service_register_received(uint8_t reg_data);

/**
 * @brief I2C DMA 에러 콜백 처리
 */
void pmic_service_communication_error(void);

#endif // PMIC_SERVICE_H