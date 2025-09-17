/**
 * @file dtc_manager.h
 * @brief DTC(Diagnostic Trouble Code) 관리 모듈
 * @author Brake System Team
 * @date 2025-09-15
 * 
 * @note UDS 프로토콜 기반 DTC 관리
 *       main_practice.c에서 DTC 관련 기능 분리
 */

#ifndef DTC_MANAGER_H
#define DTC_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "common_types.h"

// ========== DTC 코드 정의 (main_practice.c에서 이동) ==========
typedef enum {
    DTC_BRAKE_PMIC_UV     = 0xC001,  // Under Voltage
    DTC_BRAKE_PMIC_OV     = 0xC002,  // Over Voltage  
    DTC_BRAKE_PMIC_OC     = 0xC003,  // Over Current
    DTC_BRAKE_PMIC_TEMP   = 0xC004,  // Temperature Fault
    DTC_BRAKE_COMM_ERROR  = 0xC005,  // Communication Error
    DTC_BRAKE_SYSTEM_FAULT = 0xC006  // System/Power Good Fault
} brake_dtc_code_t;

// ========== DTC 상태 정의 (UDS 기반) ==========
typedef enum {
    DTC_STATUS_INACTIVE           = 0x00,
    DTC_STATUS_ACTIVE             = 0x01,
    DTC_STATUS_PENDING            = 0x02,
    DTC_STATUS_CONFIRMED          = 0x04,
    DTC_STATUS_TEST_FAILED        = 0x08,
    DTC_STATUS_WARNING_INDICATOR  = 0x10
} dtc_status_t;

// ========== DTC 테이블 구조체 (main_practice.c에서 개선) ==========
#pragma pack(push, 1)
typedef struct {
    uint16_t DTC_Code;              // DTC 코드
    char Description[50];           // 설명 문자열
    uint8_t active;                 // 활성화 상태
    uint8_t occurrence_count;       // 발생 횟수 (추가)
    dtc_status_t status;            // UDS 상태 (추가)
    uint32_t first_occurrence;      // 최초 발생 시간 (추가)
    uint32_t last_occurrence;       // 최근 발생 시간 (추가)
} DTC_Table_t;
#pragma pack(pop)

// ========== DTC 관리 설정 ==========
#define DTC_MAX_COUNT           6       // 최대 DTC 개수
#define DTC_DESCRIPTION_SIZE    50      // 설명 최대 길이

// ========== 전역 함수 선언 ==========

/**
 * @brief DTC 매니저 초기화
 * @return true: 성공, false: 실패
 */
bool dtc_manager_init(void);

/**
 * @brief DTC 추가 (main_practice.c의 add_dtc_code 대체)
 * @param dtc_code 추가할 DTC 코드
 * @return true: 성공, false: 실패/중복
 */
bool dtc_add_code(uint16_t dtc_code);

/**
 * @brief 모든 DTC 클리어 (UDS Service 0x14)
 * @return true: 성공, false: 실패
 */
bool dtc_clear_all(void);

/**
 * @brief 특정 DTC 클리어
 * @param dtc_code 클리어할 DTC 코드
 * @return true: 성공, false: 실패
 */
bool dtc_clear_specific(uint16_t dtc_code);

/**
 * @brief 활성 DTC 개수 조회
 * @return 활성 DTC 개수
 */
uint8_t dtc_get_active_count(void);

/**
 * @brief DTC 목록 조회
 * @param dtc_list DTC 목록 저장 배열
 * @param max_count 최대 개수
 * @return 실제 복사된 DTC 개수
 */
uint8_t dtc_get_list(DTC_Table_t* dtc_list, uint8_t max_count);

/**
 * @brief DTC 요약 출력 (main_practice.c의 print_dtc_summary 대체)
 */
void dtc_print_summary(void);

/**
 * @brief 진단 상태 리셋 (main_practice.c의 reset_pmic_diagnosis 일부)
 */
void dtc_reset_diagnosis(void);

/**
 * @brief DTC 상태 업데이트
 * @param dtc_code DTC 코드
 * @param new_status 새로운 상태
 */
void dtc_update_status(uint16_t dtc_code, dtc_status_t new_status);

/**
 * @brief UDS 포맷으로 DTC 데이터 생성
 * @param buffer 출력 버퍼
 * @param buffer_size 버퍼 크기
 * @return 생성된 데이터 크기
 */
uint16_t dtc_create_uds_response(uint8_t* buffer, uint16_t buffer_size);

#endif // DTC_MANAGER_H