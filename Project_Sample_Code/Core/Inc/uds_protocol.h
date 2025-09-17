/**
 * @file uds_protocol.h
 * @brief UDS(Unified Diagnostic Services) 프로토콜 모듈
 * @author Brake System Team
 * @date 2025-09-15
 * 
 * @note ISO 14229 표준 기반 UDS 프로토콜 구현
 *       브레이크 시스템 진단용 DTC 관리
 */

#ifndef UDS_PROTOCOL_H
#define UDS_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "common_types.h"

// ========== UDS 서비스 ID 정의 (ISO 14229) ==========
typedef enum {
    UDS_SERVICE_DIAG_SESSION_CTRL   = 0x10,     // Diagnostic Session Control
    UDS_SERVICE_ECU_RESET           = 0x11,     // ECU Reset
    UDS_SERVICE_CLEAR_DTC           = 0x14,     // Clear DTC Information
    UDS_SERVICE_READ_DTC            = 0x19,     // Read DTC Information
    UDS_SERVICE_READ_DATA           = 0x22,     // Read Data By Identifier
    UDS_SERVICE_WRITE_DATA          = 0x2E,     // Write Data By Identifier
    UDS_SERVICE_ROUTINE_CTRL        = 0x31,     // Routine Control
    UDS_SERVICE_TESTER_PRESENT      = 0x3E      // Tester Present
} uds_service_id_t;

// ========== UDS 응답 코드 정의 ==========
typedef enum {
    UDS_RESPONSE_POSITIVE           = 0x40,     // 긍정 응답 오프셋 (+0x40)
    UDS_NRC_GENERAL_REJECT          = 0x10,     // General Reject
    UDS_NRC_SERVICE_NOT_SUPPORTED   = 0x11,     // Service Not Supported
    UDS_NRC_SUBFUNCTION_NOT_SUPPORTED = 0x12,   // Sub-function Not Supported
    UDS_NRC_REQUEST_OUT_OF_RANGE    = 0x31,     // Request Out Of Range
    UDS_NRC_CONDITIONS_NOT_CORRECT  = 0x22      // Conditions Not Correct
} uds_response_code_t;

// ========== UDS DTC 관련 서브펑션 ==========
typedef enum {
    UDS_DTC_REPORT_NUMBER_BY_STATUS = 0x01,     // Report Number Of DTC By Status Mask
    UDS_DTC_REPORT_BY_STATUS        = 0x02,     // Report DTC By Status Mask
    UDS_DTC_REPORT_SNAPSHOT_ID      = 0x03,     // Report DTC Snapshot Identification
    UDS_DTC_REPORT_SNAPSHOT_RECORD  = 0x04,     // Report DTC Snapshot Record By DTC Number
    UDS_DTC_REPORT_EXTENDED_DATA    = 0x06      // Report DTC Extended Data Record By DTC Number
} uds_dtc_subfunction_t;

// ========== UDS 진단 세션 타입 ==========
typedef enum {
    UDS_SESSION_DEFAULT             = 0x01,     // Default Session
    UDS_SESSION_PROGRAMMING         = 0x02,     // Programming Session
    UDS_SESSION_EXTENDED            = 0x03,     // Extended Diagnostic Session
    UDS_SESSION_SAFETY_SYSTEM       = 0x04      // Safety System Diagnostic Session
} uds_session_type_t;

// ========== DTC 상태 마스크 (ISO 14229-1) ==========
typedef enum {
    UDS_DTC_STATUS_TEST_FAILED              = 0x01,    // Test Failed
    UDS_DTC_STATUS_TEST_FAILED_THIS_OP_CYCLE = 0x02,   // Test Failed This Operation Cycle
    UDS_DTC_STATUS_PENDING                  = 0x04,    // Pending DTC
    UDS_DTC_STATUS_CONFIRMED                = 0x08,    // Confirmed DTC
    UDS_DTC_STATUS_TEST_NOT_COMPLETED_SINCE_LAST_CLEAR = 0x10, // Test Not Completed Since Last Clear
    UDS_DTC_STATUS_TEST_FAILED_SINCE_LAST_CLEAR = 0x20,        // Test Failed Since Last Clear
    UDS_DTC_STATUS_TEST_NOT_COMPLETED_THIS_OP_CYCLE = 0x40,    // Test Not Completed This Operation Cycle
    UDS_DTC_STATUS_WARNING_INDICATOR_REQUESTED = 0x80          // Warning Indicator Requested
} uds_dtc_status_mask_t;

// ========== UDS 메시지 구조체 ==========
#define UDS_MAX_MESSAGE_SIZE    8       // CAN 프레임 최대 크기
#define UDS_MAX_RESPONSE_SIZE   256     // 다중 프레임 응답 지원

/**
 * @brief UDS 요청 메시지 구조체
 */
typedef struct {
    uds_service_id_t    service_id;                     // 서비스 ID
    uint8_t             subfunction;                    // 서브펑션 (옵션)
    uint8_t             data[UDS_MAX_MESSAGE_SIZE-2];   // 요청 데이터
    uint8_t             data_length;                    // 데이터 길이
} uds_request_t;

/**
 * @brief UDS 응답 메시지 구조체
 */
typedef struct {
    uint8_t             service_id;                     // 응답 서비스 ID (요청+0x40)
    uint8_t             subfunction;                    // 서브펑션 (옵션)
    uint8_t             data[UDS_MAX_RESPONSE_SIZE-2];  // 응답 데이터
    uint16_t            data_length;                    // 데이터 길이
    bool                is_negative;                    // 부정 응답 여부
    uint8_t             nrc;                           // Negative Response Code
} uds_response_t;

// ========== 브레이크 시스템 DID (Data Identifier) ==========
typedef enum {
    UDS_DID_BRAKE_SYSTEM_STATUS     = 0xF001,          // 브레이크 시스템 상태
    UDS_DID_BRAKE_PRESSURE          = 0xF002,          // 브레이크 압력
    UDS_DID_PMIC_STATUS             = 0xF003,          // PMIC 상태
    UDS_DID_DTC_COUNT               = 0xF004,          // DTC 개수
    UDS_DID_SYSTEM_UPTIME           = 0xF005,          // 시스템 가동 시간
    UDS_DID_SOFTWARE_VERSION        = 0xF010,          // 소프트웨어 버전
    UDS_DID_HARDWARE_VERSION        = 0xF011           // 하드웨어 버전
} uds_brake_did_t;

// ========== UDS 서비스 상태 ==========
typedef enum {
    UDS_STATE_IDLE = 0,
    UDS_STATE_PROCESSING,
    UDS_STATE_SENDING_RESPONSE,
    UDS_STATE_ERROR
} uds_state_t;

// ========== 공개 함수 선언 ==========

/**
 * @brief UDS 프로토콜 초기화
 * @return true: 성공, false: 실패
 */
bool uds_protocol_init(void);

/**
 * @brief UDS 요청 메시지 처리
 * @param request 요청 메시지
 * @param response 응답 메시지 (출력)
 * @return true: 성공, false: 실패
 */
bool uds_process_request(const uds_request_t *request, uds_response_t *response);

/**
 * @brief CAN 메시지를 UDS 요청으로 파싱
 * @param can_data CAN 데이터 (8바이트)
 * @param can_length CAN 데이터 길이
 * @param request UDS 요청 구조체 (출력)
 * @return true: 성공, false: 실패
 */
bool uds_parse_can_message(const uint8_t *can_data, uint8_t can_length, uds_request_t *request);

/**
 * @brief UDS 응답을 CAN 메시지로 변환
 * @param response UDS 응답 구조체
 * @param can_data CAN 데이터 버퍼 (출력)
 * @param can_length CAN 데이터 길이 (출력)
 * @return true: 성공, false: 실패
 */
bool uds_format_can_response(const uds_response_t *response, uint8_t *can_data, uint8_t *can_length);

// ========== 개별 서비스 처리 함수들 ==========

/**
 * @brief Service 0x10 - 진단 세션 제어
 * @param session_type 세션 타입
 * @param response 응답 구조체
 * @return true: 성공, false: 실패
 */
bool uds_service_diagnostic_session_control(uint8_t session_type, uds_response_t *response);

/**
 * @brief Service 0x14 - DTC 클리어
 * @param dtc_group DTC 그룹 (3바이트)
 * @param response 응답 구조체
 * @return true: 성공, false: 실패
 */
bool uds_service_clear_dtc(const uint8_t *dtc_group, uds_response_t *response);

/**
 * @brief Service 0x19 - DTC 읽기
 * @param subfunction 서브펑션
 * @param status_mask 상태 마스크
 * @param response 응답 구조체
 * @return true: 성공, false: 실패
 */
bool uds_service_read_dtc(uint8_t subfunction, uint8_t status_mask, uds_response_t *response);

/**
 * @brief Service 0x22 - 데이터 읽기
 * @param did Data Identifier
 * @param response 응답 구조체
 * @return true: 성공, false: 실패
 */
bool uds_service_read_data_by_identifier(uint16_t did, uds_response_t *response);

/**
 * @brief Service 0x3E - Tester Present
 * @param response 응답 구조체
 * @return true: 성공, false: 실패
 */
bool uds_service_tester_present(uds_response_t *response);

// ========== 유틸리티 함수들 ==========

/**
 * @brief 부정 응답 생성
 * @param service_id 서비스 ID
 * @param nrc Negative Response Code
 * @param response 응답 구조체 (출력)
 */
void uds_create_negative_response(uint8_t service_id, uint8_t nrc, uds_response_t *response);

/**
 * @brief UDS 상태 조회
 * @return 현재 UDS 상태
 */
uds_state_t uds_get_state(void);

/**
 * @brief UDS 통계 정보 출력
 */
void uds_print_statistics(void);

/**
 * @brief 브레이크 DTC를 UDS 포맷으로 변환
 * @param dtc_code 브레이크 DTC 코드
 * @return UDS DTC 포맷 (3바이트를 uint32_t로)
 */
uint32_t uds_convert_brake_dtc_to_uds_format(uint16_t dtc_code);

#endif // UDS_PROTOCOL_H