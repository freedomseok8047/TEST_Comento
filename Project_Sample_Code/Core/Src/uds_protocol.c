/**
 * @file uds_protocol.c
 * @brief UDS 프로토콜 모듈 구현
 * @note ISO 14229 표준 기반 구현
 */

#include "uds_protocol.h"
#include "dtc_manager.h"
#include "pmic_service.h"
#include "stm32f4xx_hal.h"  // HAL_GetTick() 사용을 위해 추가
#include "eeprom_service.h"
#include <string.h>
#include <stdio.h>


// ========== 내부 변수 ==========
static uds_state_t uds_current_state = UDS_STATE_IDLE;
static uds_session_type_t current_session = UDS_SESSION_DEFAULT;

// UDS 통계 정보
static struct {
    uint32_t total_requests;
    uint32_t successful_responses;
    uint32_t negative_responses;
    uint32_t service_14_count;      // Clear DTC 호출 횟수
    uint32_t service_19_count;      // Read DTC 호출 횟수
    uint32_t service_22_count;      // Read Data 호출 횟수
} uds_statistics = {0};

// ========== 내부 함수 선언 ==========
static bool uds_validate_request(const uds_request_t *request);
static void uds_update_statistics(const uds_request_t *request, const uds_response_t *response);

// ========== 공개 함수 구현 ==========

/**
 * @brief UDS 프로토콜 초기화
 */
bool uds_protocol_init(void)
{
    printf("[UDS] UDS Protocol initialization started\n");
    
    uds_current_state = UDS_STATE_IDLE;
    current_session = UDS_SESSION_DEFAULT;
    
    // 통계 초기화
    memset(&uds_statistics, 0, sizeof(uds_statistics));
    
    printf("[UDS] UDS Protocol initialized successfully\n");
    printf("[UDS] Default session activated\n");
    return true;
}

/**
 * @brief UDS 요청 메시지 처리 (메인 엔트리 포인트)
 */
bool uds_process_request(const uds_request_t *request, uds_response_t *response)
{
    if (request == NULL || response == NULL) {
        return false;
    }
    
    printf("[UDS] Processing request - Service: 0x%02X\n", request->service_id);
    
    uds_current_state = UDS_STATE_PROCESSING;
    uds_statistics.total_requests++;
    
    // 요청 유효성 검사
    if (!uds_validate_request(request)) {
        uds_create_negative_response(request->service_id, UDS_NRC_REQUEST_OUT_OF_RANGE, response);
        uds_update_statistics(request, response);
        uds_current_state = UDS_STATE_IDLE;
        return false;
    }
    
    // 서비스별 처리
    bool result = false;
    
    switch (request->service_id) {
        case UDS_SERVICE_DIAG_SESSION_CTRL:     // 0x10 - 진단 세션 시작
            result = uds_service_diagnostic_session_control(request->subfunction, response);
            break;
            
        case UDS_SERVICE_CLEAR_DTC:             // 0x14 - 모든 DTC 클리어
            result = uds_service_clear_dtc(request->data, response);
            uds_statistics.service_14_count++;
            break;
            
        case UDS_SERVICE_READ_DTC:              // 0x19 - DTC 목록 조회
            result = uds_service_read_dtc(request->subfunction, 
                                        request->data_length > 0 ? request->data[0] : 0xFF, 
                                        response);
            uds_statistics.service_19_count++;
            break;
            
        case UDS_SERVICE_READ_DATA:             // 0x22 - 브레이크 상태 데이터
            if (request->data_length >= 2) {
                uint16_t did = (request->data[0] << 8) | request->data[1];
                result = uds_service_read_data_by_identifier(did, response);
                uds_statistics.service_22_count++;
            } else {
                uds_create_negative_response(request->service_id, UDS_NRC_REQUEST_OUT_OF_RANGE, response);
            }
            break;
            
        case UDS_SERVICE_TESTER_PRESENT:        // 0x3E
            result = uds_service_tester_present(response);
            break;
            
        default:
            printf("[UDS] Unsupported service: 0x%02X\n", request->service_id);
            uds_create_negative_response(request->service_id, UDS_NRC_SERVICE_NOT_SUPPORTED, response);
            break;
    }
    
    uds_update_statistics(request, response);
    uds_current_state = UDS_STATE_IDLE;
    
    return result;
}

/**
 * @brief CAN 메시지를 UDS 요청으로 파싱
 */
bool uds_parse_can_message(const uint8_t *can_data, uint8_t can_length, uds_request_t *request)
{
    if (can_data == NULL || request == NULL || can_length == 0) {
        return false;
    }
    
    // 최소 1바이트 (Service ID) 필요
    if (can_length < 1) {
        printf("[UDS] Invalid CAN message length: %d\n", can_length);
        return false;
    }
    
    // UDS 요청 구조체 초기화
    memset(request, 0, sizeof(uds_request_t));
    
    // Service ID 파싱
    request->service_id = can_data[0];
    
    // 서브펑션이 있는 서비스들
    if (can_length > 1 && 
        (request->service_id == UDS_SERVICE_DIAG_SESSION_CTRL ||
         request->service_id == UDS_SERVICE_READ_DTC ||
         request->service_id == UDS_SERVICE_TESTER_PRESENT)) {
        request->subfunction = can_data[1];
        
        // 나머지 데이터 복사
        if (can_length > 2) {
            request->data_length = can_length - 2;
            memcpy(request->data, &can_data[2], request->data_length);
        }
    } else {
        // 서브펑션 없음 - 나머지 모든 데이터
        if (can_length > 1) {
            request->data_length = can_length - 1;
            memcpy(request->data, &can_data[1], request->data_length);
        }
    }
    
    printf("[UDS] Parsed CAN message - Service: 0x%02X, Sub: 0x%02X, DataLen: %d\n",
           request->service_id, request->subfunction, request->data_length);
    
    return true;
}

/**
 * @brief UDS 응답을 CAN 메시지로 변환
 */
bool uds_format_can_response(const uds_response_t *response, uint8_t *can_data, uint8_t *can_length)
{
    if (response == NULL || can_data == NULL || can_length == NULL) {
        return false;
    }
    
    uint8_t offset = 0;
    
    if (response->is_negative) {
        // 부정 응답: [0x7F] [Service ID] [NRC]
        can_data[offset++] = 0x7F;
        can_data[offset++] = response->service_id;
        can_data[offset++] = response->nrc;
    } else {
        // 긍정 응답: [Service ID + 0x40] [Sub] [Data...]
        can_data[offset++] = response->service_id;
        
        if (response->subfunction != 0) {
            can_data[offset++] = response->subfunction;
        }
        
        // 데이터 복사 (CAN 프레임 크기 제한 고려)
        uint16_t copy_length = response->data_length;
        if (offset + copy_length > UDS_MAX_MESSAGE_SIZE) {
            copy_length = UDS_MAX_MESSAGE_SIZE - offset;
            printf("[UDS] Response truncated to fit CAN frame\n");
        }
        
        if (copy_length > 0) {
            memcpy(&can_data[offset], response->data, copy_length);
            offset += copy_length;
        }
    }
    
    *can_length = offset;
    
    printf("[UDS] Formatted CAN response - Length: %d\n", *can_length);
    return true;
}

// ========== 개별 서비스 구현 ==========

/**
 * @brief Service 0x10 - 진단 세션 제어
 */
bool uds_service_diagnostic_session_control(uint8_t session_type, uds_response_t *response)
{
    printf("[UDS] Service 0x10 - Diagnostic Session Control: %d\n", session_type);
    
    // 응답 구조체 초기화
    memset(response, 0, sizeof(uds_response_t));
    
    // 세션 타입 유효성 검사
    if (session_type < UDS_SESSION_DEFAULT || session_type > UDS_SESSION_SAFETY_SYSTEM) {
        uds_create_negative_response(UDS_SERVICE_DIAG_SESSION_CTRL, 
                                   UDS_NRC_REQUEST_OUT_OF_RANGE, response);
        return false;
    }
    
    // 현재 세션 업데이트
    current_session = (uds_session_type_t)session_type;
    
    // 긍정 응답 생성
    response->service_id = UDS_SERVICE_DIAG_SESSION_CTRL + UDS_RESPONSE_POSITIVE;
    response->subfunction = session_type;
    response->is_negative = false;
    
    // 세션 매개변수 (예: P2 타이밍)
    response->data[0] = 0x00;  // P2 Server Max (High)
    response->data[1] = 0x32;  // P2 Server Max (Low) = 50ms
    response->data[2] = 0x01;  // P2* Server Max (High)  
    response->data[3] = 0xF4;  // P2* Server Max (Low) = 500ms
    response->data_length = 4;
    
    printf("[UDS] Session changed to: %d\n", session_type);
    return true;
}

/**
 * @brief Service 0x14 - DTC 클리어
 */
bool uds_service_clear_dtc(const uint8_t *dtc_group, uds_response_t *response)
{
    printf("[UDS] Service 0x14 - Clear DTC\n");
    
    // 응답 구조체 초기화
    memset(response, 0, sizeof(uds_response_t));
    
    // DTC 그룹 확인 (일반적으로 3바이트: 0xFFFFFF = 모든 DTC)
    if (dtc_group != NULL) {
        printf("[UDS] DTC Group: 0x%02X%02X%02X\n", 
               dtc_group[0], dtc_group[1], dtc_group[2]);
        
        // 0xFFFFFF = 모든 DTC 클리어
        if (dtc_group[0] == 0xFF && dtc_group[1] == 0xFF && dtc_group[2] == 0xFF) {
            if (dtc_clear_all()) {
                printf("[UDS] All DTCs cleared successfully\n");
            } else {
                uds_create_negative_response(UDS_SERVICE_CLEAR_DTC, 
                                           UDS_NRC_CONDITIONS_NOT_CORRECT, response);
                return false;
            }
        } else {
            printf("[UDS] Specific DTC group clear not implemented\n");
            uds_create_negative_response(UDS_SERVICE_CLEAR_DTC, 
                                       UDS_NRC_REQUEST_OUT_OF_RANGE, response);
            return false;
        }
    }
    
    // 긍정 응답 생성
    response->service_id = UDS_SERVICE_CLEAR_DTC + UDS_RESPONSE_POSITIVE;
    response->is_negative = false;
    response->data_length = 0;  // Clear DTC는 데이터 없음
    
    return true;
}

/**
 * @brief Service 0x19 - DTC 읽기
 */
bool uds_service_read_dtc(uint8_t subfunction, uint8_t status_mask, uds_response_t *response)
{
    printf("[UDS] Service 0x19 - Read DTC: Sub=0x%02X, Mask=0x%02X\n", subfunction, status_mask);
    
    // 응답 구조체 초기화
    memset(response, 0, sizeof(uds_response_t));
    
    switch (subfunction) {
        case UDS_DTC_REPORT_NUMBER_BY_STATUS:   // 0x01
        {
            uint8_t active_count = dtc_get_active_count();
            
            response->service_id = UDS_SERVICE_READ_DTC + UDS_RESPONSE_POSITIVE;
            response->subfunction = subfunction;
            response->data[0] = UDS_DTC_STATUS_TEST_FAILED;  // Status Availability Mask
            response->data[1] = 0x11;  // DTC Format Identifier (ISO14229 format)
            response->data[2] = 0x00;  // DTC Count High
            response->data[3] = active_count;  // DTC Count Low
            response->data_length = 4;
            response->is_negative = false;
            
            printf("[UDS] Reporting DTC count: %d\n", active_count);
            break;
        }
        
        case UDS_DTC_REPORT_BY_STATUS:          // 0x02
        {
            response->service_id = UDS_SERVICE_READ_DTC + UDS_RESPONSE_POSITIVE;
            response->subfunction = subfunction;
            response->data[0] = UDS_DTC_STATUS_TEST_FAILED;  // Status Availability Mask
            
            // ❌ 20250930 수정
            // ✅ EEPROM에서 DTC 읽기
            eeprom_dtc_log_t eeprom_logs[6];  // 변수 선언 추가!
            uint8_t dtc_count = eeprom_service_read_all_dtc(eeprom_logs, 6);
            
            uint8_t offset = 1;
            for (uint8_t i = 0; i < dtc_count && offset < (UDS_MAX_MESSAGE_SIZE - 3); i++) {
                // ✅ eeprom_dtc_log_t 구조체 사용
                if (eeprom_logs[i].active) {  // dtc_list → eeprom_logs
                    // UDS DTC 포맷 변환 (3바이트)
                    uint32_t uds_dtc = uds_convert_brake_dtc_to_uds_format(eeprom_logs[i].DTC_Code);
                    
                    response->data[offset++] = (uds_dtc >> 16) & 0xFF;  // DTC High
                    response->data[offset++] = (uds_dtc >> 8) & 0xFF;   // DTC Mid
                    response->data[offset++] = uds_dtc & 0xFF;          // DTC Low
                    response->data[offset++] = eeprom_logs[i].status;   // DTC Status
                }
            }
            
            response->data_length = offset;
            response->is_negative = false;
            
            printf("[UDS] Reporting %d active DTCs from EEPROM\n", dtc_count);
            break;
    }
        
        default:
            printf("[UDS] Unsupported DTC subfunction: 0x%02X\n", subfunction);
            uds_create_negative_response(UDS_SERVICE_READ_DTC, 
                                       UDS_NRC_SUBFUNCTION_NOT_SUPPORTED, response);
            return false;
    }
    
    return true;
}

/**
 * @brief Service 0x22 - 데이터 읽기
 */
bool uds_service_read_data_by_identifier(uint16_t did, uds_response_t *response)
{
    printf("[UDS] Service 0x22 - Read Data By Identifier: 0x%04X\n", did);
    
    // 응답 구조체 초기화
    memset(response, 0, sizeof(uds_response_t));
    
    response->service_id = UDS_SERVICE_READ_DATA + UDS_RESPONSE_POSITIVE;
    response->data[0] = (did >> 8) & 0xFF;  // DID High
    response->data[1] = did & 0xFF;         // DID Low
    
    uint8_t offset = 2;
    
    switch (did) {
        case UDS_DID_BRAKE_SYSTEM_STATUS:   // 0xF001
        {
            // 브레이크 시스템 전체 상태
            uint8_t system_status = 0x01;  // 0x01 = Normal, 0x02 = Warning, 0x03 = Error
            
            if (dtc_get_active_count() > 0) {
                system_status = 0x03;  // Error
            }
            
            response->data[offset++] = system_status;
            response->data[offset++] = dtc_get_active_count();  // Active DTC count
            
            printf("[UDS] Brake system status: %d, DTCs: %d\n", system_status, dtc_get_active_count());
            break;
        }
        
        case UDS_DID_BRAKE_PRESSURE:        // 0xF002
        {
            // 브레이크 압력 (시뮬레이션)
            uint16_t pressure = 850;  // 단위: kPa
            response->data[offset++] = (pressure >> 8) & 0xFF;
            response->data[offset++] = pressure & 0xFF;
            
            printf("[UDS] Brake pressure: %d kPa\n", pressure);
            break;
        }
        
        case UDS_DID_PMIC_STATUS:           // 0xF003
        {
            // PMIC 상태 데이터
            pmic_status_data_t pmic_status;
            if (pmic_service_get_status(&pmic_status)) {
                // PMIC 상태 요약 (4바이트)
                response->data[offset++] = pmic_status.regs.system_status.raw;
                response->data[offset++] = pmic_status.regs.power_good.raw;
                response->data[offset++] = pmic_status.regs.uv_ov_fault.raw;
                response->data[offset++] = pmic_status.regs.temp_fault.raw;
                
                printf("[UDS] PMIC status retrieved\n");
            } else {
                response->data[offset++] = 0xFF;  // Error indicator
                printf("[UDS] PMIC status not available\n");
            }
            break;
        }
        
        case UDS_DID_DTC_COUNT:             // 0xF004
        {
            // DTC 개수 정보
            response->data[offset++] = dtc_get_active_count();   // Active
            response->data[offset++] = 0;                        // Pending (구현 예정)
            response->data[offset++] = 0;                        // Confirmed (구현 예정)
            
            printf("[UDS] DTC counts - Active: %d\n", dtc_get_active_count());
            break;
        }
        
        case UDS_DID_SYSTEM_UPTIME:         // 0xF005
        {
            // 시스템 가동 시간 (초)
            uint32_t uptime = HAL_GetTick() / 1000;
            response->data[offset++] = (uptime >> 24) & 0xFF;
            response->data[offset++] = (uptime >> 16) & 0xFF;
            response->data[offset++] = (uptime >> 8) & 0xFF;
            response->data[offset++] = uptime & 0xFF;
            
            printf("[UDS] System uptime: %lu seconds\n", uptime);
            break;
        }
        
        case UDS_DID_SOFTWARE_VERSION:      // 0xF010
        {
            // 소프트웨어 버전 정보
            const char* sw_version = "V1.2.3";
            uint8_t len = strlen(sw_version);
            if (len > 6) len = 6;  // 안전성 체크
            strncpy((char*)&response->data[offset], sw_version, len);
            offset += len;
            
            printf("[UDS] Software version: %s\n", sw_version);
            break;
        }
        
        case UDS_DID_HARDWARE_VERSION:      // 0xF011
        {
            // 하드웨어 버전 정보
            const char* hw_version = "HW2.1";
            uint8_t len = strlen(hw_version);
            if (len > 5) len = 5;  // 안전성 체크
            strncpy((char*)&response->data[offset], hw_version, len);
            offset += len;
            
            printf("[UDS] Hardware version: %s\n", hw_version);
            break;
        }
        
        default:
            printf("[UDS] Unsupported DID: 0x%04X\n", did);
            uds_create_negative_response(UDS_SERVICE_READ_DATA, 
                                       UDS_NRC_REQUEST_OUT_OF_RANGE, response);
            return false;
    }
    
    response->data_length = offset;
    response->is_negative = false;
    
    return true;
}

/**
 * @brief Service 0x3E - Tester Present
 */
bool uds_service_tester_present(uds_response_t *response)
{
    printf("[UDS] Service 0x3E - Tester Present\n");
    
    // 응답 구조체 초기화
    memset(response, 0, sizeof(uds_response_t));
    
    // 긍정 응답 생성
    response->service_id = UDS_SERVICE_TESTER_PRESENT + UDS_RESPONSE_POSITIVE;
    response->subfunction = 0x00;  // Zero SubFunction
    response->is_negative = false;
    response->data_length = 0;
    
    printf("[UDS] Tester Present acknowledged\n");
    return true;
}

// ========== 유틸리티 함수 구현 ==========

/**
 * @brief 부정 응답 생성
 */
void uds_create_negative_response(uint8_t service_id, uint8_t nrc, uds_response_t *response)
{
    if (response == NULL) {
        return;
    }
    
    memset(response, 0, sizeof(uds_response_t));
    
    response->service_id = service_id;
    response->nrc = nrc;
    response->is_negative = true;
    response->data_length = 0;
    
    uds_statistics.negative_responses++;
    
    printf("[UDS] Negative response: Service=0x%02X, NRC=0x%02X\n", service_id, nrc);
}

/**
 * @brief UDS 상태 조회
 */
uds_state_t uds_get_state(void)
{
    return uds_current_state;
}

/**
 * @brief UDS 통계 정보 출력
 */
void uds_print_statistics(void)
{
    printf("\n========= UDS Statistics =========\n");
    printf("Total Requests:      %lu\n", uds_statistics.total_requests);
    printf("Successful Responses: %lu\n", uds_statistics.successful_responses);
    printf("Negative Responses:   %lu\n", uds_statistics.negative_responses);
    printf("Service 0x14 (Clear): %lu\n", uds_statistics.service_14_count);
    printf("Service 0x19 (Read):  %lu\n", uds_statistics.service_19_count);
    printf("Service 0x22 (Data):  %lu\n", uds_statistics.service_22_count);
    printf("Current Session:      %d\n", current_session);
    printf("==================================\n\n");
}

/**
 * @brief 브레이크 DTC를 UDS 포맷으로 변환
 */
uint32_t uds_convert_brake_dtc_to_uds_format(uint16_t dtc_code)
{
    // 브레이크 DTC (0xC001) → UDS 3바이트 포맷 (0x0C0001)
    // UDS DTC Format: [High Byte][Mid Byte][Low Byte]
    // High: System (0x0C = Body/Chassis)
    // Mid+Low: Specific fault code
    
    uint32_t uds_dtc = 0;
    
    switch (dtc_code & 0xFF00) {  // 상위 바이트로 카테고리 구분
        case 0xC000:  // 브레이크 시스템 DTC
            uds_dtc = 0x0C0000 | (dtc_code & 0x00FF);
            break;
            
        default:
            uds_dtc = 0x0F0000 | dtc_code;  // 0x0F = Reserved/Other
            break;
    }
    
    return uds_dtc;
}

// ========== 내부 함수 구현 ==========

/**
 * @brief UDS 요청 유효성 검사
 */
static bool uds_validate_request(const uds_request_t *request)
{
    if (request == NULL) {
        return false;
    }
    
    // 서비스별 데이터 길이 검사
    switch (request->service_id) {
        case UDS_SERVICE_CLEAR_DTC:
            if (request->data_length < 3) {  // DTC Group 3바이트 필요
                printf("[UDS] Clear DTC: insufficient data length\n");
                return false;
            }
            break;
            
        case UDS_SERVICE_READ_DATA:
            if (request->data_length < 2) {  // DID 2바이트 필요
                printf("[UDS] Read Data: insufficient data length\n");
                return false;
            }
            break;
            
        default:
            // 다른 서비스들은 기본 검사만
            break;
    }
    
    return true;
}

/**
 * @brief UDS 통계 업데이트
 */
static void uds_update_statistics(const uds_request_t *request, const uds_response_t *response)
{
    if (response == NULL) {
        return;
    }
    
    if (response->is_negative) {
        uds_statistics.negative_responses++;
    } else {
        uds_statistics.successful_responses++;
    }
}