/**
 * @file can_service.h
 * @brief CAN Service 헤더 파일 - 브레이크 시스템 DTC 전송
 * @author Brake System Team
 * @date 2025-09-16
 *
 * @note TJA1051 트랜시버를 사용한 CAN 통신
 *       UDS 프로토콜 기반 DTC 진단 데이터 전송
 *       인터럽트 방식 동작으로 실시간 처리
 */

#ifndef CAN_SERVICE_H
#define CAN_SERVICE_H

#include <stdint.h>
#include <stdbool.h>

//=============================================================================
// TJA1051 트랜시버 제어 (통합)
//=============================================================================

/**
 * @brief TJA1051 동작 모드
 */
typedef enum {
    TJA1051_MODE_NORMAL     = 0,    // Normal mode (송수신 가능)
    TJA1051_MODE_SILENT     = 1,    // Silent mode (수신만 가능)
    TJA1051_MODE_OFF        = 2     // Off mode (저전력)
} tja1051_mode_t;

// TJA1051 제어 핀 정의 (실제 프로젝트에 맞게 수정)
#define TJA1051_S_PORT      GPIOB
#define TJA1051_S_PIN       GPIO_PIN_7

//=============================================================================
// CAN 기본 정의 (자동차 표준 기준)
//=============================================================================

/**
 * @brief CAN 속도 정의 (브레이크 시스템용)
 * @note 브레이크는 안전 시스템이므로 500kbps 사용 (빠른 응답 필요)
 */
typedef enum {
    CAN_SPEED_125K      = 125000,   // Low speed CAN (Body network)
    CAN_SPEED_250K      = 250000,   // Medium speed CAN 
    CAN_SPEED_500K      = 500000,   // High speed CAN (Powertrain, Brake)
    CAN_SPEED_1M        = 1000000   // Very high speed CAN
} can_speed_t;

/**
 * @brief CAN ID 타입 정의
 * @note 11비트 표준 ID vs 29비트 확장 ID
 */
typedef enum {
    CAN_ID_STANDARD     = 0,        // 11비트 ID (0x000 ~ 0x7FF)
    CAN_ID_EXTENDED     = 1         // 29비트 ID (0x00000000 ~ 0x1FFFFFFF)
} can_id_type_t;

/**
 * @brief CAN 프레임 타입
 */
typedef enum {
    CAN_FRAME_DATA      = 0,        // 데이터 프레임 (일반적인 데이터 전송)
    CAN_FRAME_REMOTE    = 1,        // 리모트 프레임 (데이터 요청)
    CAN_FRAME_ERROR     = 2,        // 에러 프레임
    CAN_FRAME_OVERLOAD  = 3         // 오버로드 프레임
} can_frame_type_t;

//=============================================================================
// 브레이크 시스템 CAN ID 정의 (실제 자동차 기준)
//=============================================================================

/**
 * @brief 브레이크 시스템 CAN ID 맵핑
 * @note 실제 자동차에서 사용되는 ID 체계
 *       낮은 ID = 높은 우선순위 (중요한 안전 메시지)
 */
#define CAN_ID_BRAKE_EMERGENCY      0x080   // 긴급 브레이크 (최고 우선순위)
#define CAN_ID_BRAKE_STATUS         0x200   // 브레이크 상태 (주기적 전송)
#define CAN_ID_BRAKE_PRESSURE       0x201   // 브레이크 압력 데이터
#define CAN_ID_BRAKE_TEMPERATURE    0x202   // 브레이크 온도 데이터
#define CAN_ID_BRAKE_DTC            0x203   // DTC 이벤트 전송

// UDS 진단 통신용 ID (ISO 14229 표준)
#define CAN_ID_UDS_REQUEST          0x7E0   // Diagnostic Request (Tester -> ECU)
#define CAN_ID_UDS_RESPONSE         0x7E8   // Diagnostic Response (ECU -> Tester)

//=============================================================================
// CAN 프레임 구조체 (8바이트 데이터 필드)
//=============================================================================

/**
 * @brief CAN 프레임 구조체
 * @note CAN 2.0B 표준 기준, 8바이트 데이터 필드
 */
typedef struct {
    uint32_t            id;                 // CAN ID (11bit or 29bit)
    can_id_type_t       id_type;            // ID 타입 (표준/확장)
    can_frame_type_t    frame_type;         // 프레임 타입
    uint8_t             dlc;                // Data Length Code (0~8)
    bool                rtr;                // Remote Transmission Request
    uint8_t             data[8];            // 데이터 필드 (8바이트)
    uint32_t            timestamp;          // 수신/송신 시간 (ms)
} can_frame_t;

/**
 * @brief CAN 상태 정보
 */
typedef struct {
    bool                bus_off;            // Bus Off 상태
    bool                error_warning;      // Error Warning 상태
    bool                error_passive;      // Error Passive 상태
    uint8_t             tx_error_count;     // 송신 에러 카운터
    uint8_t             rx_error_count;     // 수신 에러 카운터
    uint32_t            total_tx_count;     // 총 송신 프레임 수
    uint32_t            total_rx_count;     // 총 수신 프레임 수
    uint32_t            total_error_count;  // 총 에러 발생 수
} can_status_t;

//=============================================================================
// UDS 프로토콜 연동을 위한 구조체 (ISO-TP 계층)
//=============================================================================

/**
 * @brief ISO-TP PCI (Protocol Control Information) 타입
 * @note CAN 프레임에서 멀티 프레임 메시지 처리용
 */
typedef enum {
    ISO_TP_PCI_SF   = 0,    // Single Frame (0~7바이트)
    ISO_TP_PCI_FF   = 1,    // First Frame (멀티프레임 시작)
    ISO_TP_PCI_CF   = 2,    // Consecutive Frame (연속 프레임)
    ISO_TP_PCI_FC   = 3     // Flow Control (흐름 제어)
} iso_tp_pci_type_t;

/**
 * @brief UDS 메시지 상태
 */
typedef enum {
    UDS_MSG_IDLE        = 0,
    UDS_MSG_RECEIVING   = 1,
    UDS_MSG_SENDING     = 2,
    UDS_MSG_COMPLETE    = 3,
    UDS_MSG_ERROR       = 4
} uds_message_state_t;

//=============================================================================
// 브레이크 시스템 DTC 데이터 구조체
//=============================================================================

/**
 * @brief 브레이크 DTC CAN 전송 데이터
 * @note 8바이트 CAN 프레임에 최적화
 */
typedef union {
    uint8_t raw_data[8];    // 원시 데이터

    // DTC 이벤트 알림 (ID: 0x203)
    struct {
        uint16_t dtc_code;          // DTC 코드 (예: 0xC001)
        uint8_t  dtc_status;        // DTC 상태 (Active/Inactive)
        uint8_t  occurrence_count;   // 발생 횟수
        uint32_t timestamp;         // 타임스탬프 (초)
    } __attribute__((packed)) dtc_event;

    // 브레이크 상태 데이터 (ID: 0x200)
    struct {
        uint8_t  system_status;     // 시스템 상태
        uint8_t  active_dtc_count;  // 활성 DTC 수
        uint16_t brake_pressure;    // 브레이크 압력 (kPa)
        uint8_t  temperature;       // 온도 (°C)
        uint8_t  power_mode;        // 전원 모드
        uint16_t reserved;          // 예약 영역
    } __attribute__((packed)) status;

} brake_can_data_t;

//=============================================================================
// 함수 선언 - 2단계 (UDS 연동 추가)
//=============================================================================

/**
 * @brief CAN Service 초기화
 * @param speed CAN 통신 속도
 * @return true: 성공, false: 실패
 * 
 * @note TJA1051 트랜시버와 CAN 컨트롤러 초기화
 *       - CAN 컨트롤러 설정 (속도, 필터 등)
 *       - TJA1051 Normal 모드 설정
 *       - 인터럽트 활성화
 *       - UDS 프로토콜 연동
 */
bool can_service_init(can_speed_t speed);

/**
 * @brief CAN Service 종료
 * @return true: 성공, false: 실패
 */
bool can_service_deinit(void);

/**
 * @brief CAN 상태 정보 읽기
 * @param status 상태 정보 저장 포인터
 * @return true: 성공, false: 실패
 */
bool can_service_get_status(can_status_t *status);

/**
 * @brief CAN 프레임 송신
 * @param frame 송신할 프레임
 * @return true: 성공, false: 실패
 */
bool can_service_transmit(const can_frame_t *frame);

//=============================================================================
// UDS 진단 통신 함수들
//=============================================================================

/**
 * @brief UDS 진단 메시지 처리 (수신된 CAN 프레임에서)
 * @param frame 수신된 CAN 프레임
 * @return true: UDS 메시지로 처리됨, false: 일반 CAN 메시지
 * 
 * @note CAN ID가 0x7E0 (진단 요청)인 경우 UDS로 처리
 *       ISO-TP 프로토콜로 멀티프레임 지원
 */
bool can_service_handle_uds_request(const can_frame_t *frame);

/**
 * @brief UDS 응답 전송
 * @param response_data UDS 응답 데이터
 * @param data_length 데이터 길이
 * @return true: 성공, false: 실패
 * 
 * @note 8바이트 초과시 자동으로 멀티프레임 전송
 */
bool can_service_send_uds_response(const uint8_t *response_data, uint16_t data_length);

//=============================================================================
// 브레이크 DTC 전송 함수들 (핵심 기능)
//=============================================================================

/**
 * @brief DTC 이벤트 브로드캐스트
 * @param dtc_code DTC 코드
 * @param status DTC 상태 (Active=1, Inactive=0)
 * @return true: 성공, false: 실패
 * 
 * @note DTC 발생/해제시 CAN ID 0x203으로 전송
 *       모든 ECU에 DTC 상태 알림
 */
bool can_service_broadcast_dtc_event(uint16_t dtc_code, uint8_t status);

/**
 * @brief 브레이크 시스템 상태 전송
 * @param status_data 상태 데이터
 * @return true: 성공, false: 실패
 * 
 * @note 주기적으로 CAN ID 0x200으로 전송 (100ms 주기)
 */
bool can_service_send_brake_status(const brake_can_data_t *status_data);

#endif // CAN_SERVICE_H