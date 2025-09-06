/**
 * @file tja1051.h
 * @brief TJA1051 CAN Transceiver 제어 헤더 파일 (실제 데이터시트 기준)
 * @author Brake System Team
 * @date 2025-09-05
 *
 * @note 브레이크 시스템 진단 데이터 전송
 *       CAN Controller와 CAN Bus 사이의 물리층 트랜시버
 *       UDS 프로토콜 기반, CAN Interrupt 방식 동작
 *       데이터시트: TJA1051 NXP 공식 문서 기준
 */

#ifndef TJA1051_H
#define TJA1051_H

#include <stdint.h>
#include <stdbool.h>

//=============================================================================
// TJA1051 핀 정의 (데이터시트 Table 3 기준)
//=============================================================================

/**
 * @brief TJA1051 핀 번호 정의
 * @note 데이터시트 Pinning information 기준
 */
typedef enum {
    TJA1051_PIN_TXD     = 1,    // Transmit data input
    TJA1051_PIN_GND     = 2,    // Ground
    TJA1051_PIN_VCC     = 3,    // Supply voltage (4.5V~5.5V)
    TJA1051_PIN_RXD     = 4,    // Receive data output
    TJA1051_PIN_EN      = 5,    // Enable control (TJA1051T/E only)
    TJA1051_PIN_VIO     = 5,    // I/O supply voltage (TJA1051T/3 only)
    TJA1051_PIN_NC      = 5,    // Not connected (TJA1051T only)
    TJA1051_PIN_CANL    = 6,    // LOW-level CAN bus line
    TJA1051_PIN_CANH    = 7,    // HIGH-level CAN bus line
    TJA1051_PIN_S       = 8     // Silent mode control input
} tja1051_pin_t;

//=============================================================================
// TJA1051 동작 모드 (데이터시트 Table 4 기준)
//=============================================================================

/**
 * @brief TJA1051 동작 모드 정의
 * @note 데이터시트 Table 4: Operating modes 기준
 */
typedef enum {
    TJA1051_MODE_NORMAL     = 0,    // Normal mode (S=LOW)
    TJA1051_MODE_SILENT     = 1,    // Silent mode (S=HIGH)
    TJA1051_MODE_OFF        = 2     // Off mode (EN=LOW, TJA1051T/E only)
} tja1051_mode_t;

/**
 * @brief TJA1051 버전별 타입 정의
 */
typedef enum {
    TJA1051_TYPE_T          = 0,    // TJA1051T (핀5 = n.c.)
    TJA1051_TYPE_T_E        = 1,    // TJA1051T/E (핀5 = EN)
    TJA1051_TYPE_T_3        = 2,    // TJA1051T/3 (핀5 = VIO)
    TJA1051_TYPE_TK_3       = 3     // TJA1051TK/3 (핀5 = VIO)
} tja1051_type_t;

//=============================================================================
// 전기적 특성 매크로 (데이터시트 기준)
//=============================================================================

/**
 * @brief 공급 전압 범위 (데이터시트 Table 7)
 */
#define TJA1051_VCC_MIN         4.5f    // V
#define TJA1051_VCC_MAX         5.5f    // V
#define TJA1051_VIO_MIN         2.8f    // V (TJA1051T/3 only)
#define TJA1051_VIO_MAX         5.5f    // V

/**
 * @brief 공급 전류 (데이터시트 Table 7)
 */
#define TJA1051_ICC_OFF_MAX     8       // μA (Off mode, TJA1051T/E)
#define TJA1051_ICC_SILENT_MAX  2500    // μA (Silent mode)
#define TJA1051_ICC_NORMAL_REC  10000   // μA (Normal mode, recessive)
#define TJA1051_ICC_NORMAL_DOM  70000   // μA (Normal mode, dominant)

/**
 * @brief CAN 버스 전압 레벨 (데이터시트 Table 7)
 */
#define TJA1051_CANH_DOMINANT_MIN   2.75f   // V
#define TJA1051_CANH_DOMINANT_TYP   3.5f    // V
#define TJA1051_CANH_DOMINANT_MAX   4.5f    // V
#define TJA1051_CANL_DOMINANT_MIN   0.5f    // V
#define TJA1051_CANL_DOMINANT_TYP   1.5f    // V
#define TJA1051_CANL_DOMINANT_MAX   2.25f   // V

/**
 * @brief 차동 전압 (데이터시트 Table 7)
 */
#define TJA1051_VDIFF_DOMINANT_MIN  1.5f    // V
#define TJA1051_VDIFF_DOMINANT_MAX  3.0f    // V
#define TJA1051_VDIFF_RECESSIVE_MIN -50e-3f // mV
#define TJA1051_VDIFF_RECESSIVE_MAX +50e-3f // mV

/**
 * @brief 수신기 임계값 (데이터시트 Table 7)
 */
#define TJA1051_VTH_DIFF_MIN        0.5f    // V
#define TJA1051_VTH_DIFF_TYP        0.7f    // V
#define TJA1051_VTH_DIFF_MAX        0.9f    // V

/**
 * @brief TXD 도미넌트 타임아웃 (데이터시트 Table 8)
 */
#define TJA1051_TXD_TIMEOUT_MIN     0.3f    // ms
#define TJA1051_TXD_TIMEOUT_TYP     1.0f    // ms
#define TJA1051_TXD_TIMEOUT_MAX     5.0f    // ms

//=============================================================================
// 타이밍 특성 (데이터시트 Table 8)
//=============================================================================

/**
 * @brief 전파 지연 시간 (나노초)
 */
#define TJA1051_TD_TXD_BUS_DOM_TYP  65      // ns
#define TJA1051_TD_TXD_BUS_REC_TYP  90      // ns
#define TJA1051_TD_BUS_RXD_DOM_TYP  60      // ns
#define TJA1051_TD_BUS_RXD_REC_TYP  65      // ns

//=============================================================================
// CAN 프레임 및 UDS 구조체 (8바이트 데이터)
//=============================================================================

/**
 * @brief CAN 프레임 타입 정의
 */
typedef enum {
    CAN_FRAME_TYPE_DATA         = 0,    // 데이터 프레임
    CAN_FRAME_TYPE_REMOTE       = 1,    // 리모트 프레임
    CAN_FRAME_TYPE_ERROR        = 2,    // 에러 프레임
    CAN_FRAME_TYPE_OVERLOAD     = 3     // 오버로드 프레임
} can_frame_type_t;

/**
 * @brief CAN 프레임 구조체 (8바이트 데이터)
 */
typedef struct {
    uint32_t            id;             // CAN ID (11비트 또는 29비트)
    uint8_t             dlc;            // Data Length Code (0~8)
    can_frame_type_t    type;           // 프레임 타입
    bool                extended;       // 확장 프레임 여부
    bool                rtr;            // Remote Transmission Request
    uint8_t             data[8];        // 데이터 (8바이트)
} can_frame_t;

/**
 * @brief UDS 서비스 ID 정의 (요구사항: UDS 프로토콜)
 */
typedef enum {
    UDS_SERVICE_DIAG_CTRL           = 0x10,     // Diagnostic Session Control
    UDS_SERVICE_ECU_RESET           = 0x11,     // ECU Reset
    UDS_SERVICE_CLEAR_DTC           = 0x14,     // Clear DTC (요구사항 핵심!)
    UDS_SERVICE_READ_DTC            = 0x19,     // Read DTC Information
    UDS_SERVICE_READ_DATA           = 0x22,     // Read Data By Identifier
    UDS_SERVICE_WRITE_DATA          = 0x2E,     // Write Data By Identifier
    UDS_SERVICE_ROUTINE_CTRL        = 0x31,     // Routine Control
    UDS_SERVICE_TESTER_PRESENT      = 0x3E      // Tester Present
} uds_service_id_t;

/**
 * @brief UDS 진단 프레임 구조체 (Union 사용)
 */
typedef union {
    uint8_t raw_data[8];                    // 원시 데이터

    // Single Frame (SF) - 데이터 길이 7바이트 이하
    struct {
        uint8_t pci_type    : 4;            // PCI Type (0: SF)
        uint8_t sf_dl       : 4;            // Single Frame Data Length
        uint8_t service_id;                 // UDS 서비스 ID
        uint8_t data[6];                    // 서비스 데이터 (최대 6바이트)
    } __attribute__((packed)) single_frame;

    // First Frame (FF) - 긴 메시지의 첫 프레임
    struct {
        uint8_t pci_type    : 4;            // PCI Type (1: FF)
        uint8_t ff_dl_high  : 4;            // Data Length 상위 4비트
        uint8_t ff_dl_low;                  // Data Length 하위 8비트
        uint8_t service_id;                 // UDS 서비스 ID
        uint8_t data[5];                    // 서비스 데이터 (5바이트)
    } __attribute__((packed)) first_frame;

    // Consecutive Frame (CF) - 연속 프레임
    struct {
        uint8_t pci_type    : 4;            // PCI Type (2: CF)
        uint8_t sn          : 4;            // Sequence Number
        uint8_t data[7];                    // 데이터 (7바이트)
    } __attribute__((packed)) consecutive_frame;

} uds_frame_data_t;

/**
 * @brief 브레이크 시스템 CAN ID 정의
 */
#define CAN_ID_BRAKE_DIAG_TX        0x7E0       // 브레이크 진단 송신 ID
#define CAN_ID_BRAKE_DIAG_RX        0x7E8       // 브레이크 진단 수신 ID
#define CAN_ID_BRAKE_STATUS         0x200       // 브레이크 상태 ID
#define CAN_ID_BRAKE_DTC            0x201       // 브레이크 DTC ID

/**
 * @brief 브레이크 시스템 상태 데이터 (8바이트)
 */
typedef union {
    uint8_t raw_data[8];

    struct {
        uint16_t    active_dtc_count;       // 활성 DTC 수
        uint16_t    pending_dtc_count;      // 대기 DTC 수
        uint8_t     system_status;          // 시스템 상태
        uint8_t     power_mode;             // 전원 모드
        uint8_t     brake_pressure;         // 브레이크 압력 (%)
        uint8_t     temperature;            // 온도 (°C)
    } __attribute__((packed)) status;

    struct {
        uint16_t    dtc_code;               // DTC 코드
        uint8_t     dtc_status;             // DTC 상태
        uint32_t    timestamp;              // 타임스탬프
        uint8_t     occurrence_count;       // 발생 횟수
    } __attribute__((packed)) dtc_info;

} brake_can_data_t;

//=============================================================================
// TJA1051 상태 구조체
//=============================================================================

/**
 * @brief TJA1051 상태 구조체
 */
typedef struct {
    tja1051_mode_t  mode;                   // 현재 모드
    tja1051_type_t  type;                   // TJA1051 버전
    bool            txd_timeout_occurred;   // TXD 타임아웃 발생 여부
    bool            over_temperature;       // 과온도 보호 활성화
    bool            undervoltage_vcc;       // VCC 저전압 감지
    bool            undervoltage_vio;       // VIO 저전압 감지 (T/3 only)
    uint32_t        tx_error_count;         // 송신 에러 카운터
    uint32_t        rx_error_count;         // 수신 에러 카운터
} tja1051_status_t;

//=============================================================================
// 함수 선언
//=============================================================================

/**
 * @brief TJA1051 초기화
 * @param type TJA1051 버전 타입
 * @return true: 성공, false: 실패
 */
bool tja1051_init(tja1051_type_t type);

/**
 * @brief TJA1051 모드 설정
 * @param mode 설정할 모드 (Normal/Silent/Off)
 * @return true: 성공, false: 실패
 * @note S 핀과 EN 핀(T/E only) 제어
 */
bool tja1051_set_mode(tja1051_mode_t mode);

/**
 * @brief TJA1051 현재 모드 읽기
 * @return 현재 동작 모드
 */
tja1051_mode_t tja1051_get_mode(void);

/**
 * @brief TJA1051 상태 읽기
 * @param status 상태 정보 저장 포인터
 * @return true: 성공, false: 실패
 */
bool tja1051_get_status(tja1051_status_t *status);

/**
 * @brief CAN 송신 활성화/비활성화 (TXD 핀 제어)
 * @param enable true: 도미넌트 송신, false: 리시시브
 * @return true: 성공, false: 실패
 * @note TJA1051은 단순 물리층이므로 핀 레벨만 제어
 */
bool tja1051_set_transmit(bool enable);

/**
 * @brief CAN 수신 상태 읽기 (RXD 핀 읽기)
 * @return true: 도미넌트 수신됨, false: 리시시브
 */
bool tja1051_get_receive_status(void);

/**
 * @brief Silent 모드 설정 (S 핀 제어)
 * @param silent true: Silent mode, false: Normal mode
 * @return true: 성공, false: 실패
 */
bool tja1051_set_silent_mode(bool silent);

/**
 * @brief Enable 제어 (EN 핀 제어, TJA1051T/E only)
 * @param enable true: Enable, false: Off mode
 * @return true: 성공, false: 실패
 */
bool tja1051_set_enable(bool enable);

/**
 * @brief VCC 전압 모니터링
 * @return VCC 전압 (V), 측정 불가시 -1.0f
 */
float tja1051_get_vcc_voltage(void);

/**
 * @brief VIO 전압 모니터링 (TJA1051T/3 only)
 * @return VIO 전압 (V), 측정 불가시 -1.0f
 */
float tja1051_get_vio_voltage(void);

/**
 * @brief 저전압 감지 상태 확인
 * @param vcc_uvd VCC 저전압 감지 상태 저장 포인터
 * @param vio_uvd VIO 저전압 감지 상태 저장 포인터 (T/3 only)
 * @return true: 정상, false: 저전압 감지됨
 */
bool tja1051_check_undervoltage(bool *vcc_uvd, bool *vio_uvd);

/**
 * @brief TXD 도미넌트 타임아웃 확인
 * @return true: 타임아웃 발생, false: 정상
 */
bool tja1051_check_txd_timeout(void);

/**
 * @brief 과온도 보호 상태 확인
 * @return true: 과온도 보호 활성, false: 정상
 */
bool tja1051_check_overtemperature(void);

/**
 * @brief CAN 버스 도미넌트/리시시브 감지
 * @return true: 도미넌트, false: 리시시브
 */
bool tja1051_is_bus_dominant(void);

/**
 * @brief CAN 버스 차동 전압 측정
 * @return 차동 전압 (CANH - CANL), 측정 불가시 0.0f
 */
float tja1051_get_bus_differential_voltage(void);

/**
 * @brief 입력 저항 측정 (데이터시트 특성 확인용)
 * @return 입력 저항 (kΩ), 측정 불가시 -1.0f
 */
float tja1051_get_input_resistance(void);

//=============================================================================
// CAN Controller 인터페이스 함수 (상위 레이어)
//=============================================================================

/**
 * @brief CAN 프레임 송신 준비 (CAN Controller와 연동)
 * @param frame 송신할 CAN 프레임
 * @return true: 성공, false: 실패
 * @note 실제 송신은 CAN Controller가 담당
 */
bool tja1051_prepare_transmit(const can_frame_t *frame);

/**
 * @brief UDS 진단 데이터 송신 (요구사항: UDS 프로토콜)
 * @param service_id UDS 서비스 ID
 * @param data 서비스 데이터
 * @param data_length 데이터 길이
 * @return true: 성공, false: 실패
 */
bool tja1051_transmit_uds(uds_service_id_t service_id, const uint8_t *data, uint8_t data_length);

/**
 * @brief DTC Clear 요청 처리 (요구사항: Service 0x14)
 * @param dtc_group DTC 그룹 (0xFFFFFF: 모든 DTC)
 * @return true: 성공, false: 실패
 */
bool tja1051_handle_clear_dtc(uint32_t dtc_group);

/**
 * @brief 브레이크 시스템 상태 브로드캐스트
 * @param status_data 상태 데이터
 * @return true: 성공, false: 실패
 */
bool tja1051_broadcast_brake_status(const brake_can_data_t *status_data);

/**
 * @brief 진단 데이터 전송 (요구사항: 진단 데이터 전송)
 * @param diag_data 진단 데이터
 * @param data_length 데이터 길이
 * @return true: 성공, false: 실패
 */
bool tja1051_transmit_diagnostic_data(const uint8_t *diag_data, uint8_t data_length);

//=============================================================================
// 인터럽트 콜백 함수 (요구사항: CAN Interrupt 방식)
//=============================================================================

/**
 * @brief CAN 송신 완료 콜백 (인터럽트에서 호출)
 * @param frame 송신 완료된 프레임
 */
void tja1051_tx_complete_callback(const can_frame_t *frame);

/**
 * @brief CAN 수신 완료 콜백 (인터럽트에서 호출)
 * @param frame 수신된 프레임
 */
void tja1051_rx_complete_callback(const can_frame_t *frame);

/**
 * @brief CAN 에러 콜백 (인터럽트에서 호출)
 * @param error_type 에러 타입
 */
void tja1051_error_callback(uint32_t error_type);

/**
 * @brief TXD 타임아웃 콜백 (인터럽트에서 호출)
 */
void tja1051_txd_timeout_callback(void);

#endif // TJA1051_H
