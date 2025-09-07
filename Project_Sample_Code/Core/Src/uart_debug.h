/**
 * @file uart_debug.h
 * @brief UART Debug Log Polling 방식 헤더 파일
 * @author Brake System Team
 * @date 2025-09-05
 *
 * @note UART Polling 방식으로 시스템 상태와 디버그 정보를 출력합니다.
 */

#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include <stdint.h>
#include <stdbool.h>

//=============================================================================
// 매크로 정의
//=============================================================================

/**
 * @brief UART 설정 매크로
 */
#define UART_DEFAULT_BAUD_RATE      115200      // 기본 전송 속도
#define UART_BUFFER_SIZE            256         // 송신 버퍼 크기
#define UART_TIMEOUT_MS             100         // Polling 타임아웃 (ms)

//=============================================================================
// 로그 레벨 정의
//=============================================================================

/**
 * @brief UART 로그 레벨 정의
 */
typedef enum {
    UART_LOG_DEBUG      = 0,    // 디버그 정보
    UART_LOG_INFO       = 1,    // 일반 정보
    UART_LOG_WARNING    = 2,    // 경고
    UART_LOG_ERROR      = 3,    // 에러
    UART_LOG_CRITICAL   = 4     // 치명적 에러
} uart_log_level_t;

//=============================================================================
// 함수 선언
//=============================================================================

/**
 * @brief UART Debug 초기화 (Polling 방식)
 * @param baud_rate 전송 속도 (기본: 115200bps)
 * @return true: 성공, false: 실패
 */
bool uart_debug_init(uint32_t baud_rate);

/**
 * @brief UART Polling 방식 문자열 송신
 * @param data 송신할 문자열
 * @param length 문자열 길이
 * @return true: 성공, false: 실패
 */
bool uart_debug_transmit_polling(const char* data, uint16_t length);

/**
 * @brief 포맷된 로그 메시지 출력 (가변 인자)
 * @param level 로그 레벨
 * @param format 포맷 문자열
 * @param ... 가변 인자
 * @return true: 성공, false: 실패
 */
bool uart_debug_log(uart_log_level_t level, const char* format, ...);

/**
 * @brief 정보 로그 출력
 */
bool uart_debug_print_info(const char* format, ...);

/**
 * @brief 경고 로그 출력
 */
bool uart_debug_print_warning(const char* format, ...);

/**
 * @brief 에러 로그 출력
 */
bool uart_debug_print_error(const char* format, ...);

/**
 * @brief 시스템 상태 로그 출력 (요구사항)
 * @param i2c_status I2C PMIC 상태
 * @param spi_status SPI EEPROM 상태
 * @param can_status CAN 통신 상태
 * @return true: 성공, false: 실패
 */
bool uart_debug_print_system_status(bool i2c_status, bool spi_status, bool can_status);

/**
 * @brief 정상 동작 메시지 출력 (요구사항)
 * @param loop_count 루프 카운터
 * @return true: 성공, false: 실패
 */
bool uart_debug_print_normal_operation(uint32_t loop_count);

/**
 * @brief 에러 발생시 상세 정보 출력 (요구사항)
 * @param error_code 에러 코드
 * @param module_name 모듈 이름
 * @param detail_message 상세 메시지
 * @return true: 성공, false: 실패
 */
bool uart_debug_print_error_detail(uint32_t error_code,
                                  const char* module_name,
                                  const char* detail_message);

/**
 * @brief 성능 모니터링 로그 출력 (요구사항)
 * @param i2c_time I2C 처리 시간 (ms)
 * @param spi_time SPI 처리 시간 (ms)
 * @param can_time CAN 처리 시간 (ms)
 * @param total_time 전체 루프 시간 (ms)
 * @return true: 성공, false: 실패
 */
bool uart_debug_print_performance(uint32_t i2c_time, uint32_t spi_time,
                                 uint32_t can_time, uint32_t total_time);

/**
 * @brief UART 통계 정보 출력
 * @return true: 성공, false: 실패
 */
bool uart_debug_print_statistics(void);

/**
 * @brief 로그 레벨 설정
 * @param level 새로운 로그 레벨
 * @return true: 성공, false: 실패
 */
bool uart_debug_set_log_level(uart_log_level_t level);

/**
 * @brief 디버그 출력 활성화/비활성화
 * @param enable true: 활성화, false: 비활성화
 * @return true: 성공, false: 실패
 */
bool uart_debug_enable(bool enable);

/**
 * @brief 헥사 덤프 출력 (바이너리 데이터 디버깅용)
 * @param data 출력할 데이터
 * @param length 데이터 길이
 * @param description 데이터 설명
 * @return true: 성공, false: 실패
 */
bool uart_debug_print_hex_dump(const uint8_t* data, uint16_t length, const char* description);

/**
 * @brief UART Debug 상태 확인
 * @return true: 정상, false: 이상
 */
bool uart_debug_is_ok(void);

/**
 * @brief 현재 상태 반환
 * @param bytes_sent 총 전송 바이트 수 저장 포인터
 * @param message_count 메시지 수 저장 포인터
 * @param error_count 에러 수 저장 포인터
 * @return true: 성공, false: 실패
 */
bool uart_debug_get_statistics(uint32_t* bytes_sent, uint32_t* message_count, uint32_t* error_count);

#endif // UART_DEBUG_H
