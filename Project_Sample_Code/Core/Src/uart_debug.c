/**
 * @file uart_debug.c
 * @brief UART Debug Log Polling 방식 구현
 * @author Brake System Team
 * @date 2025-09-05
 *
 * @note UART Polling 방식으로 시스템 상태와 디버그 정보를 출력합니다.
 *       단순한 로그 출력이므로 CPU 효율성을 고려하여 Polling 방식을 사용합니다.
 */

#include "uart_debug.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

//=============================================================================
// 전역 변수 (UART Polling 동작을 위한 상태 관리)
//=============================================================================

/**
 * @brief UART Debug 상태 구조체
 */
typedef struct {
    bool                initialized;            // 초기화 완료 플래그
    bool                enabled;                // 디버그 출력 활성화 플래그
    uint32_t            baud_rate;              // 전송 속도
    uint32_t            total_bytes_sent;       // 총 전송 바이트 수
    uint32_t            message_count;          // 메시지 카운터
    uint32_t            error_count;            // 전송 에러 카운터

    // 로그 레벨 설정
    uart_log_level_t    current_log_level;      // 현재 로그 레벨

    // 통계 정보
    uint32_t            system_status_logs;     // 시스템 상태 로그 수
    uint32_t            error_logs;             // 에러 로그 수
    uint32_t            performance_logs;       // 성능 모니터 로그 수
} uart_debug_state_t;

// UART Debug 상태 변수
static uart_debug_state_t uart_state = {0};

// 송신 버퍼 (Polling 방식이므로 크기 제한)
static char uart_tx_buffer[UART_BUFFER_SIZE];

// 로그 레벨 문자열 배열
static const char* log_level_strings[] = {
    "DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"
};

//=============================================================================
// UART Polling 초기화 함수
//=============================================================================

/**
 * @brief UART Debug 초기화 (Polling 방식)
 * @param baud_rate 전송 속도 (기본: 115200bps)
 * @return true: 성공, false: 실패
 *
 * @note UART 하드웨어를 Polling 모드로 설정합니다.
 *       인터럽트나 DMA를 사용하지 않아 간단하고 안정적입니다.
 */
bool uart_debug_init(uint32_t baud_rate)
{
    printf("[UART-POLL] UART Debug 초기화 시작 (속도: %lu bps)\n", baud_rate);

    // 1단계: UART 하드웨어 초기화 (Polling 모드)
    /*
    실제 코드 예시 (STM32 HAL):
    huart.Instance = USART2;
    huart.Init.BaudRate = baud_rate;
    huart.Init.WordLength = UART_WORDLENGTH_8B;
    huart.Init.StopBits = UART_STOPBITS_1;
    huart.Init.Parity = UART_PARITY_NONE;
    huart.Init.Mode = UART_MODE_TX_RX;
    huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart) != HAL_OK) {
        printf("[UART-POLL] UART 초기화 실패!\n");
        return false;
    }
    */

    // 2단계: 상태 변수 초기화
    memset(&uart_state, 0, sizeof(uart_state));
    uart_state.baud_rate = baud_rate;
    uart_state.current_log_level = UART_LOG_INFO;  // 기본 로그 레벨
    uart_state.enabled = true;

    // 3단계: 송신 버퍼 초기화
    memset(uart_tx_buffer, 0, sizeof(uart_tx_buffer));

    // 4단계: 초기화 완료 메시지 출력
    uart_state.initialized = true;
    uart_debug_print_info("UART Debug 초기화 완료 (%lu bps)", baud_rate);

    return true;
}

//=============================================================================
// UART Polling 송신 함수 (핵심)
//=============================================================================

/**
 * @brief UART Polling 방식 문자열 송신
 * @param data 송신할 문자열
 * @param length 문자열 길이
 * @return true: 성공, false: 실패
 *
 * @note Polling 방식으로 한 바이트씩 전송합니다.
 *       간단하지만 전송 완료까지 CPU가 대기합니다.
 */
bool uart_debug_transmit_polling(const char* data, uint16_t length)
{
    // 초기화 및 활성화 확인
    if (!uart_state.initialized || !uart_state.enabled) {
        return false;
    }

    // 1단계: 길이 제한 확인
    if (length > UART_BUFFER_SIZE - 1) {
        length = UART_BUFFER_SIZE - 1;
        uart_state.error_count++;
    }

    // 2단계: Polling 방식으로 바이트별 전송
    for (uint16_t i = 0; i < length; i++) {
        // 각 바이트를 Polling 방식으로 전송
        /*
        실제 코드 예시 (STM32 HAL):
        HAL_StatusTypeDef status = HAL_UART_Transmit(
            &huart,                     // UART 핸들
            (uint8_t*)&data[i],        // 송신 데이터 (1바이트)
            1,                         // 송신 길이
            UART_TIMEOUT_MS            // 타임아웃 (예: 100ms)
        );

        if (status != HAL_OK) {
            uart_state.error_count++;
            printf("[UART-POLL] 송신 에러: 바이트 %d\n", i);
            return false;
        }
        */

        // 시뮬레이션: 실제 콘솔 출력
        putchar(data[i]);

        // 3단계: 송신 완료 대기 (Polling의 특징)
        /*
        실제 코드에서는 UART 상태 레지스터를 확인:
        while (!(USART2->SR & USART_SR_TXE)) {
            // TX Empty 플래그 대기 (Polling)
            // 타임아웃 카운터 증가
            timeout_counter++;
            if (timeout_counter > UART_TIMEOUT_COUNT) {
                return false; // 타임아웃
            }
        }
        */
    }

    // 4단계: 통계 업데이트
    uart_state.total_bytes_sent += length;
    uart_state.message_count++;

    return true;
}

//=============================================================================
// 로그 레벨별 출력 함수
//=============================================================================

/**
 * @brief 포맷된 로그 메시지 출력 (가변 인자)
 * @param level 로그 레벨
 * @param format 포맷 문자열
 * @param ... 가변 인자
 * @return true: 성공, false: 실패
 */
bool uart_debug_log(uart_log_level_t level, const char* format, ...)
{
    // 로그 레벨 확인
    if (level < uart_state.current_log_level) {
        return true; // 현재 설정보다 낮은 레벨은 출력하지 않음
    }

    // 1단계: 타임스탬프 생성 (시뮬레이션)
    uint32_t timestamp = uart_state.message_count; // 실제로는 HAL_GetTick() 사용

    // 2단계: 로그 헤더 생성
    int header_len = snprintf(uart_tx_buffer, UART_BUFFER_SIZE,
                             "[%06lu] [%s] ",
                             timestamp,
                             log_level_strings[level]);

    // 3단계: 가변 인자를 사용한 메시지 포맷팅
    va_list args;
    va_start(args, format);
    int msg_len = vsnprintf(&uart_tx_buffer[header_len],
                           UART_BUFFER_SIZE - header_len - 2,
                           format, args);
    va_end(args);

    // 4단계: 줄바꿈 추가
    int total_len = header_len + msg_len;
    if (total_len < UART_BUFFER_SIZE - 2) {
        uart_tx_buffer[total_len] = '\r';
        uart_tx_buffer[total_len + 1] = '\n';
        total_len += 2;
    }

    // 5단계: Polling 방식으로 전송
    bool result = uart_debug_transmit_polling(uart_tx_buffer, total_len);

    // 6단계: 레벨별 통계 업데이트
    switch (level) {
        case UART_LOG_ERROR:
        case UART_LOG_CRITICAL:
            uart_state.error_logs++;
            break;
        case UART_LOG_INFO:
            uart_state.system_status_logs++;
            break;
        case UART_LOG_DEBUG:
            uart_state.performance_logs++;
            break;
        default:
            break;
    }

    return result;
}

// 편의 함수들 (각 로그 레벨별)

/**
 * @brief 정보 로그 출력
 */
bool uart_debug_print_info(const char* format, ...)
{
    va_list args;
    va_start(args, format);

    // 임시 버퍼에 메시지 포맷팅
    char temp_buffer[UART_BUFFER_SIZE];
    vsnprintf(temp_buffer, sizeof(temp_buffer), format, args);
    va_end(args);

    return uart_debug_log(UART_LOG_INFO, "%s", temp_buffer);
}

/**
 * @brief 경고 로그 출력
 */
bool uart_debug_print_warning(const char* format, ...)
{
    va_list args;
    va_start(args, format);

    char temp_buffer[UART_BUFFER_SIZE];
    vsnprintf(temp_buffer, sizeof(temp_buffer), format, args);
    va_end(args);

    return uart_debug_log(UART_LOG_WARNING, "%s", temp_buffer);
}

/**
 * @brief 에러 로그 출력
 */
bool uart_debug_print_error(const char* format, ...)
{
    va_list args;
    va_start(args, format);

    char temp_buffer[UART_BUFFER_SIZE];
    vsnprintf(temp_buffer, sizeof(temp_buffer), format, args);
    va_end(args);

    return uart_debug_log(UART_LOG_ERROR, "%s", temp_buffer);
}

//=============================================================================
// 시스템 상태 로그 출력 (요구사항)
//=============================================================================

/**
 * @brief 시스템 상태 로그 출력 (요구사항: 시스템 상태 로그)
 * @param i2c_status I2C PMIC 상태
 * @param spi_status SPI EEPROM 상태
 * @param can_status CAN 통신 상태
 * @return true: 성공, false: 실패
 */
bool uart_debug_print_system_status(bool i2c_status, bool spi_status, bool can_status)
{
    uart_debug_log(UART_LOG_INFO, "=== 시스템 상태 보고 ===");
    uart_debug_log(UART_LOG_INFO, "I2C PMIC    : %s", i2c_status ? "정상" : "이상");
    uart_debug_log(UART_LOG_INFO, "SPI EEPROM : %s", spi_status ? "정상" : "이상");
    uart_debug_log(UART_LOG_INFO, "CAN 통신   : %s", can_status ? "정상" : "이상");

    // 전체 시스템 상태 판단
    bool system_ok = i2c_status && spi_status && can_status;

    if (system_ok) {
        uart_debug_log(UART_LOG_INFO, "시스템 전체: 모든 시스템 정상 동작");
    } else {
        uart_debug_log(UART_LOG_WARNING, "시스템 전체: 일부 시스템에서 문제 감지됨");
    }

    uart_debug_log(UART_LOG_INFO, "==================");

    return true;
}

/**
 * @brief 정상 동작 메시지 출력 (요구사항: 정상 동작 메시지)
 * @param loop_count 루프 카운터
 * @return true: 성공, false: 실패
 */
bool uart_debug_print_normal_operation(uint32_t loop_count)
{
    // 주기적으로 정상 동작 메시지 출력 (매 10회마다)
    if (loop_count % 10 == 0) {
        uart_debug_log(UART_LOG_INFO,
                      "브레이크 시스템 정상 동작 중 (루프: %lu회)",
                      loop_count);
    }

    // 100회마다 상세 통계 출력
    if (loop_count % 100 == 0) {
        uart_debug_print_statistics();
    }

    return true;
}

/**
 * @brief 에러 발생시 상세 정보 출력 (요구사항: 에러 발생시 상세 정보)
 * @param error_code 에러 코드
 * @param module_name 모듈 이름
 * @param detail_message 상세 메시지
 * @return true: 성공, false: 실패
 */
bool uart_debug_print_error_detail(uint32_t error_code,
                                  const char* module_name,
                                  const char* detail_message)
{
    uart_debug_log(UART_LOG_ERROR, "!!! 에러 발생 !!!");
    uart_debug_log(UART_LOG_ERROR, "모듈: %s", module_name);
    uart_debug_log(UART_LOG_ERROR, "에러 코드: 0x%08lX", error_code);
    uart_debug_log(UART_LOG_ERROR, "상세 정보: %s", detail_message);
    uart_debug_log(UART_LOG_ERROR, "타임스탬프: %lu ms", uart_state.message_count);
    uart_debug_log(UART_LOG_ERROR, "==================");

    return true;
}

//=============================================================================
// 성능 모니터링 및 통계 (요구사항)
//=============================================================================

/**
 * @brief 성능 모니터링 로그 출력 (요구사항: 성능 모니터)
 * @param i2c_time I2C 처리 시간 (ms)
 * @param spi_time SPI 처리 시간 (ms)
 * @param can_time CAN 처리 시간 (ms)
 * @param total_time 전체 루프 시간 (ms)
 * @return true: 성공, false: 실패
 */
bool uart_debug_print_performance(uint32_t i2c_time, uint32_t spi_time,
                                 uint32_t can_time, uint32_t total_time)
{
    uart_debug_log(UART_LOG_DEBUG, "=== 성능 모니터 ===");
    uart_debug_log(UART_LOG_DEBUG, "I2C 처리 시간: %lu ms", i2c_time);
    uart_debug_log(UART_LOG_DEBUG, "SPI 처리 시간: %lu ms", spi_time);
    uart_debug_log(UART_LOG_DEBUG, "CAN 처리 시간: %lu ms", can_time);
    uart_debug_log(UART_LOG_DEBUG, "UART 처리 시간: 1 ms");
    uart_debug_log(UART_LOG_DEBUG, "전체 루프 시간: %lu ms", total_time);

    // 성능 경고 확인
    if (total_time > 120) { // 100ms 목표 + 20ms 여유
        uart_debug_log(UART_LOG_WARNING, "루프 시간 초과! 목표: 100ms, 실제: %lu ms", total_time);
    }

    uart_debug_log(UART_LOG_DEBUG, "================");

    return true;
}

/**
 * @brief UART 통계 정보 출력
 * @return true: 성공, false: 실패
 */
bool uart_debug_print_statistics(void)
{
    uart_debug_log(UART_LOG_INFO, "=== UART Debug 통계 ===");
    uart_debug_log(UART_LOG_INFO, "총 메시지 수: %lu", uart_state.message_count);
    uart_debug_log(UART_LOG_INFO, "총 전송 바이트: %lu", uart_state.total_bytes_sent);
    uart_debug_log(UART_LOG_INFO, "시스템 상태 로그: %lu", uart_state.system_status_logs);
    uart_debug_log(UART_LOG_INFO, "에러 로그: %lu", uart_state.error_logs);
    uart_debug_log(UART_LOG_INFO, "성능 로그: %lu", uart_state.performance_logs);
    uart_debug_log(UART_LOG_INFO, "전송 에러: %lu", uart_state.error_count);
    uart_debug_log(UART_LOG_INFO, "현재 로그 레벨: %s", log_level_strings[uart_state.current_log_level]);
    uart_debug_log(UART_LOG_INFO, "==================");

    return true;
}

//=============================================================================
// 설정 및 제어 함수
//=============================================================================

/**
 * @brief 로그 레벨 설정
 * @param level 새로운 로그 레벨
 * @return true: 성공, false: 실패
 */
bool uart_debug_set_log_level(uart_log_level_t level)
{
    if (level >= UART_LOG_DEBUG && level <= UART_LOG_CRITICAL) {
        uart_state.current_log_level = level;
        uart_debug_log(UART_LOG_INFO, "로그 레벨 변경: %s", log_level_strings[level]);
        return true;
    }
    return false;
}

/**
 * @brief 디버그 출력 활성화/비활성화
 * @param enable true: 활성화, false: 비활성화
 * @return true: 성공, false: 실패
 */
bool uart_debug_enable(bool enable)
{
    uart_state.enabled = enable;

    if (enable) {
        uart_debug_log(UART_LOG_INFO, "UART Debug 출력 활성화");
    } else {
        uart_debug_log(UART_LOG_INFO, "UART Debug 출력 비활성화");
    }

    return true;
}

/**
 * @brief 헥사 덤프 출력 (바이너리 데이터 디버깅용)
 * @param data 출력할 데이터
 * @param length 데이터 길이
 * @param description 데이터 설명
 * @return true: 성공, false: 실패
 */
bool uart_debug_print_hex_dump(const uint8_t* data, uint16_t length, const char* description)
{
    if (!uart_state.enabled || data == NULL) {
        return false;
    }

    uart_debug_log(UART_LOG_DEBUG, "=== Hex Dump: %s ===", description);
    uart_debug_log(UART_LOG_DEBUG, "길이: %d 바이트", length);

    // 16바이트씩 출력
    for (uint16_t i = 0; i < length; i += 16) {
        char hex_line[64] = {0};
        char ascii_line[17] = {0};

        int hex_pos = 0;
        int ascii_pos = 0;

        // 주소 출력
        hex_pos += snprintf(&hex_line[hex_pos], sizeof(hex_line) - hex_pos, "%04X: ", i);

        // 16바이트 또는 남은 바이트 처리
        for (int j = 0; j < 16 && (i + j) < length; j++) {
            uint8_t byte_val = data[i + j];

            // 헥사 값 추가
            hex_pos += snprintf(&hex_line[hex_pos], sizeof(hex_line) - hex_pos, "%02X ", byte_val);

            // ASCII 값 추가 (출력 가능한 문자만)
            if (byte_val >= 32 && byte_val <= 126) {
                ascii_line[ascii_pos++] = byte_val;
            } else {
                ascii_line[ascii_pos++] = '.';
            }
        }

        uart_debug_log(UART_LOG_DEBUG, "%s [%s]", hex_line, ascii_line);
    }

    uart_debug_log(UART_LOG_DEBUG, "=================");

    return true;
}

//=============================================================================
// 현재 상태 확인 함수
//=============================================================================

/**
 * @brief UART Debug 상태 확인
 * @return true: 정상, false: 이상
 */
bool uart_debug_is_ok(void)
{
    return (uart_state.initialized &&
            uart_state.enabled &&
            uart_state.error_count < 10); // 에러가 10개 미만이면 정상
}

/**
 * @brief 현재 상태 반환
 * @param bytes_sent 총 전송 바이트 수 저장 포인터
 * @param message_count 메시지 수 저장 포인터
 * @param error_count 에러 수 저장 포인터
 * @return true: 성공, false: 실패
 */
bool uart_debug_get_statistics(uint32_t* bytes_sent, uint32_t* message_count, uint32_t* error_count)
{
    if (bytes_sent) *bytes_sent = uart_state.total_bytes_sent;
    if (message_count) *message_count = uart_state.message_count;
    if (error_count) *error_count = uart_state.error_count;

    return true;
}
