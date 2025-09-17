/**
 * @file eeprom_service.h
 * @brief EEPROM 서비스 모듈 - SPI + 25LC256 통합
 * @author Brake System Team
 * @date 2025-09-15
 * 
 * @note SPI DMA + 25LC256 EEPROM을 하나의 서비스로 통합
 *       main_practice.c에서 EEPROM/SPI 관련 기능 분리
 */

#ifndef EEPROM_SERVICE_H
#define EEPROM_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "common_types.h"
#include "stm32f4xx_hal.h"

// ========== 25LC256 EEPROM 설정 (25lc256.h에서 이동) ==========
#define LC256_SPI_SPEED_MAX     10000000    // 10MHz (VCC ≥ 4.5V)
#define LC256_PAGE_SIZE         64          // 페이지 크기 (바이트)
#define LC256_TOTAL_SIZE        32768       // 전체 크기 (32KB = 256Kbit)
#define LC256_PAGE_COUNT        512         // 총 페이지 수 (32KB/64B)
#define LC256_WRITE_CYCLE_TIME  5           // 최대 5ms (데이터시트 기준)

// ========== 25LC256 SPI 명령어 (25lc256.h에서 이동) ==========
typedef enum {
    LC256_CMD_READ          = 0x03,    // Read data from memory
    LC256_CMD_WRITE         = 0x02,    // Write data to memory
    LC256_CMD_WRDI          = 0x04,    // Write Disable
    LC256_CMD_WREN          = 0x06,    // Write Enable
    LC256_CMD_RDSR          = 0x05,    // Read STATUS register
    LC256_CMD_WRSR          = 0x01     // Write STATUS register
} lc256_command_t;

// ========== EEPROM 메모리 영역 정의 (25lc256.h에서 이동) ==========
typedef enum {
    LC256_AREA_DTC_CURRENT      = 0x0000,   // 현재 DTC (0~4KB)
    LC256_AREA_DTC_HISTORY      = 0x1000,   // DTC 히스토리 (4~16KB)
    LC256_AREA_CONFIG           = 0x4000,   // 설정 데이터 (16~20KB)
    LC256_AREA_CALIBRATION      = 0x5000,   // 캘리브레이션 (20~24KB)
    LC256_AREA_USER             = 0x6000,   // 사용자 영역 (24~28KB)
    LC256_AREA_RESERVED         = 0x7000    // 예약 영역 (28~32KB)
} lc256_memory_area_t;

// ========== GPIO 핀 정의 (main_practice.c에서 이동) ==========
#define EEPROM_CS_PORT    GPIOB
#define EEPROM_CS_PIN     GPIO_PIN_4

// ========== EEPROM 상태 머신 (main_practice.c에서 이동/개선) ==========
typedef enum {
    EEPROM_SERVICE_IDLE = 0,
    EEPROM_SERVICE_WRITE_ENABLE,
    EEPROM_SERVICE_WRITING,
    EEPROM_SERVICE_READ_STATUS,
    EEPROM_SERVICE_READING,
    EEPROM_SERVICE_COMPLETE,
    EEPROM_SERVICE_ERROR
} eeprom_service_state_t;

// ========== DTC 로그 구조체 (main_practice.c에서 이동/개선) ==========
#pragma pack(push, 1)
typedef struct {
    uint32_t timestamp;         // 발생 시간
    uint16_t DTC_Code;         // DTC 코드
    char Description[40];       // 설명 (48 → 40으로 축소)
    uint8_t active;            // 활성화 상태
    uint8_t occurrence_count;   // 발생 횟수
    uint8_t status;            // DTC 상태
    uint8_t reserved[9];       // 예약된 공간 (64바이트 맞춤)
} eeprom_dtc_log_t;
#pragma pack(pop)

// ========== 25LC256 상태 레지스터 Union (25lc256.h에서 이동) ==========
typedef union {
    uint8_t raw;
    struct {
        uint8_t wip             : 1;    // [0] Write In Progress
        uint8_t wel             : 1;    // [1] Write Enable Latch
        uint8_t bp0             : 1;    // [2] Block Protect bit 0
        uint8_t bp1             : 1;    // [3] Block Protect bit 1
        uint8_t reserved        : 3;    // [4:6] 예약된 비트
        uint8_t wpen            : 1;    // [7] Write Protect Enable
    } bits;
} lc256_status_t;

// ========== 공개 함수 선언 ==========

/**
 * @brief EEPROM 서비스 초기화 (SPI + EEPROM 통합)
 * @param spi_handle SPI 핸들 포인터
 * @return true: 성공, false: 실패
 */
bool eeprom_service_init(SPI_HandleTypeDef *spi_handle);

/**
 * @brief DTC를 EEPROM에 저장 (main_practice.c의 save_dtc_to_eeprom 대체)
 * @param dtc_code DTC 코드
 * @param description DTC 설명
 * @return true: 성공, false: 실패
 */
bool eeprom_service_save_dtc(uint16_t dtc_code, const char* description);

/**
 * @brief EEPROM에서 DTC 읽기
 * @param index DTC 인덱스 (0부터 시작)
 * @param dtc_log DTC 로그 저장 포인터
 * @return true: 성공, false: 실패
 */
bool eeprom_service_read_dtc(uint8_t index, eeprom_dtc_log_t *dtc_log);

/**
 * @brief 모든 DTC 클리어
 * @return true: 성공, false: 실패
 */
bool eeprom_service_clear_all_dtc(void);

/**
 * @brief EEPROM 서비스 상태 조회
 * @return 현재 상태
 */
eeprom_service_state_t eeprom_service_get_state(void);

/**
 * @brief EEPROM Write Enable 명령 (내부 사용)
 * @return true: 성공, false: 실패
 */
bool eeprom_service_write_enable(void);

/**
 * @brief EEPROM WIP 비트 체크 (내부 사용)
 * @return true: 쓰기 진행 중, false: 완료됨
 */
bool eeprom_service_is_write_in_progress(void);

/**
 * @brief EEPROM 상태 레지스터 읽기
 * @param status 상태 저장 포인터
 * @return true: 성공, false: 실패
 */
bool eeprom_service_read_status(lc256_status_t *status);

/**
 * @brief DTC 저장 개수 조회
 * @return 저장된 DTC 개수
 */
uint16_t eeprom_service_get_dtc_count(void);

/**
 * @brief EEPROM 서비스 상태 출력 (디버깅용)
 */
void eeprom_service_print_status(void);

// ========== 콜백 함수들 (main_practice.c에서 호출) ==========

/**
 * @brief SPI DMA 송신 완료 콜백 처리
 */
void eeprom_service_tx_complete_callback(void);

/**
 * @brief SPI DMA 송수신 완료 콜백 처리
 */
void eeprom_service_txrx_complete_callback(void);

/**
 * @brief SPI DMA 에러 콜백 처리
 */
void eeprom_service_error_callback(void);

#endif // EEPROM_SERVICE_H