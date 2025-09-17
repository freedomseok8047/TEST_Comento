
/**
 * @file 25lc256.h
 * @brief 25LC256 EEPROM SPI 제어 헤더 파일 (실제 데이터시트 기준)
 * @author JUNSEOK LEE
 * @date 2025-09-05
 *
 * @note 브레이크 시스템 DTC 저장용 EEPROM
 *       SPI DMA Interrupt 방식으로 동작
 *       총 용량: 256Kbit (32KB), Page Size: 64bytes
 *       데이터시트: 25LC256 Microchip 공식 문서 기준
 */

#ifndef LC256_H
#define LC256_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 25LC256 SPI 통신 설정
 */
#define LC256_SPI_SPEED_MAX     10000000    // 10MHz (VCC ≥ 4.5V)
#define LC256_SPI_SPEED_LOW     5000000     // 5MHz (2.5V ≤ VCC < 4.5V)
#define LC256_PAGE_SIZE         64          // 페이지 크기 (바이트)
#define LC256_TOTAL_SIZE        32768       // 전체 크기 (32KB = 256Kbit)
#define LC256_PAGE_COUNT        512         // 총 페이지 수 (32KB/64B)

/**
 * @brief 주소 관련 매크로 (15비트 주소)
 */
#define LC256_ADDRESS_MASK      0x7FFF      // 15비트 주소 마스크
#define LC256_ADDRESS_MAX       0x7FFF      // 최대 주소 (32767)
#define LC256_PAGE_MASK         0x7FC0      // 페이지 주소 마스크 (상위 9비트)
#define LC256_OFFSET_MASK       0x003F      // 페이지 내 오프셋 마스크 (하위 6비트)

/**
 * @brief 쓰기 사이클 시간
 */
#define LC256_WRITE_CYCLE_TIME  5           // 최대 5ms (데이터시트 기준)

/**
 * @brief 25LC256 SPI 명령어 정의
 * @note 데이터시트 TABLE 2-1: INSTRUCTION SET 기준
 */
typedef enum {
	LC256_CMD_READ          = 0x03,    // 0000 0011 - Read data from memory
	LC256_CMD_WRITE         = 0x02,    // 0000 0010 - Write data to memory
	LC256_CMD_WRDI          = 0x04,    // 0000 0100 - Write Disable
	LC256_CMD_WREN          = 0x06,    // 0000 0110 - Write Enable (요구사항 명시)
	LC256_CMD_RDSR          = 0x05,    // 0000 0101 - Read STATUS register
	LC256_CMD_WRSR          = 0x01     // 0000 0001 - Write STATUS register
} lc256_command_t;

/**
 * @brief 25LC256 상태 레지스터 비트필드 (데이터시트 기준)
 * @note WIP 비트 체크가 요구사항에 명시됨
 */
typedef struct {
    uint8_t wip             : 1;    // [0] Write In Progress (요구사항 핵심!)
    uint8_t wel             : 1;    // [1] Write Enable Latch
    uint8_t bp0             : 1;    // [2] Block Protect bit 0
    uint8_t bp1             : 1;    // [3] Block Protect bit 1
    uint8_t reserved        : 3;    // [4:6] 예약된 비트
    uint8_t wpen            : 1;    // [7] Write Protect Enable
} lc256_status_bits_t;

/**
 * @brief 25LC256 상태 레지스터 Union
 */
typedef union {
    uint8_t                 raw;        // 원시 데이터
    lc256_status_bits_t     bits;       // 비트필드 접근
} lc256_status_t;

/**
 * @brief 배열 보호 수준 정의 (데이터시트 TABLE 2-3: ARRAY PROTECTION)
 */
typedef enum {
    LC256_PROTECT_NONE      = 0x00,    // BP1=0, BP0=0: 보호 없음
    LC256_PROTECT_UPPER_1_4 = 0x04,    // BP1=0, BP0=1: 상위 1/4 (6000h-7FFFh)
    LC256_PROTECT_UPPER_1_2 = 0x08,    // BP1=1, BP0=0: 상위 1/2 (4000h-7FFFh)
    LC256_PROTECT_ALL       = 0x0C     // BP1=1, BP0=1: 전체 (0000h-7FFFh)
} lc256_protection_t;

/**
 * @brief DTC 엔트리 구조체 (8바이트)
 * @note 브레이크 시스템 진단 정보 저장
 */
typedef struct {
    uint16_t    dtc_code;           // DTC 코드 (예: 0xC001)
    uint32_t    timestamp;          // 발생 시간 (Unix timestamp)
    uint8_t     occurrence_count;   // 발생 횟수
    uint8_t     status;            // DTC 상태 (Active, Pending, etc.)
} __attribute__((packed)) dtc_entry_t;

/**
 * @brief EEPROM 페이지 데이터 구조체 (64바이트)
 * @note Page Write를 위한 구조체 (데이터시트 페이지 경계 제한 준수)
 */
typedef union {
    uint8_t         raw_data[LC256_PAGE_SIZE];      // 원시 데이터 (64바이트)
    dtc_entry_t     dtc_entries[8];                 // DTC 엔트리 8개 (8*8=64바이트)

    struct {
        uint32_t    magic_number;                   // 매직 넘버 (유효성 확인)
        uint16_t    version;                        // 데이터 버전
        uint16_t    checksum;                       // 체크섬
        dtc_entry_t dtc_data[7];                    // DTC 데이터 7개
    } __attribute__((packed)) structured;

} lc256_page_data_t;

/**
 * @brief EEPROM 메모리 영역 정의 (총 32KB)
 */
typedef enum {
    LC256_AREA_DTC_CURRENT      = 0x0000,   // 현재 DTC (0~4KB)
    LC256_AREA_DTC_HISTORY      = 0x1000,   // DTC 히스토리 (4~16KB)
    LC256_AREA_CONFIG           = 0x4000,   // 설정 데이터 (16~20KB)
    LC256_AREA_CALIBRATION      = 0x5000,   // 캘리브레이션 (20~24KB)
    LC256_AREA_USER             = 0x6000,   // 사용자 영역 (24~28KB)
    LC256_AREA_RESERVED         = 0x7000    // 예약 영역 (28~32KB)
} lc256_memory_area_t;

//=============================================================================
// Write-Protect 기능 매트릭스 (데이터시트 TABLE 2-1 기준)
//=============================================================================

/**
 * @brief Write-Protect 상태 구조체
 */
typedef struct {
    bool wel_bit;           // Write Enable Latch 상태
    bool wpen_bit;          // Write Protect Enable 상태
    bool wp_pin_low;        // WP 핀 상태 (Low = 보호 활성)
    lc256_protection_t protection_level;  // 보호 수준
} lc256_wp_status_t;

/**
 * @brief 25LC256 SPI 초기화
 * @return true: 성공, false: 실패
 */
bool lc256_init(void);

/**
 * @brief Write Enable 명령 실행 (요구사항 명시: WREN 0x06)
 * @return true: 성공, false: 실패
 */
bool lc256_write_enable(void);

/**
 * @brief Write Disable 명령 실행
 * @return true: 성공, false: 실패
 */
bool lc256_write_disable(void);

/**
 * @brief 상태 레지스터 읽기 (RDSR 0x05)
 * @param status 상태 데이터 저장 포인터
 * @return true: 성공, false: 실패
 */
bool lc256_read_status(lc256_status_t *status);

/**
 * @brief 상태 레지스터 쓰기 (WRSR 0x01)
 * @param status 설정할 상태 데이터
 * @return true: 성공, false: 실패
 */
bool lc256_write_status(lc256_status_t status);

/**
 * @brief WIP 비트 체크 (요구사항 명시)
 * @return true: Write In Progress, false: 완료됨
 */
bool lc256_is_write_in_progress(void);

/**
 * @brief Write Enable Latch 상태 확인
 * @return true: Write 가능, false: Write 불가
 */
bool lc256_is_write_enable(void);

/**
 * @brief 바이트 단위 읽기 (READ 0x03)
 * @param address 읽을 주소 (15비트, 0x0000~0x7FFF)
 * @param data 읽은 데이터 저장 포인터
 * @param length 읽을 바이트 수
 * @return true: 성공, false: 실패
 */
bool lc256_read_bytes(uint16_t address, uint8_t *data, uint16_t length);

/**
 * @brief 바이트 단위 쓰기 (WRITE 0x02)
 * @param address 쓸 주소 (15비트, 0x0000~0x7FFF)
 * @param data 쓸 데이터 포인터
 * @param length 쓸 바이트 수
 * @return true: 성공, false: 실패
 * @note 페이지 경계를 넘지 않도록 주의 (데이터시트 경고사항)
 */
bool lc256_write_bytes(uint16_t address, const uint8_t *data, uint16_t length);

/**
 * @brief 페이지 단위 쓰기 (요구사항: Page Write 사용)
 * @param page_addr 페이지 주소 (0~511)
 * @param page_data 페이지 데이터 포인터 (64바이트)
 * @return true: 성공, false: 실패
 * @note 64바이트 페이지 경계 내에서만 동작 (데이터시트 제한사항)
 */
bool lc256_write_page(uint16_t page_addr, const lc256_page_data_t *page_data);

/**
 * @brief 페이지 단위 읽기
 * @param page_addr 페이지 주소 (0~511)
 * @param page_data 페이지 데이터 저장 포인터 (64바이트)
 * @return true: 성공, false: 실패
 */
bool lc256_read_page(uint16_t page_addr, lc256_page_data_t *page_data);

/**
 * @brief Sequential Read (연속 읽기)
 * @param start_address 시작 주소
 * @param data 데이터 저장 포인터
 * @param length 읽을 바이트 수
 * @return true: 성공, false: 실패
 * @note 주소 0x7FFF 도달 시 0x0000으로 롤오버 (데이터시트 기능)
 */
bool lc256_sequential_read(uint16_t start_address, uint8_t *data, uint16_t length);

/**
 * @brief DTC 데이터 저장 (요구사항: DTC 저장/읽기)
 * @param dtc_code DTC 코드
 * @param dtc_data DTC 상세 정보
 * @return true: 성공, false: 실패
 */
bool lc256_store_dtc(uint16_t dtc_code, const dtc_entry_t *dtc_data);

/**
 * @brief DTC 데이터 읽기
 * @param dtc_code DTC 코드
 * @param dtc_data DTC 상세 정보 저장 포인터
 * @return true: 성공, false: 실패
 */
bool lc256_read_dtc(uint16_t dtc_code, dtc_entry_t *dtc_data);

/**
 * @brief 모든 DTC 삭제
 * @return true: 성공, false: 실패
 */
bool lc256_clear_all_dtc(void);

/**
 * @brief 블록 보호 설정 (데이터시트 TABLE 2-3 기준)
 * @param protection 보호 수준
 * @return true: 성공, false: 실패
 */
bool lc256_set_block_protection(lc256_protection_t protection);

/**
 * @brief Write-Protect 상태 확인
 * @param wp_status Write-Protect 상태 저장 포인터
 * @return true: 성공, false: 실패
 */
bool lc256_get_write_protect_status(lc256_wp_status_t *wp_status);

/**
 * @brief 쓰기 사이클 완료 대기 (최대 5ms)
 * @return true: 완료, false: 타임아웃
 */
bool lc256_wait_write_complete(void);

/**
 * @brief 페이지 경계 검사 (데이터시트 경고사항 준수)
 * @param address 시작 주소
 * @param length 데이터 길이
 * @return true: 페이지 경계 내, false: 경계 넘음
 */
bool lc256_check_page_boundary(uint16_t address, uint16_t length);

/**
 * @brief SPI DMA 완료 콜백 함수 (인터럽트에서 호출)
 */
void lc256_spi_dma_complete_callback(void);

/**
 * @brief SPI 에러 콜백 함수 (인터럽트에서 호출)
 */
void lc256_spi_error_callback(void);

#endif // LC256_H


























