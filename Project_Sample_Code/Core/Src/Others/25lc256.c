/**
 * @file 25lc256.c
 * @brief 25LC256 EEPROM SPI DMA Interrupt 방식 구현
 * @author Brake System Team
 * @date 2025-09-05
 *
 * @note SPI DMA Interrupt 방식으로 EEPROM에 DTC 데이터를 저장/읽기합니다.
 *       DMA를 사용하여 대용량 데이터 전송 시 CPU 부하를 최소화합니다.
 */

#include "25lc256.h"
#include <string.h>
#include <stdio.h>

//=============================================================================
// 전역 변수 (SPI DMA 동작을 위한 상태 관리)
//=============================================================================

/**
 * @brief SPI DMA 동작 상태 구조체
 */
typedef struct {
    volatile bool           dma_busy;           // DMA 전송 진행 중 플래그
    volatile bool           dma_complete;       // DMA 전송 완료 플래그
    volatile bool           dma_error;          // DMA 에러 발생 플래그
    volatile bool           write_in_progress;  // EEPROM 쓰기 진행 중
    lc256_status_t         last_status;         // 마지막 상태 레지스터 값
    uint32_t               total_operations;    // 총 동작 횟수
    uint32_t               error_count;         // 누적 에러 카운터
} spi_dma_state_t;

// SPI DMA 상태 변수 (정적으로 선언하여 인터럽트에서 접근 가능)
static spi_dma_state_t spi_state = {0};

// DMA 송신용 버퍼 (명령어 + 주소 + 데이터)
static uint8_t spi_tx_buffer[67] __attribute__((aligned(4))); // 1(CMD) + 2(ADDR) + 64(DATA)

// DMA 수신용 버퍼
static uint8_t spi_rx_buffer[67] __attribute__((aligned(4)));

// CS 핀 제어를 위한 매크로 (실제로는 GPIO HAL 사용)
#define SPI_CS_LOW()    printf("[SPI-DMA] CS = LOW\n")
#define SPI_CS_HIGH()   printf("[SPI-DMA] CS = HIGH\n")

//=============================================================================
// SPI DMA 초기화 함수
//=============================================================================

/**
 * @brief 25LC256 SPI DMA 초기화
 * @return true: 성공, false: 실패
 *
 * @note SPI 하드웨어, DMA 채널, CS 핀을 설정합니다.
 */
bool lc256_init(void)
{
    printf("[SPI-DMA] 25LC256 초기화 시작...\n");

    // 1단계: SPI 하드웨어 초기화
    // - SPI 모드: Master, Mode 0 (CPOL=0, CPHA=0)
    // - 클럭: 10MHz (VCC >= 4.5V 시)
    // - 데이터 크기: 8비트
    // - MSB First
    /*
    실제 코드 예시 (STM32 HAL):
    hspi.Instance = SPI1;
    hspi.Init.Mode = SPI_MODE_MASTER;
    hspi.Init.Direction = SPI_DIRECTION_2LINES;
    hspi.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 10MHz
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    HAL_SPI_Init(&hspi);
    */

    // 2단계: DMA 채널 초기화 (송신/수신 각각)
    /*
    실제 코드 예시:
    hdma_spi_tx.Instance = DMA1_Channel3;
    hdma_spi_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_tx.Init.MemInc = DMA_MINC_ENABLE;
    HAL_DMA_Init(&hdma_spi_tx);

    hdma_spi_rx.Instance = DMA1_Channel2;
    hdma_spi_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    HAL_DMA_Init(&hdma_spi_rx);
    */

    // 3단계: CS 핀 초기화 (GPIO)
    /*
    실제 코드 예시:
    GPIO_InitStruct.Pin = SPI_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI_CS_PORT, &GPIO_InitStruct);
    */

    // 4단계: 상태 변수 초기화
    memset(&spi_state, 0, sizeof(spi_state));
    memset(spi_tx_buffer, 0, sizeof(spi_tx_buffer));
    memset(spi_rx_buffer, 0, sizeof(spi_rx_buffer));

    // 5단계: EEPROM 연결 확인 (Device ID 읽기)
    uint8_t device_id = 0;
    if (!lc256_read_device_id(&device_id)) {
        printf("[SPI-DMA] EEPROM 연결 확인 실패!\n");
        return false;
    }

    printf("[SPI-DMA] 25LC256 초기화 완료 (Device ID: 0x%02X)\n", device_id);
    return true;
}

//=============================================================================
// Write Enable 명령 (요구사항: WREN 0x06)
//=============================================================================

/**
 * @brief Write Enable 명령 실행 (DMA 방식)
 * @return true: 성공, false: 실패
 *
 * @note 모든 쓰기 동작 전에 반드시 실행해야 합니다.
 *       DMA를 사용하여 빠른 명령 전송을 수행합니다.
 */
bool lc256_write_enable(void)
{
    // DMA 사용 가능 여부 확인
    if (spi_state.dma_busy) {
        printf("[SPI-DMA] DMA 사용 중, WREN 명령 대기\n");
        return false;
    }

    printf("[SPI-DMA] Write Enable 명령 시작\n");

    // 1단계: DMA 상태 설정
    spi_state.dma_busy = true;
    spi_state.dma_complete = false;
    spi_state.dma_error = false;

    // 2단계: 송신 버퍼 준비 (WREN 명령)
    spi_tx_buffer[0] = LC256_CMD_WREN;  // 0x06

    // 3단계: CS 신호 LOW (전송 시작)
    SPI_CS_LOW();

    // 4단계: SPI DMA 송신 시작
    /*
    실제 코드 예시 (STM32 HAL):
    HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(
        &hspi,              // SPI 핸들
        spi_tx_buffer,      // 송신 데이터
        1                   // 송신 바이트 수
    );

    if (status != HAL_OK) {
        SPI_CS_HIGH();
        spi_state.dma_busy = false;
        spi_state.dma_error = true;
        return false;
    }
    */

    // 시뮬레이션: DMA 완료
    lc256_simulate_wren_complete();

    return true;
}

//=============================================================================
// 상태 레지스터 읽기 (WIP 비트 체크)
//=============================================================================

/**
 * @brief 상태 레지스터 읽기 (DMA 방식)
 * @param status 상태 데이터 저장 포인터
 * @return true: 성공, false: 실패
 *
 * @note WIP 비트를 확인하여 쓰기 완료 여부를 판단합니다.
 */
bool lc256_read_status(lc256_status_t *status)
{
    if (spi_state.dma_busy) {
        printf("[SPI-DMA] DMA 사용 중, 상태 읽기 대기\n");
        return false;
    }

    printf("[SPI-DMA] 상태 레지스터 읽기 시작\n");

    // 1단계: DMA 상태 설정
    spi_state.dma_busy = true;
    spi_state.dma_complete = false;

    // 2단계: 송신 버퍼 준비 (RDSR 명령 + 더미 바이트)
    spi_tx_buffer[0] = LC256_CMD_RDSR;  // 0x05
    spi_tx_buffer[1] = 0x00;            // 더미 바이트 (상태 수신용)

    // 3단계: CS LOW
    SPI_CS_LOW();

    // 4단계: SPI DMA 송수신 시작
    /*
    실제 코드 예시:
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
        &hspi,
        spi_tx_buffer,      // 송신 데이터
        spi_rx_buffer,      // 수신 데이터
        2                   // 송수신 바이트 수
    );
    */

    // 시뮬레이션: 정상 상태 반환
    spi_rx_buffer[1] = 0x00;  // WIP=0, WEL=0 (쓰기 완료됨)
    spi_state.last_status.raw = spi_rx_buffer[1];

    if (status != NULL) {
        *status = spi_state.last_status;
    }

    SPI_CS_HIGH();
    spi_state.dma_complete = true;
    spi_state.dma_busy = false;

    printf("[SPI-DMA] 상태 읽기 완료: 0x%02X\n", spi_state.last_status.raw);
    return true;
}

/**
 * @brief WIP 비트 체크 (요구사항 명시)
 * @return true: Write In Progress, false: 완료됨
 */
bool lc256_is_write_in_progress(void)
{
    lc256_status_t status;
    if (lc256_read_status(&status)) {
        return status.bits.wip;
    }
    return false; // 읽기 실패 시 완료된 것으로 간주
}

//=============================================================================
// 페이지 쓰기 (요구사항: Page Write 사용)
//=============================================================================

/**
 * @brief 페이지 단위 쓰기 (DMA 방식)
 * @param page_addr 페이지 주소 (0~511)
 * @param page_data 페이지 데이터 포인터 (64바이트)
 * @return true: 성공, false: 실패
 *
 * @note 64바이트 페이지 경계를 준수하여 쓰기를 수행합니다.
 *       DMA를 사용하여 대용량 데이터를 효율적으로 전송합니다.
 */
bool lc256_write_page(uint16_t page_addr, const lc256_page_data_t *page_data)
{
    if (spi_state.dma_busy) {
        printf("[SPI-DMA] DMA 사용 중, 페이지 쓰기 대기\n");
        return false;
    }

    // 1단계: 페이지 주소 유효성 확인
    if (page_addr >= LC256_PAGE_COUNT) {
        printf("[SPI-DMA] 잘못된 페이지 주소: %d\n", page_addr);
        return false;
    }

    printf("[SPI-DMA] 페이지 쓰기 시작 (페이지: %d)\n", page_addr);

    // 2단계: Write Enable 실행
    if (!lc256_write_enable()) {
        printf("[SPI-DMA] Write Enable 실패\n");
        return false;
    }

    // 3단계: DMA 상태 설정
    spi_state.dma_busy = true;
    spi_state.dma_complete = false;
    spi_state.write_in_progress = true;

    // 4단계: 송신 버퍼 준비
    uint16_t byte_addr = page_addr * LC256_PAGE_SIZE;

    spi_tx_buffer[0] = LC256_CMD_WRITE;         // WRITE 명령 (0x02)
    spi_tx_buffer[1] = (byte_addr >> 8) & 0x7F; // 주소 상위 바이트 (15비트)
    spi_tx_buffer[2] = byte_addr & 0xFF;        // 주소 하위 바이트

    // 페이지 데이터 복사 (64바이트)
    memcpy(&spi_tx_buffer[3], page_data->raw_data, LC256_PAGE_SIZE);

    // 5단계: CS LOW
    SPI_CS_LOW();

    // 6단계: SPI DMA 송신 시작 (CMD + ADDR + DATA = 67바이트)
    /*
    실제 코드 예시:
    HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(
        &hspi,
        spi_tx_buffer,
        3 + LC256_PAGE_SIZE     // 명령(1) + 주소(2) + 데이터(64)
    );

    if (status != HAL_OK) {
        SPI_CS_HIGH();
        spi_state.dma_busy = false;
        spi_state.dma_error = true;
        return false;
    }
    */

    // 시뮬레이션: 쓰기 완료
    lc256_simulate_write_complete();

    return true;
}

//=============================================================================
// 페이지 읽기
//=============================================================================

/**
 * @brief 페이지 단위 읽기 (DMA 방식)
 * @param page_addr 페이지 주소 (0~511)
 * @param page_data 페이지 데이터 저장 포인터 (64바이트)
 * @return true: 성공, false: 실패
 */
bool lc256_read_page(uint16_t page_addr, lc256_page_data_t *page_data)
{
    if (spi_state.dma_busy) {
        printf("[SPI-DMA] DMA 사용 중, 페이지 읽기 대기\n");
        return false;
    }

    if (page_addr >= LC256_PAGE_COUNT) {
        printf("[SPI-DMA] 잘못된 페이지 주소: %d\n", page_addr);
        return false;
    }

    printf("[SPI-DMA] 페이지 읽기 시작 (페이지: %d)\n", page_addr);

    // 1단계: DMA 상태 설정
    spi_state.dma_busy = true;
    spi_state.dma_complete = false;

    // 2단계: 송신 버퍼 준비 (READ 명령 + 주소 + 더미 바이트들)
    uint16_t byte_addr = page_addr * LC256_PAGE_SIZE;

    spi_tx_buffer[0] = LC256_CMD_READ;          // READ 명령 (0x03)
    spi_tx_buffer[1] = (byte_addr >> 8) & 0x7F; // 주소 상위 바이트
    spi_tx_buffer[2] = byte_addr & 0xFF;        // 주소 하위 바이트

    // 더미 바이트들 (읽기용)
    memset(&spi_tx_buffer[3], 0x00, LC256_PAGE_SIZE);

    // 3단계: CS LOW
    SPI_CS_LOW();

    // 4단계: SPI DMA 송수신 시작
    /*
    실제 코드 예시:
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
        &hspi,
        spi_tx_buffer,      // 송신 (명령 + 주소 + 더미)
        spi_rx_buffer,      // 수신 (더미 + 실제 데이터)
        3 + LC256_PAGE_SIZE
    );
    */

    // 시뮬레이션: 읽기 완료
    // 수신 버퍼의 처음 3바이트는 더미, 나머지가 실제 데이터
    memset(&spi_rx_buffer[3], 0xAA, LC256_PAGE_SIZE); // 시뮬레이션 데이터

    if (page_data != NULL) {
        memcpy(page_data->raw_data, &spi_rx_buffer[3], LC256_PAGE_SIZE);
    }

    SPI_CS_HIGH();
    spi_state.dma_complete = true;
    spi_state.dma_busy = false;
    spi_state.total_operations++;

    printf("[SPI-DMA] 페이지 읽기 완료\n");
    return true;
}

//=============================================================================
// DTC 데이터 저장/읽기 (요구사항)
//=============================================================================

/**
 * @brief DTC 데이터 저장
 * @param dtc_code DTC 코드
 * @param dtc_data DTC 상세 정보
 * @return true: 성공, false: 실패
 *
 * @note DTC를 EEPROM의 DTC 영역에 저장합니다.
 */
bool lc256_store_dtc(uint16_t dtc_code, const dtc_entry_t *dtc_data)
{
    printf("[SPI-DMA] DTC 저장 시작: 0x%04X\n", dtc_code);

    // 1단계: DTC 저장 위치 계산 (현재 DTC 영역 사용)
    uint16_t page_addr = LC256_AREA_DTC_CURRENT / LC256_PAGE_SIZE;

    // 2단계: 페이지 데이터 준비
    lc256_page_data_t page;
    memset(&page, 0, sizeof(page));

    // 매직 넘버와 버전 설정
    page.structured.magic_number = 0xDTC12345;
    page.structured.version = 0x0001;

    // DTC 데이터 복사
    if (dtc_data != NULL) {
        memcpy(&page.structured.dtc_data[0], dtc_data, sizeof(dtc_entry_t));
    } else {
        // 기본 DTC 엔트리 생성
        page.structured.dtc_data[0].dtc_code = dtc_code;
        page.structured.dtc_data[0].timestamp = 0x12345678; // 시뮬레이션
        page.structured.dtc_data[0].occurrence_count = 1;
        page.structured.dtc_data[0].status = 0x01; // Active
    }

    // 3단계: 체크섬 계산 (간단한 XOR 체크섬)
    uint16_t checksum = 0;
    for (int i = 0; i < sizeof(dtc_entry_t); i++) {
        checksum ^= ((uint8_t*)&page.structured.dtc_data[0])[i];
    }
    page.structured.checksum = checksum;

    // 4단계: 페이지 쓰기 실행
    bool result = lc256_write_page(page_addr, &page);

    if (result) {
        printf("[SPI-DMA] DTC 저장 완료: 0x%04X\n", dtc_code);
    } else {
        printf("[SPI-DMA] DTC 저장 실패: 0x%04X\n", dtc_code);
    }

    return result;
}

/**
 * @brief DTC 데이터 읽기
 * @param dtc_code DTC 코드
 * @param dtc_data DTC 상세 정보 저장 포인터
 * @return true: 성공, false: 실패
 */
bool lc256_read_dtc(uint16_t dtc_code, dtc_entry_t *dtc_data)
{
    printf("[SPI-DMA] DTC 읽기 시작: 0x%04X\n", dtc_code);

    // 1단계: DTC 저장 위치에서 페이지 읽기
    uint16_t page_addr = LC256_AREA_DTC_CURRENT / LC256_PAGE_SIZE;

    lc256_page_data_t page;
    if (!lc256_read_page(page_addr, &page)) {
        printf("[SPI-DMA] 페이지 읽기 실패\n");
        return false;
    }

    // 2단계: 매직 넘버 확인
    if (page.structured.magic_number != 0xDTC12345) {
        printf("[SPI-DMA] 잘못된 매직 넘버: 0x%08lX\n", page.structured.magic_number);
        return false;
    }

    // 3단계: 체크섬 확인
    uint16_t checksum = 0;
    for (int i = 0; i < sizeof(dtc_entry_t); i++) {
        checksum ^= ((uint8_t*)&page.structured.dtc_data[0])[i];
    }

    if (checksum != page.structured.checksum) {
        printf("[SPI-DMA] 체크섬 오류\n");
        return false;
    }

    // 4단계: DTC 데이터 복사
    if (dtc_data != NULL) {
        memcpy(dtc_data, &page.structured.dtc_data[0], sizeof(dtc_entry_t));
    }

    printf("[SPI-DMA] DTC 읽기 완료: 0x%04X\n", page.structured.dtc_data[0].dtc_code);
    return true;
}

//=============================================================================
// SPI DMA 콜백 함수 (인터럽트에서 호출)
//=============================================================================

/**
 * @brief SPI DMA 송신 완료 콜백
 * @note 송신 완료 후 CS를 HIGH로 설정하고 완료 플래그를 설정합니다.
 */
void lc256_spi_tx_dma_complete_callback(void)
{
    printf("[SPI-DMA] 송신 완료\n");

    // CS 신호 HIGH (전송 종료)
    SPI_CS_HIGH();

    // 완료 플래그 설정
    spi_state.dma_complete = true;
    spi_state.dma_busy = false;
    spi_state.total_operations++;

    // 쓰기 동작이었다면 WIP 상태로 전환
    if (spi_state.write_in_progress) {
        printf("[SPI-DMA] 쓰기 사이클 시작 (최대 5ms)\n");
        // 실제로는 타이머를 설정하여 5ms 후 WIP 체크
    }
}

/**
 * @brief SPI DMA 수신 완료 콜백
 */
void lc256_spi_rx_dma_complete_callback(void)
{
    printf("[SPI-DMA] 수신 완료\n");
    lc256_spi_tx_dma_complete_callback(); // 공통 처리
}

/**
 * @brief SPI DMA 에러 콜백
 */
void lc256_spi_error_callback(void)
{
    printf("[SPI-DMA] 에러 발생!\n");

    // CS 신호 HIGH (안전한 상태로 복귀)
    SPI_CS_HIGH();

    // 에러 플래그 설정
    spi_state.dma_error = true;
    spi_state.dma_busy = false;
    spi_state.error_count++;

    printf("[SPI-DMA] 에러 복구 (에러: %lu회)\n", spi_state.error_count);
}

//=============================================================================
// 시뮬레이션 함수 (실제 구현에서는 제거)
//=============================================================================

/**
 * @brief Write Enable 완료 시뮬레이션
 */
void lc256_simulate_wren_complete(void)
{
    SPI_CS_HIGH();
    spi_state.dma_complete = true;
    spi_state.dma_busy = false;
    printf("[SPI-DMA] WREN 명령 완료\n");
}

/**
 * @brief 쓰기 완료 시뮬레이션
 */
void lc256_simulate_write_complete(void)
{
    SPI_CS_HIGH();
    spi_state.dma_complete = true;
    spi_state.dma_busy = false;
    spi_state.write_in_progress = false;
    spi_state.total_operations++;
    printf("[SPI-DMA] 페이지 쓰기 완료\n");
}

/**
 * @brief Device ID 읽기 (연결 확인용)
 */
bool lc256_read_device_id(uint8_t *device_id)
{
    printf("[SPI-DMA] Device ID 확인 중...\n");

    if (device_id != NULL) {
        *device_id = 0x29; // 25LC256 시뮬레이션 ID
    }

    printf("[SPI-DMA] Device ID: 0x29 (정상)\n");
    return true;
}

/**
 * @brief 현재 상태 출력 (디버깅용)
 */
void lc256_print_status(void)
{
    printf("\n=== 25LC256 SPI DMA 상태 ===\n");
    printf("DMA 사용 중: %s\n", spi_state.dma_busy ? "예" : "아니오");
    printf("쓰기 진행 중: %s\n", spi_state.write_in_progress ? "예" : "아니오");
    printf("총 동작 횟수: %lu\n", spi_state.total_operations);
    printf("에러 횟수: %lu\n", spi_state.error_count);
    printf("마지막 상태: 0x%02X\n", spi_state.last_status.raw);
    printf("==========================\n\n");
}
