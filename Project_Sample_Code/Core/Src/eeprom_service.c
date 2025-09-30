/**
 * @file eeprom_service.c
 * @brief EEPROM 서비스 모듈 구현 (SPI + 25LC256 통합)
 * @note main_practice.c, spi_service.c, 25lc256.c에서 이동된 기능들을 간단히 통합
 */

#include "eeprom_service.h"
#include "dtc_manager.h"
#include <string.h>
#include <stdio.h>

// ========== 내부 변수 (main_practice.c에서 이동) ==========
static SPI_HandleTypeDef *eeprom_spi_handle = NULL;
static eeprom_service_state_t eeprom_service_state = EEPROM_SERVICE_IDLE;

// EEPROM 상태 관리 (main_practice.c에서 이동)
static volatile bool eeprom_dma_busy = false;
static volatile bool eeprom_write_in_progress = false;
static uint8_t current_dtc_save_index = 0;
static uint16_t current_eeprom_address = LC256_AREA_DTC_CURRENT;
static uint16_t dtc_log_count = 0;
static bool eeprom_is_init_mode = false;

// DMA 버퍼들 (main_practice.c에서 이동)
uint8_t eeprom_tx_buffer[67] __attribute__((aligned(4)));  // extern 접근용
uint8_t eeprom_rx_buffer[67] __attribute__((aligned(4)));  // extern 접근용
static eeprom_dtc_log_t current_dtc_log;

// DTC 저장 큐
static eeprom_dtc_log_t dtc_save_queue[6];
static uint8_t dtc_queue_count = 0;

// ========== 내부 함수 선언 ==========
static void eeprom_cs_select(void);
static void eeprom_cs_deselect(void);
static HAL_StatusTypeDef eeprom_write_enable_internal(void);
static HAL_StatusTypeDef eeprom_wip_check_internal(void);
static HAL_StatusTypeDef eeprom_write_dtc_log_internal(uint16_t address, const eeprom_dtc_log_t* dtc_log);
static bool eeprom_is_write_complete_internal(void);
static void convert_dtc_to_log(uint16_t dtc_code, const char* description, eeprom_dtc_log_t* log);
static uint16_t get_next_eeprom_address(void);

// ========== 공개 함수 구현 ==========

/**
 * @brief EEPROM 서비스 초기화
 */
bool eeprom_service_init(SPI_HandleTypeDef *spi_handle)
{
    if (spi_handle == NULL) {
        printf("[EEPROM] ERROR: Invalid SPI handle\n");
        return false;
    }
    
    printf("[EEPROM] EEPROM Service initialization started\n");
    
    eeprom_spi_handle = spi_handle;
    eeprom_service_state = EEPROM_SERVICE_IDLE;
    eeprom_dma_busy = false;
    eeprom_write_in_progress = false;
    
    // 변수 초기화
    current_dtc_save_index = 0;
    current_eeprom_address = LC256_AREA_DTC_CURRENT;
    dtc_log_count = 0;
    dtc_queue_count = 0;
    
    // 버퍼 초기화
    memset(eeprom_tx_buffer, 0, sizeof(eeprom_tx_buffer));
    memset(eeprom_rx_buffer, 0, sizeof(eeprom_rx_buffer));
    memset(dtc_save_queue, 0, sizeof(dtc_save_queue));
    
    // CS 핀 HIGH (비선택)
    eeprom_cs_deselect();
    
    // EEPROM 연결 확인
    printf("[EEPROM] Checking EEPROM connection...\n");
    eeprom_is_init_mode = true;
    
    if (eeprom_wip_check_internal() != HAL_OK) {
        printf("[EEPROM] ERROR: Connection check failed\n");
        return false;
    }
    
    // 초기화 완료 대기
    uint32_t timeout = HAL_GetTick() + 100;
    while (eeprom_dma_busy && (HAL_GetTick() < timeout)) {
        HAL_Delay(1);
    }
    
    printf("[EEPROM] EEPROM Service initialized successfully\n");
    return true;
}

/**
 * @brief DTC를 EEPROM에 저장
 */
bool eeprom_service_save_dtc(uint16_t dtc_code, const char* description)
{
    printf("[EEPROM] Save DTC: 0x%04X\n", dtc_code);
    
    if (dtc_queue_count >= 6) {
        printf("[EEPROM] ERROR: Save queue full\n");
        return false;
    }
    
    // 메모리상 DTC 정보를 EEPROM 저장에 최적화된 64바이트 고정 로그 형식으로 변환 (시간, 발생횟수, 설명 등등)
    convert_dtc_to_log(dtc_code, description, &dtc_save_queue[dtc_queue_count]);
    dtc_queue_count++;
    
    // IDLE 상태이면 즉시 저장 시작
    if (eeprom_service_state == EEPROM_SERVICE_IDLE && dtc_queue_count > 0) {
        current_dtc_save_index = 0;
        memcpy(&current_dtc_log, &dtc_save_queue[0], sizeof(eeprom_dtc_log_t));
        current_eeprom_address = get_next_eeprom_address();
        
        if (eeprom_write_enable_internal() != HAL_OK) {
            printf("[EEPROM] ERROR: Failed to start save\n");
            return false;
        }
    }
    
    return true;
}

/**
 * @brief EEPROM에서 DTC 읽기 (간단 구현)
 */
bool eeprom_service_read_dtc(uint8_t index, eeprom_dtc_log_t *dtc_log)
{
    if (dtc_log == NULL || eeprom_service_state != EEPROM_SERVICE_IDLE) {
        return false;
    }
    
    printf("[EEPROM] Reading DTC index: %d (simulated)\n", index);
    
    // 시뮬레이션 데이터
    memset(dtc_log, 0, sizeof(eeprom_dtc_log_t));
    dtc_log->DTC_Code = 0xC001;
    dtc_log->timestamp = HAL_GetTick();
    strcpy(dtc_log->Description, "Test DTC");
    dtc_log->active = 1;
    
    return true;
}

/**
 * @brief EEPROM에서 모든 DTC 읽기 (블로킹 방식)
 * @note UDS 요청 시 사용 - SPI 블로킹 읽기로 즉시 응답
 */
// ✅ 20250930 추가
uint8_t eeprom_service_read_all_dtc(eeprom_dtc_log_t *dtc_logs, uint8_t max_count)
{
    if (dtc_logs == NULL || max_count == 0) {
        printf("[EEPROM] Invalid parameters for read_all_dtc\n");
        return 0;
    }
    
    // EEPROM이 바쁘면 대기 (최대 100ms)
    uint32_t timeout = HAL_GetTick() + 100;
    while (eeprom_service_state != EEPROM_SERVICE_IDLE && HAL_GetTick() < timeout) {
        HAL_Delay(1);
    }
    
    if (eeprom_service_state != EEPROM_SERVICE_IDLE) {
        printf("[EEPROM] Service busy, cannot read DTCs\n");
        return 0;
    }
    
    // 실제 저장된 DTC 개수 확인
    uint16_t stored_count = dtc_log_count;
    if (stored_count == 0) {
        printf("[EEPROM] No DTCs stored in EEPROM\n");
        return 0;
    }
    
    // 읽을 개수 결정 (저장된 개수와 요청 개수 중 작은 값)
    uint8_t read_count = (stored_count < max_count) ? stored_count : max_count;
    
    printf("[EEPROM] Reading %d DTCs from EEPROM (blocking mode)\n", read_count);
    
    uint8_t success_count = 0;
    
    // 각 DTC를 순차적으로 읽기
    for (uint8_t i = 0; i < read_count; i++) {
        uint16_t address = LC256_AREA_DTC_CURRENT + (i * sizeof(eeprom_dtc_log_t));
        
        // 블로킹 SPI 읽기
        eeprom_cs_select();
        
        // READ 명령어 + 주소
        uint8_t cmd_buffer[3];
        cmd_buffer[0] = LC256_CMD_READ;
        cmd_buffer[1] = (address >> 8) & 0x7F;
        cmd_buffer[2] = address & 0xFF;
        
        // 명령어 전송 (블로킹)
        if (HAL_SPI_Transmit(eeprom_spi_handle, cmd_buffer, 3, 100) != HAL_OK) {
            printf("[EEPROM] Failed to send READ command for DTC %d\n", i);
            eeprom_cs_deselect();
            break;
        }
        
        // 데이터 수신 (블로킹)
        uint8_t read_buffer[sizeof(eeprom_dtc_log_t)];
        if (HAL_SPI_Receive(eeprom_spi_handle, read_buffer, sizeof(eeprom_dtc_log_t), 100) == HAL_OK) {
            // 데이터 복사
            memcpy(&dtc_logs[success_count], read_buffer, sizeof(eeprom_dtc_log_t));
            success_count++;
        } else {
            printf("[EEPROM] Failed to read data for DTC %d\n", i);
        }
        
        eeprom_cs_deselect();
        
        // 안정성을 위한 짧은 딜레이 (옵션)
        HAL_Delay(1);
    }
    
    printf("[EEPROM] Successfully read %d/%d DTCs from EEPROM\n", success_count, read_count);
    
    return success_count;
}

/**
 * @brief 모든 DTC 클리어
 */
bool eeprom_service_clear_all_dtc(void)
{
    printf("[EEPROM] Clearing all DTCs\n");
    
    dtc_log_count = 0;
    current_eeprom_address = LC256_AREA_DTC_CURRENT;
    dtc_queue_count = 0;
    memset(dtc_save_queue, 0, sizeof(dtc_save_queue));
    
    return true;
}

/**
 * @brief EEPROM 서비스 상태 조회
 */
eeprom_service_state_t eeprom_service_get_state(void)
{
    return eeprom_service_state;
}

/**
 * @brief Write Enable 명령
 */
bool eeprom_service_write_enable(void)
{
    return (eeprom_write_enable_internal() == HAL_OK);
}

/**
 * @brief WIP 비트 체크
 */
bool eeprom_service_is_write_in_progress(void)
{
    return (eeprom_rx_buffer[1] & 0x01) != 0;
}

/**
 * @brief 상태 레지스터 읽기
 */
bool eeprom_service_read_status(lc256_status_t *status)
{
    if (status == NULL) return false;
    
    status->raw = eeprom_rx_buffer[1];
    return true;
}

/**
 * @brief 저장된 DTC 개수
 */
uint16_t eeprom_service_get_dtc_count(void)
{
    return dtc_log_count;
}

/**
 * @brief 상태 출력
 */
void eeprom_service_print_status(void)
{
    printf("\n=== EEPROM Service Status ===\n");
    printf("State: %d, DMA Busy: %s\n", eeprom_service_state, eeprom_dma_busy ? "Yes" : "No");
    printf("Stored DTCs: %d, Queue: %d/6\n", dtc_log_count, dtc_queue_count);
    printf("============================\n\n");
}

// ========== 콜백 함수 구현 ==========

/**
 * @brief SPI 송신 완료 콜백 (main_practice.c에서 호출)
 */
void eeprom_service_tx_complete_callback(void)
{
    eeprom_cs_deselect();
    eeprom_dma_busy = false;

    switch (eeprom_service_state) {
        case EEPROM_SERVICE_WRITE_ENABLE:
            printf("[EEPROM] Write Enable completed\n");
            HAL_Delay(1);
            
            if (eeprom_write_dtc_log_internal(current_eeprom_address, &current_dtc_log) != HAL_OK) {
                eeprom_service_state = EEPROM_SERVICE_ERROR;
            }
            break;
            
        case EEPROM_SERVICE_WRITING:
            printf("[EEPROM] Write completed, checking status\n");
            HAL_Delay(LC256_WRITE_CYCLE_TIME); //쓰기 시간 만큼 딜레이
            
            //WIP(Write In Progress) 비트확인 쓰기완료여부 판단 
            // eeprom_wip_check_internal return값 HAL_OK 이면 저장 완료
            if (eeprom_wip_check_internal() != HAL_OK) { 
                eeprom_service_state = EEPROM_SERVICE_ERROR;
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief SPI 송수신 완료 콜백 (main_practice.c에서 호출)
 */
void eeprom_service_txrx_complete_callback(void)
{
    eeprom_cs_deselect();
    eeprom_dma_busy = false;

    // 초기화 모드
    if (eeprom_is_init_mode) {
        printf("[EEPROM] Init check: 0x%02X\n", eeprom_rx_buffer[1]);
        eeprom_is_init_mode = false;
        eeprom_service_state = EEPROM_SERVICE_IDLE;
        return;
    }

    // 상태 읽기 완료
    if (eeprom_service_state == EEPROM_SERVICE_READ_STATUS) {
        if (eeprom_is_write_complete_internal()) {
            printf("[EEPROM] Write completed successfully\n");
            
            dtc_log_count++;
            current_dtc_save_index++;

            // 큐에 더 저장할 DTC가 있는지 확인
            if (current_dtc_save_index < dtc_queue_count) {
                printf("[EEPROM] Processing next DTC (%d/%d)\n", 
                       current_dtc_save_index + 1, dtc_queue_count);
                
                memcpy(&current_dtc_log, &dtc_save_queue[current_dtc_save_index], sizeof(eeprom_dtc_log_t));
                current_eeprom_address = get_next_eeprom_address();
                
                if (eeprom_write_enable_internal() != HAL_OK) {
                    eeprom_service_state = EEPROM_SERVICE_ERROR;
                }
            } else {
                printf("[EEPROM] All DTCs saved successfully\n");
                
                // 큐 정리
                dtc_queue_count = 0;
                current_dtc_save_index = 0;
                memset(dtc_save_queue, 0, sizeof(dtc_save_queue));
                
                eeprom_service_state = EEPROM_SERVICE_IDLE;
            }
        } else {
            printf("[EEPROM] Write in progress, checking again\n");
            HAL_Delay(1);
            if (eeprom_wip_check_internal() != HAL_OK) {
                eeprom_service_state = EEPROM_SERVICE_ERROR;
            }
        }
    }
}

/**
 * @brief SPI 에러 콜백 (main_practice.c에서 호출)
 */
void eeprom_service_error_callback(void)
{
    printf("[EEPROM] SPI Error!\n");
    
    eeprom_cs_deselect();
    eeprom_service_state = EEPROM_SERVICE_ERROR;
    eeprom_dma_busy = false;
    eeprom_write_in_progress = false;
    
    // 에러 복구
    HAL_Delay(10);
    eeprom_service_state = EEPROM_SERVICE_IDLE;
}

// ========== 내부 함수 구현 ==========

static void eeprom_cs_select(void)
{
    HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_RESET);
}

static void eeprom_cs_deselect(void)
{
    HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_SET);
}

static HAL_StatusTypeDef eeprom_write_enable_internal(void)
{
    if (eeprom_dma_busy) return HAL_BUSY;

    eeprom_dma_busy = true;
    eeprom_tx_buffer[0] = LC256_CMD_WREN;
    
    eeprom_cs_select();
    eeprom_service_state = EEPROM_SERVICE_WRITE_ENABLE;
    
    HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(eeprom_spi_handle, eeprom_tx_buffer, 1);
    
    if (status != HAL_OK) {
        eeprom_cs_deselect();
        eeprom_service_state = EEPROM_SERVICE_ERROR;
        eeprom_dma_busy = false;
    }
    
    return status;
}

static HAL_StatusTypeDef eeprom_wip_check_internal(void)
{
    if (eeprom_dma_busy) return HAL_BUSY;

    eeprom_dma_busy = true;
    eeprom_tx_buffer[0] = LC256_CMD_RDSR;
    eeprom_tx_buffer[1] = 0x00;

    eeprom_cs_select();
    eeprom_service_state = EEPROM_SERVICE_READ_STATUS;
    
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(eeprom_spi_handle,
                                                           eeprom_tx_buffer,
                                                           eeprom_rx_buffer, 2);
    if (status != HAL_OK) {
        eeprom_cs_deselect();
        eeprom_service_state = EEPROM_SERVICE_ERROR;
        eeprom_dma_busy = false;
    }

    return status;
}

static HAL_StatusTypeDef eeprom_write_dtc_log_internal(uint16_t address, const eeprom_dtc_log_t* dtc_log)
{
    if (eeprom_dma_busy) return HAL_BUSY;

    eeprom_dma_busy = true;
    eeprom_write_in_progress = true;

    eeprom_tx_buffer[0] = LC256_CMD_WRITE;
    eeprom_tx_buffer[1] = (address >> 8) & 0x7F;
    eeprom_tx_buffer[2] = address & 0xFF;
    
    memcpy(&eeprom_tx_buffer[3], dtc_log, sizeof(eeprom_dtc_log_t));

    eeprom_cs_select();
    eeprom_service_state = EEPROM_SERVICE_WRITING;

    HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(eeprom_spi_handle,
                                                    eeprom_tx_buffer,
                                                    3 + sizeof(eeprom_dtc_log_t));
    if (status != HAL_OK) {
        eeprom_cs_deselect();
        eeprom_service_state = EEPROM_SERVICE_ERROR;
        eeprom_dma_busy = false;
        eeprom_write_in_progress = false;
    }
  
    return status;
}

static bool eeprom_is_write_complete_internal(void)
{
    return !(eeprom_rx_buffer[1] & 0x01);  // WIP = 0이면 완료
}

static void convert_dtc_to_log(uint16_t dtc_code, const char* description, eeprom_dtc_log_t* log)
{
    memset(log, 0, sizeof(eeprom_dtc_log_t));

    log->timestamp = HAL_GetTick();
    log->DTC_Code = dtc_code;
    
    if (description) {
        strncpy(log->Description, description, 39);
        log->Description[39] = '\0';
    } else {
        strcpy(log->Description, "Unknown DTC");
    }
    
    log->active = 1;
    log->occurrence_count = 1;
    log->status = 0x01;
}

static uint16_t get_next_eeprom_address(void)
{
    uint16_t next_addr = LC256_AREA_DTC_CURRENT + (dtc_log_count * sizeof(eeprom_dtc_log_t));

    if (next_addr + sizeof(eeprom_dtc_log_t) >= LC256_AREA_DTC_HISTORY) {
        next_addr = LC256_AREA_DTC_CURRENT;
        dtc_log_count = 0;
    }

    return next_addr;
}