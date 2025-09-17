/**
 * @file pmic_service.c
 * @brief PMIC 서비스 모듈 구현 (Union 기반)
 * @note main_practice.c에서 이동된 PMIC 관련 기능들
 */

#include "pmic_service.h"
#include "dtc_manager.h"
#include <string.h>
#include <stdio.h>

// ========== 내부 변수 (main_practice.c에서 이동) ==========

// 🔄 main_practice.c의 I2C 핸들과 상태 변수들 → 여기로 이동
static I2C_HandleTypeDef *pmic_i2c_handle = NULL;
static pmic_service_state_t pmic_service_state = PMIC_SERVICE_IDLE;

// 🔄 main_practice.c의 레지스터 관련 변수들 → 여기로 이동  
static uint8_t current_reg_index = 0;
static uint8_t pmic_reg_address = 0;
uint8_t i2c_rx_buffer[1] = {0};  // extern으로 접근 가능하도록 static 제거

// 🔄 main_practice.c의 reg_addresses 배열 → 여기로 이동
static const mp5475_register_t reg_addresses[] = {
    MP5475_REG_SYSTEM_STATUS,	// 0x05
    MP5475_REG_POWER_GOOD,      // 0x06
    MP5475_REG_UV_OV_FAULT,     // 0x07
    MP5475_REG_OC_FAULT,        // 0x08 (추가)
    MP5475_REG_TEMP_FAULT       // 0x09
};

#define PMIC_REG_COUNT (sizeof(reg_addresses)/sizeof(reg_addresses[0]))

// Union 기반 PMIC 상태 데이터 저장소
static pmic_status_data_t current_pmic_status = {0};
static bool pmic_data_ready = false;

// ========== 내부 함수 선언 ==========
static HAL_StatusTypeDef pmic_start_register_read(uint8_t reg_addr);
static void pmic_process_register_data(uint8_t reg_addr, uint8_t reg_data);
static uint8_t pmic_analyze_uv_ov_faults(const pmic_uv_ov_fault_reg_t *uv_ov_reg);
static uint8_t pmic_analyze_oc_faults(const pmic_oc_fault_reg_t *oc_reg);
static uint8_t pmic_analyze_temp_faults(const pmic_temp_fault_reg_t *temp_reg);

// ========== 공개 함수 구현 ==========

/**
 * @brief PMIC 서비스 초기화
 */
bool pmic_service_init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) {
        printf("[PMIC] ERROR: Invalid I2C handle\n");
        return false;
    }
    
    printf("[PMIC] PMIC Service initialization started\n");
    
    pmic_i2c_handle = hi2c;
    pmic_service_state = PMIC_SERVICE_IDLE;
    current_reg_index = 0;
    pmic_data_ready = false;
    
    // Union 데이터 초기화
    memset(&current_pmic_status, 0, sizeof(current_pmic_status));
    memset(i2c_rx_buffer, 0, sizeof(i2c_rx_buffer));
    
    printf("[PMIC] PMIC Service initialized successfully\n");
    return true;
}

/**
 * @brief PMIC 진단 시작 (🔄 main_practice.c의 start_pmic_register_read 개선)
 */
bool pmic_service_start_diagnosis(void)
{
    if (pmic_service_state != PMIC_SERVICE_IDLE) {
        printf("[PMIC] Service busy, diagnosis ignored\n");
        return false;
    }
    
    printf("[PMIC] Starting PMIC diagnosis sequence\n");
    
    // 상태 초기화
    pmic_service_reset_state();
    current_reg_index = 0;
    pmic_data_ready = false;
    
    // 첫 번째 레지스터 읽기 시작
    if (pmic_start_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
        printf("[PMIC] Failed to start diagnosis\n");
        pmic_service_state = PMIC_SERVICE_ERROR;
        return false;
    }
    
    return true;
}

/**
 * @brief PMIC 상태 데이터 가져오기
 */
bool pmic_service_get_status(pmic_status_data_t *status_data)
{
    if (status_data == NULL) {
        return false;
    }
    
    if (!pmic_data_ready) {
        printf("[PMIC] Status data not ready\n");
        return false;
    }
    
    // Union 데이터 전체 복사
    memcpy(status_data, &current_pmic_status, sizeof(pmic_status_data_t));
    return true;
}

/**
 * @brief PMIC 서비스 상태 조회
 */
pmic_service_state_t pmic_service_get_state(void)
{
    return pmic_service_state;
}

/**
 * @brief Power Good 상태 확인 (Union 비트필드 활용)
 */
bool pmic_service_is_power_good(const pmic_status_data_t *status_data)
{
    if (status_data == NULL) {
        return false;
    }
    
    // Union 비트필드를 사용한 깔끔한 접근
    const pmic_power_good_reg_t *pg_reg = &status_data->regs.power_good;
    
    // 모든 Buck의 Power Good 필터링된 상태 확인
    return (pg_reg->bits.bucka_pg_filt && 
            pg_reg->bits.buckb_pg_filt && 
            pg_reg->bits.buckc_pg_filt && 
            pg_reg->bits.buckd_pg_filt);
}

/**
 * @brief PMIC 고장 분석 및 DTC 생성 (🔄 main_practice.c의 process_pmic_register_data 개선)
 */
uint8_t pmic_service_analyze_faults(const pmic_status_data_t *status_data)
{
    if (status_data == NULL) {
        return 0;
    }
    
    uint8_t dtc_count = 0;
    
    printf("[PMIC] Analyzing faults using Union bitfields\n");
    
    // 1. Power Good 상태 확인
    if (!pmic_service_is_power_good(status_data)) {
        printf("[PMIC] FAULT: Power Good failure detected\n");
        if (dtc_add_code(DTC_BRAKE_SYSTEM_FAULT)) {
            dtc_count++;
        }
    }
    
    // 2. UV/OV 고장 분석 (Union 비트필드 활용)
    dtc_count += pmic_analyze_uv_ov_faults(&status_data->regs.uv_ov_fault);
    
    // 3. OC 고장 분석 (Union 비트필드 활용)
    dtc_count += pmic_analyze_oc_faults(&status_data->regs.oc_fault);
    
    // 4. 온도 고장 분석 (Union 비트필드 활용)
    dtc_count += pmic_analyze_temp_faults(&status_data->regs.temp_fault);
    
    printf("[PMIC] Fault analysis completed: %d DTCs generated\n", dtc_count);
    return dtc_count;
}

/**
 * @brief 개별 레지스터 값 출력 (Union 활용)
 */
void pmic_service_print_registers(const pmic_status_data_t *status_data)
{
    if (status_data == NULL) {
        return;
    }
    
    printf("\n=== PMIC Registers (Union Format) ===\n");
    
    // 원시 데이터 출력
    printf("Raw Data: ");
    for (int i = 0; i < 8; i++) {
        printf("0x%02X ", status_data->raw_data[i]);
    }
    printf("\n");
    
    // 구조화된 레지스터 출력
    printf("System Status (0x05): 0x%02X\n", status_data->regs.system_status.raw);
    printf("  - PMR Good: %d, FSM State: %d\n", 
           status_data->regs.system_status.bits.pmr_good_filt,
           status_data->regs.system_status.bits.fsm_state);
    
    printf("Power Good (0x06): 0x%02X\n", status_data->regs.power_good.raw);
    printf("  - Buck A/B/C/D PG: %d/%d/%d/%d\n",
           status_data->regs.power_good.bits.bucka_pg_filt,
           status_data->regs.power_good.bits.buckb_pg_filt,
           status_data->regs.power_good.bits.buckc_pg_filt,
           status_data->regs.power_good.bits.buckd_pg_filt);
    
    printf("UV/OV Fault (0x07): 0x%02X\n", status_data->regs.uv_ov_fault.raw);
    printf("  - UV: A/B/C/D = %d/%d/%d/%d\n",
           status_data->regs.uv_ov_fault.bits.bucka_uv,
           status_data->regs.uv_ov_fault.bits.buckb_uv,
           status_data->regs.uv_ov_fault.bits.buckc_uv,
           status_data->regs.uv_ov_fault.bits.buckd_uv);
    printf("  - OV: A/B/C/D = %d/%d/%d/%d\n",
           status_data->regs.uv_ov_fault.bits.bucka_ov,
           status_data->regs.uv_ov_fault.bits.buckb_ov,
           status_data->regs.uv_ov_fault.bits.buckc_ov,
           status_data->regs.uv_ov_fault.bits.buckd_ov);
    
    printf("OC Fault (0x08): 0x%02X\n", status_data->regs.oc_fault.raw);
    printf("Temp Fault (0x09): 0x%02X\n", status_data->regs.temp_fault.raw);
    
    // 다른 접근 방식 (16비트, 32비트)
    printf("16-bit words: 0x%04X 0x%04X 0x%04X 0x%04X\n",
           status_data->word_data[0], status_data->word_data[1],
           status_data->word_data[2], status_data->word_data[3]);
    
    printf("64-bit full: 0x%016llX\n", status_data->full_data);
    printf("=====================================\n\n");
}

/**
 * @brief PMIC 상태 리셋
 */
void pmic_service_reset_state(void)
{
    printf("[PMIC] Resetting PMIC service state\n");
    
    current_reg_index = 0;
    pmic_reg_address = 0;
    pmic_data_ready = false;
    
    // Union 데이터 초기화
    memset(&current_pmic_status, 0, sizeof(current_pmic_status));
    memset(i2c_rx_buffer, 0, sizeof(i2c_rx_buffer));
}

// ========== 콜백 함수 구현 ==========

/**
 * @brief I2C DMA 수신 완료 콜백 처리 (🔄 main_practice.c에서 호출)
 */
void pmic_service_register_received(uint8_t reg_addr, uint8_t reg_data)
{
    printf("[PMIC] Register received: 0x%02X = 0x%02X\n", reg_addr, reg_data);
    
    // 레지스터 데이터 처리
    pmic_process_register_data(reg_addr, reg_data);
    
    // 다음 레지스터로 이동
    current_reg_index++;
    
    if (current_reg_index < PMIC_REG_COUNT) {
        // 다음 레지스터 읽기
        if (pmic_start_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
            pmic_service_state = PMIC_SERVICE_ERROR;
        }
    } else {
        // 모든 레지스터 읽기 완료
        printf("[PMIC] All registers read completed\n");
        pmic_service_state = PMIC_SERVICE_COMPLETE;
        pmic_data_ready = true;
        
        // 고장 분석 및 DTC 생성
        uint8_t fault_count = pmic_service_analyze_faults(&current_pmic_status);
        
        // 레지스터 상태 출력 (디버깅)
        pmic_service_print_registers(&current_pmic_status);
        
        // DTC 요약 출력
        if (fault_count > 0) {
            dtc_print_summary();
        }
        
        // 상태를 IDLE로 변경
        pmic_service_state = PMIC_SERVICE_IDLE;
    }
}

/**
 * @brief I2C DMA 에러 콜백 처리
 */
void pmic_service_communication_error(void)
{
    printf("[PMIC] Communication error occurred\n");
    
    pmic_service_state = PMIC_SERVICE_ERROR;
    
    // 통신 에러 DTC 추가
    dtc_add_code(DTC_BRAKE_COMM_ERROR);
    
    // 에러 후 상태 복구
    pmic_service_state = PMIC_SERVICE_IDLE;
}

// ========== 내부 함수 구현 ==========

/**
 * @brief 개별 레지스터 읽기 시작 (🔄 main_practice.c의 start_pmic_register_read)
 */
static HAL_StatusTypeDef pmic_start_register_read(uint8_t reg_addr)
{
    // 현재 레지스터 주소 저장
    pmic_reg_address = reg_addr;
    
    // 상태를 READING으로 변경
    pmic_service_state = PMIC_SERVICE_READING;
    
    // DMA 버퍼 초기화
    i2c_rx_buffer[0] = 0;
    
    printf("[PMIC] Starting DMA read - Register: 0x%02X\n", reg_addr);
    
    // HAL_I2C_Mem_Read_DMA 호출
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(pmic_i2c_handle,
                                                    (MP5475_I2C_ADDRESS << 1),
                                                    reg_addr,
                                                    I2C_MEMADD_SIZE_8BIT,
                                                    i2c_rx_buffer,
                                                    MP5475_REG_READ_SIZE);
    
    // 결과 확인
    if (status != HAL_OK) {
        printf("[PMIC] ERROR: Failed to start DMA read\n");
        pmic_service_state = PMIC_SERVICE_ERROR;
        return status;
    }

    return HAL_OK;
}

/**
 * @brief 레지스터 데이터 처리 (🔄 main_practice.c의 process_pmic_register_data 개선)
 */
static void pmic_process_register_data(uint8_t reg_addr, uint8_t reg_data)
{
    printf("[PMIC] Processing register 0x%02X with data 0x%02X\n", reg_addr, reg_data);
    
    // Union을 사용하여 레지스터별로 데이터 저장
    switch (reg_addr) {
        case MP5475_REG_SYSTEM_STATUS:  // 0x05
            current_pmic_status.regs.system_status.raw = reg_data;
            break;
            
        case MP5475_REG_POWER_GOOD:     // 0x06
            current_pmic_status.regs.power_good.raw = reg_data;
            break;
            
        case MP5475_REG_UV_OV_FAULT:    // 0x07
            current_pmic_status.regs.uv_ov_fault.raw = reg_data;
            break;
            
        case MP5475_REG_OC_FAULT:       // 0x08
            current_pmic_status.regs.oc_fault.raw = reg_data;
            break;
            
        case MP5475_REG_TEMP_FAULT:     // 0x09
            current_pmic_status.regs.temp_fault.raw = reg_data;
            break;
            
        default:
            printf("[PMIC] WARNING: Unknown register 0x%02X\n", reg_addr);
            break;
    }
}

/**
 * @brief UV/OV 고장 분석 (Union 비트필드 활용)
 */
static uint8_t pmic_analyze_uv_ov_faults(const pmic_uv_ov_fault_reg_t *uv_ov_reg)
{
    uint8_t dtc_count = 0;
    
    // Under Voltage 체크 (비트 7-4)
    if (uv_ov_reg->bits.bucka_uv || uv_ov_reg->bits.buckb_uv || 
        uv_ov_reg->bits.buckc_uv || uv_ov_reg->bits.buckd_uv) {
        printf("[PMIC] FAULT: Under Voltage detected - Raw: 0x%02X\n", uv_ov_reg->raw);
        printf("        UV Status: A=%d, B=%d, C=%d, D=%d\n",
               uv_ov_reg->bits.bucka_uv, uv_ov_reg->bits.buckb_uv,
               uv_ov_reg->bits.buckc_uv, uv_ov_reg->bits.buckd_uv);
        if (dtc_add_code(DTC_BRAKE_PMIC_UV)) {
            dtc_count++;
        }
    }
    
    // Over Voltage 체크 (비트 3-0)
    if (uv_ov_reg->bits.bucka_ov || uv_ov_reg->bits.buckb_ov || 
        uv_ov_reg->bits.buckc_ov || uv_ov_reg->bits.buckd_ov) {
        printf("[PMIC] FAULT: Over Voltage detected - Raw: 0x%02X\n", uv_ov_reg->raw);
        printf("        OV Status: A=%d, B=%d, C=%d, D=%d\n",
               uv_ov_reg->bits.bucka_ov, uv_ov_reg->bits.buckb_ov,
               uv_ov_reg->bits.buckc_ov, uv_ov_reg->bits.buckd_ov);
        if (dtc_add_code(DTC_BRAKE_PMIC_OV)) {
            dtc_count++;
        }
    }
    
    return dtc_count;
}

/**
 * @brief OC 고장 분석 (Union 비트필드 활용)
 */
static uint8_t pmic_analyze_oc_faults(const pmic_oc_fault_reg_t *oc_reg)
{
    uint8_t dtc_count = 0;
    
    // Over Current 체크 (비트 7-4)
    if (oc_reg->bits.bucka_oc || oc_reg->bits.buckb_oc || 
        oc_reg->bits.buckc_oc || oc_reg->bits.buckd_oc) {
        printf("[PMIC] FAULT: Over Current detected - Raw: 0x%02X\n", oc_reg->raw);
        printf("        OC Status: A=%d, B=%d, C=%d, D=%d\n",
               oc_reg->bits.bucka_oc, oc_reg->bits.buckb_oc,
               oc_reg->bits.buckc_oc, oc_reg->bits.buckd_oc);
        if (dtc_add_code(DTC_BRAKE_PMIC_OC)) {
            dtc_count++;
        }
    }
    
    // OC Warning 체크 (비트 3-0)
    if (oc_reg->bits.bucka_oc_warn || oc_reg->bits.buckb_oc_warn || 
        oc_reg->bits.buckc_oc_warn || oc_reg->bits.buckd_oc_warn) {
        printf("[PMIC] WARNING: Over Current Warning - Raw: 0x%02X\n", oc_reg->raw);
        printf("        OC Warning: A=%d, B=%d, C=%d, D=%d\n",
               oc_reg->bits.bucka_oc_warn, oc_reg->bits.buckb_oc_warn,
               oc_reg->bits.buckc_oc_warn, oc_reg->bits.buckd_oc_warn);
        // Warning은 DTC를 생성하지 않음 (선택사항)
    }
    
    return dtc_count;
}

/**
 * @brief 온도 고장 분석 (Union 비트필드 활용)
 */
static uint8_t pmic_analyze_temp_faults(const pmic_temp_fault_reg_t *temp_reg)
{
    uint8_t dtc_count = 0;
    
    // 온도 Shutdown 체크 (비트 0)
    if (temp_reg->bits.pmic_temp_shutdown) {
        printf("[PMIC] FAULT: High Temperature Shutdown - Raw: 0x%02X\n", temp_reg->raw);
        if (dtc_add_code(DTC_BRAKE_PMIC_TEMP)) {
            dtc_count++;
        }
    }
    
    // 온도 Warning 체크 (비트 1)
    if (temp_reg->bits.pmic_temp_warning) {
        printf("[PMIC] WARNING: High Temperature Warning - Raw: 0x%02X\n", temp_reg->raw);
        // Warning을 DTC로 처리할지는 설계에 따라 결정
    }
    
    // 기타 전압 고장들
    if (temp_reg->bits.vdrv_ov || temp_reg->bits.vbulk_ov) {
        printf("[PMIC] FAULT: Voltage fault (VDRV/VBULK) - Raw: 0x%02X\n", temp_reg->raw);
        if (dtc_add_code(DTC_BRAKE_SYSTEM_FAULT)) {
            dtc_count++;
        }
    }
    
    return dtc_count;
}