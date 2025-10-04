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
    
    printf("[PMIC] Analyzing faults using Union bitfields\n");
    
    // ========== 1단계: 모든 고장 상태 분석 (DTC 생성 안함) ==========
    bool power_good_fail = false;
    bool uv_fault = false;
    bool ov_fault = false; 
    bool oc_fault = false;
    bool temp_fault = false;
    
    // Power Good 상태 분석
    if (!pmic_service_is_power_good(status_data)) {
        power_good_fail = true;
        printf("[PMIC] ANALYSIS: Power Good failure detected\n");
    }
    
    // UV/OV 고장 분석
    const pmic_uv_ov_fault_reg_t *uv_ov_reg = &status_data->regs.uv_ov_fault;
    if (uv_ov_reg->bits.bucka_uv || uv_ov_reg->bits.buckb_uv || 
        uv_ov_reg->bits.buckc_uv || uv_ov_reg->bits.buckd_uv) {
        uv_fault = true;
        printf("[PMIC] ANALYSIS: Under Voltage detected\n");
    }
    if (uv_ov_reg->bits.bucka_ov || uv_ov_reg->bits.buckb_ov || 
        uv_ov_reg->bits.buckc_ov || uv_ov_reg->bits.buckd_ov) {
        ov_fault = true;
        printf("[PMIC] ANALYSIS: Over Voltage detected\n");
    }
    
    // OC 고장 분석
    const pmic_oc_fault_reg_t *oc_reg = &status_data->regs.oc_fault;
    if (oc_reg->bits.bucka_oc || oc_reg->bits.buckb_oc || 
        oc_reg->bits.buckc_oc || oc_reg->bits.buckd_oc) {
        oc_fault = true;
        printf("[PMIC] ANALYSIS: Over Current detected\n");
    }
    
    // 온도 고장 분석
    const pmic_temp_fault_reg_t *temp_reg = &status_data->regs.temp_fault;
    if (temp_reg->bits.pmic_temp_shutdown) {
        temp_fault = true;
        printf("[PMIC] ANALYSIS: Temperature Shutdown detected\n");
    }
    
    // ========== 2단계: 우선순위에 따른 DTC 생성 ==========
    uint8_t dtc_count = 0;
    
    // 높은 우선순위 -> 내부정책 : 온도 셧다운 (가장 위험)  if ~ else 
    if (temp_fault) {
        printf("[PMIC] FAULT: High Temperature Shutdown\n");
        if (dtc_add_code(DTC_BRAKE_PMIC_TEMP)) {
            dtc_count++;
        }
    }
    
    // 전압/전류 고장
    if (uv_fault) {
        printf("[PMIC] FAULT: Under Voltage\n");
        if (dtc_add_code(DTC_BRAKE_PMIC_UV)) {
            dtc_count++;
        }
    }
    
    if (ov_fault) {
        printf("[PMIC] FAULT: Over Voltage\n");
        if (dtc_add_code(DTC_BRAKE_PMIC_OV)) {
            dtc_count++;
        }
    }
    
    if (oc_fault) {
        printf("[PMIC] FAULT: Over Current\n");
        if (dtc_add_code(DTC_BRAKE_PMIC_OC)) {
            dtc_count++;
        }
    }
    
    // 낮은 우선순위: Power Good (위의 고장들로 인한 결과일 수 있음)
    if (power_good_fail && dtc_count == 0) {  // 다른 고장이 없을 때만
        printf("[PMIC] FAULT: Power Good failure (no other faults detected)\n");
        if (dtc_add_code(DTC_BRAKE_SYSTEM_FAULT)) {
            dtc_count++;
        }
    }
    
    printf("[PMIC] Fault analysis completed: %d DTCs generated\n", dtc_count);
    return dtc_count;
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
void pmic_service_register_received(uint8_t reg_data)
{
    // 현재 읽고 있던 레지스터 주소 사용 (내부에서 관리)
    printf("[PMIC] Register received: 0x%02X = 0x%02X\n", pmic_reg_address, reg_data);
    
    // 레지스터 데이터 처리 - Union을 사용하여 레지스터별로 데이터 저장
    pmic_process_register_data(pmic_reg_address, reg_data);
    
    // 다음 레지스터로 이동
    current_reg_index++;
    
    if (current_reg_index < PMIC_REG_COUNT) {
        if (pmic_start_register_read(reg_addresses[current_reg_index]) != HAL_OK) {
            pmic_service_state = PMIC_SERVICE_ERROR;
        }
    } else {
        // 모든 레지스터 읽기 완료
        printf("[PMIC] All registers read completed\n");
        pmic_service_state = PMIC_SERVICE_COMPLETE;
        pmic_data_ready = true;
        
        // 고장 분석 시작 (수정된 버전)
        uint8_t fault_count = pmic_service_analyze_faults(&current_pmic_status);
        
        if (fault_count > 0) {
            dtc_print_summary();
        }
        
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