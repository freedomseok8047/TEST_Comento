/**
 * @file dtc_manager.c
 * @brief DTC 관리 모듈 구현
 * @note main_practice.c에서 이동된 DTC 관련 기능들
 */

#include "dtc_manager.h"
#include "eeprom_service.h"
#include <string.h>
#include <stdio.h>

// ========== 내부 변수 (main_practice.c에서 이동) ==========

// 🔄 main_practice.c의 dtc_master_table → 여기로 이동
static const DTC_Table_t dtc_master_table[DTC_MAX_COUNT] = {
    {DTC_BRAKE_PMIC_UV,     "Buck Undervoltage Fault",     0, 0, DTC_STATUS_INACTIVE, 0, 0},
    {DTC_BRAKE_PMIC_OV,     "Buck Overvoltage Fault",      0, 0, DTC_STATUS_INACTIVE, 0, 0},
    {DTC_BRAKE_PMIC_OC,     "Buck Overcurrent Fault",      0, 0, DTC_STATUS_INACTIVE, 0, 0},
    {DTC_BRAKE_PMIC_TEMP,   "PMIC Temperature Fault",      0, 0, DTC_STATUS_INACTIVE, 0, 0},
    {DTC_BRAKE_COMM_ERROR,  "I2C Communication Error",     0, 0, DTC_STATUS_INACTIVE, 0, 0},
    {DTC_BRAKE_SYSTEM_FAULT,"System/Power Good Fault",     0, 0, DTC_STATUS_INACTIVE, 0, 0}
};

// 🔄 main_practice.c의 detected_dtc_table, dtc_count → 여기로 이동
static DTC_Table_t detected_dtc_table[DTC_MAX_COUNT];
static uint8_t dtc_count = 0;

// ========== 내부 함수 선언 ==========
static bool dtc_find_master_entry(uint16_t dtc_code, const DTC_Table_t** master_entry);
static int dtc_find_detected_index(uint16_t dtc_code);
static void dtc_copy_from_master(const DTC_Table_t* master, DTC_Table_t* detected);

// ========== 공개 함수 구현 ==========

/**
 * @brief DTC 매니저 초기화
 */
bool dtc_manager_init(void)
{
    printf("[DTC] DTC Manager initialization started\n");
    
    // detected_dtc_table 초기화
    memset(detected_dtc_table, 0, sizeof(detected_dtc_table));
    dtc_count = 0;
    
    printf("[DTC] DTC Manager initialized successfully\n");
    return true;
}

/**
 * @brief DTC 추가 (🔄 main_practice.c의 add_dtc_code 개선 버전)
 */
bool dtc_add_code(uint16_t dtc_code)
{
    // 1단계: 중복 DTC 확인
    if (dtc_find_detected_index(dtc_code) >= 0) {
        // 이미 존재하는 DTC - 발생 횟수만 증가
        int index = dtc_find_detected_index(dtc_code);
        detected_dtc_table[index].occurrence_count++;
        detected_dtc_table[index].last_occurrence = HAL_GetTick();
        printf("[DTC] DTC 0x%04X occurrence count updated: %d\n", 
               dtc_code, detected_dtc_table[index].occurrence_count);
        
        // EEPROM에 업데이트된 정보 저장
        eeprom_service_save_dtc(dtc_code, detected_dtc_table[index].Description);
        
        return true;
    }

    // 2단계: DTC 테이블 공간 확인
    if (dtc_count >= DTC_MAX_COUNT) {
        printf("[DTC] ERROR: DTC table full (max: %d)\n", DTC_MAX_COUNT);
        return false;
    }

    // 3단계: 마스터 테이블에서 DTC 정보 찾기
    const DTC_Table_t* master_entry = NULL;
    if (!dtc_find_master_entry(dtc_code, &master_entry)) {
        printf("[DTC] ERROR: Unknown DTC code: 0x%04X\n", dtc_code);
        return false;
    }

    // 4단계: detected_dtc_table에 추가
    dtc_copy_from_master(master_entry, &detected_dtc_table[dtc_count]);
    
    // 새로운 DTC 설정
    detected_dtc_table[dtc_count].active = 1;
    detected_dtc_table[dtc_count].occurrence_count = 1;
    detected_dtc_table[dtc_count].status = DTC_STATUS_ACTIVE;
    detected_dtc_table[dtc_count].first_occurrence = HAL_GetTick();
    detected_dtc_table[dtc_count].last_occurrence = HAL_GetTick();

    printf("[DTC] Added: 0x%04X - %s\n", dtc_code, master_entry->Description);
    dtc_count++;
    
    // 5단계: 자동으로 EEPROM에 저장
    printf("[DTC] Auto-saving to EEPROM...\n");
    if (!eeprom_service_save_dtc(dtc_code, master_entry->Description)) {
        printf("[DTC] WARNING: Failed to save DTC to EEPROM\n");
        // DTC 추가는 성공이지만 EEPROM 저장 실패를 알림
    } else {
        printf("[DTC] DTC automatically saved to EEPROM\n");
    }
    
    return true;
}

/**
 * @brief 모든 DTC 클리어 (UDS Service 0x14)
 */
bool dtc_clear_all(void)
{
    printf("[DTC] Clearing all DTCs (UDS Service 0x14)\n");
    
    for (uint8_t i = 0; i < dtc_count; i++) {
        detected_dtc_table[i].active = 0;
        detected_dtc_table[i].status = DTC_STATUS_INACTIVE;
        printf("[DTC] Cleared: 0x%04X\n", detected_dtc_table[i].DTC_Code);
    }
    
    // 테이블 초기화
    memset(detected_dtc_table, 0, sizeof(detected_dtc_table));
    dtc_count = 0;
    
    // EEPROM에서도 클리어
    printf("[DTC] Clearing DTCs from EEPROM...\n");
    if (!eeprom_service_clear_all_dtc()) {
        printf("[DTC] WARNING: Failed to clear DTCs from EEPROM\n");
    } else {
        printf("[DTC] DTCs cleared from EEPROM successfully\n");
    }
    
    printf("[DTC] All DTCs cleared successfully\n");
    return true;
}

/**
 * @brief 특정 DTC 클리어
 */
bool dtc_clear_specific(uint16_t dtc_code)
{
    int index = dtc_find_detected_index(dtc_code);
    if (index < 0) {
        printf("[DTC] DTC 0x%04X not found for clearing\n", dtc_code);
        return false;
    }
    
    detected_dtc_table[index].active = 0;
    detected_dtc_table[index].status = DTC_STATUS_INACTIVE;
    
    printf("[DTC] Cleared specific DTC: 0x%04X\n", dtc_code);
    return true;
}

/**
 * @brief 활성 DTC 개수 조회
 */
uint8_t dtc_get_active_count(void)
{
    uint8_t active_count = 0;
    for (uint8_t i = 0; i < dtc_count; i++) {
        if (detected_dtc_table[i].active) {
            active_count++;
        }
    }
    return active_count;
}

/**
 * @brief DTC 목록 조회
 */
uint8_t dtc_get_list(DTC_Table_t* dtc_list, uint8_t max_count)
{
    if (dtc_list == NULL) {
        return 0;
    }
    
    uint8_t copy_count = (dtc_count < max_count) ? dtc_count : max_count;
    memcpy(dtc_list, detected_dtc_table, copy_count * sizeof(DTC_Table_t));
    
    return copy_count;
}

/**
 * @brief DTC 요약 출력 (🔄 main_practice.c의 print_dtc_summary 개선 버전)
 */
void dtc_print_summary(void)
{
    printf("\n========= DTC SUMMARY (UDS Format) ===========\n");
    printf("Total detected DTCs: %d\n", dtc_count);
    printf("Active DTCs: %d\n", dtc_get_active_count());
    printf("EEPROM stored DTCs: %d\n", eeprom_service_get_dtc_count());
    
    if (dtc_count == 0) {
        printf("No faults detected - System OK\n");
    } else {
        printf("\nDetailed DTC Information:\n");
        for (uint8_t i = 0; i < dtc_count; i++) {
            printf("DTC[%d]: 0x%04X - %s\n", 
                   i + 1,
                   detected_dtc_table[i].DTC_Code,
                   detected_dtc_table[i].Description);
            printf("        Status: 0x%02X, Count: %d, Active: %s\n",
                   detected_dtc_table[i].status,
                   detected_dtc_table[i].occurrence_count,
                   detected_dtc_table[i].active ? "Yes" : "No");
            printf("        First: %lu ms, Last: %lu ms\n",
                   detected_dtc_table[i].first_occurrence,
                   detected_dtc_table[i].last_occurrence);
        }
    }
    
    // EEPROM 상태 출력
    printf("\nEEPROM Storage Status:\n");
    eeprom_service_print_status();
    
    printf("=============================================\n\n");
}

/**
 * @brief 진단 상태 리셋 (🔄 main_practice.c의 reset_pmic_diagnosis 일부)
 */
void dtc_reset_diagnosis(void)
{
    printf("[DTC] Resetting diagnosis state\n");
    
    // 기존 DTC들을 pending 상태로 변경 (완전히 삭제하지 않음)
    for (uint8_t i = 0; i < dtc_count; i++) {
        if (detected_dtc_table[i].status == DTC_STATUS_ACTIVE) {
            detected_dtc_table[i].status = DTC_STATUS_PENDING;
        }
    }
    
    printf("[DTC] Diagnosis state reset completed\n");
}

/**
 * @brief DTC 상태 업데이트
 */
void dtc_update_status(uint16_t dtc_code, dtc_status_t new_status)
{
    int index = dtc_find_detected_index(dtc_code);
    if (index >= 0) {
        detected_dtc_table[index].status = new_status;
        detected_dtc_table[index].last_occurrence = HAL_GetTick();
        printf("[DTC] Updated DTC 0x%04X status to 0x%02X\n", dtc_code, new_status);
    }
}

/**
 * @brief UDS 포맷으로 DTC 데이터 생성
 */
uint16_t dtc_create_uds_response(uint8_t* buffer, uint16_t buffer_size)
{
    if (buffer == NULL || buffer_size < 3) {
        return 0;
    }
    
    uint16_t offset = 0;
    
    // UDS Response: Service ID + DTC Count + DTC Data
    buffer[offset++] = 0x59;  // Positive response for service 0x19
    buffer[offset++] = dtc_get_active_count();  // Active DTC count
    
    // DTC 데이터 추가 (3바이트씩: DTC Code + Status)
    for (uint8_t i = 0; i < dtc_count && (offset + 3) <= buffer_size; i++) {
        if (detected_dtc_table[i].active) {
            buffer[offset++] = (detected_dtc_table[i].DTC_Code >> 8) & 0xFF;
            buffer[offset++] = detected_dtc_table[i].DTC_Code & 0xFF;
            buffer[offset++] = detected_dtc_table[i].status;
        }
    }
    
    return offset;
}

// ========== 내부 함수 구현 ==========

/**
 * @brief 마스터 테이블에서 DTC 엔트리 찾기
 */
static bool dtc_find_master_entry(uint16_t dtc_code, const DTC_Table_t** master_entry)
{
    for (uint8_t i = 0; i < DTC_MAX_COUNT; i++) {
        if (dtc_master_table[i].DTC_Code == dtc_code) {
            *master_entry = &dtc_master_table[i];
            return true;
        }
    }
    return false;
}

/**
 * @brief detected_dtc_table에서 DTC 인덱스 찾기
 */
static int dtc_find_detected_index(uint16_t dtc_code)
{
    for (uint8_t i = 0; i < dtc_count; i++) {
        if (detected_dtc_table[i].DTC_Code == dtc_code) {
            return i;
        }
    }
    return -1;  // 찾지 못함
}

/**
 * @brief 마스터에서 detected로 DTC 정보 복사
 */
static void dtc_copy_from_master(const DTC_Table_t* master, DTC_Table_t* detected)
{
    detected->DTC_Code = master->DTC_Code;
    strncpy(detected->Description, master->Description, DTC_DESCRIPTION_SIZE - 1);
    detected->Description[DTC_DESCRIPTION_SIZE - 1] = '\0';  // NULL 종료 보장
}