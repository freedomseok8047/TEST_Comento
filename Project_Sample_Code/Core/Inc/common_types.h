#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// 기본 결과 타입
typedef enum {
    RESULT_OK = 0,
    RESULT_ERROR,
    RESULT_BUSY,
    RESULT_TIMEOUT
} result_t;

// 통신 상태 (기존 main_practice.c에서 사용중)
typedef enum {
    COMM_STATE_IDLE = 0,
    COMM_STATE_TX_BUSY,
    COMM_STATE_RX_BUSY, 
    COMM_STATE_COMPLETE,
    COMM_STATE_ERROR
} comm_state_t;

// 기존 main_practice.c의 i2c_state_t를 대체
typedef comm_state_t i2c_state_t;
#define I2C_STATE_IDLE      COMM_STATE_IDLE
#define I2C_STATE_RX_BUSY   COMM_STATE_RX_BUSY
#define I2C_STATE_RX_COMPLETE COMM_STATE_COMPLETE
#define I2C_STATE_ERROR     COMM_STATE_ERROR

// 기존 main_practice.c의 eeprom_state_t를 대체
typedef enum {
    EEPROM_STATE_IDLE = 0,
    EEPROM_STATE_WRITE_ENABLE,
    EEPROM_STATE_WRITING,
    EEPROM_STATE_READ_STATUS,
    EEPROM_STATE_COMPLETE,
    EEPROM_STATE_ERROR
} eeprom_state_t;

// DTC 관련 (기존 코드 호환)
typedef struct {
    uint16_t DTC_Code;
    char Description[50];
    uint8_t active;
} DTC_Table_t;

#endif // COMMON_TYPES_H