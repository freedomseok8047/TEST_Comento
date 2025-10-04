/**
 * @file can_service.c
 * @brief CAN Service 핵심 구현 - 브레이크 DTC 전송
 * @author Brake System Team
 * @date 2025-09-16
 */

#include "can_service.h"
#include "uds_protocol.h"
#include "dtc_manager.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

//=============================================================================
// 전역 변수
//=============================================================================
extern osMutexId CAN_MutexHandle;
extern CAN_HandleTypeDef hcan1;
static bool can_initialized = false;
static can_status_t can_status = {0};

//=============================================================================
// TJA1051 트랜시버 제어 함수들 (통합)
//=============================================================================

/**
 * @brief TJA1051 트랜시버 초기화
 */
static bool tja1051_init(void)
{
    // S 핀 (Silent 제어) GPIO 초기화
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = TJA1051_S_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TJA1051_S_PORT, &GPIO_InitStruct);
    
    // 기본적으로 Normal 모드로 설정
    HAL_GPIO_WritePin(TJA1051_S_PORT, TJA1051_S_PIN, GPIO_PIN_RESET);
    
    printf("[TJA1051] 트랜시버 초기화 완료\n");
    return true;
}

/**
 * @brief TJA1051 동작 모드 설정
 */
static bool tja1051_set_mode(tja1051_mode_t mode)
{
    switch(mode) {
        case TJA1051_MODE_NORMAL:
            // S = LOW: Normal mode (송수신 가능)
            HAL_GPIO_WritePin(TJA1051_S_PORT, TJA1051_S_PIN, GPIO_PIN_RESET);
            printf("[TJA1051] Normal 모드 설정\n");
            break;
            
        case TJA1051_MODE_SILENT:
            // S = HIGH: Silent mode (수신만 가능)
            HAL_GPIO_WritePin(TJA1051_S_PORT, TJA1051_S_PIN, GPIO_PIN_SET);
            printf("[TJA1051] Silent 모드 설정\n");
            break;
            
        case TJA1051_MODE_OFF:
            // TJA1051T 버전은 OFF 모드 없음 (항상 동작)
            printf("[TJA1051] OFF 모드는 지원되지 않음\n");
            return false;
    }
    return true;
}

//=============================================================================
// CAN Service 초기화
//=============================================================================
bool can_service_init(can_speed_t speed)
{
    printf("[CAN] CAN Service 초기화\n");
    
    // TJA1051 트랜시버 초기화 및 Normal 모드 설정
    if (!tja1051_init() || !tja1051_set_mode(TJA1051_MODE_NORMAL)) {
        return false;
    }
    
    // CAN 필터 설정 (브레이크 시스템 + UDS 진단)
    CAN_FilterTypeDef filter;
    filter.FilterIdHigh = (0x200 << 5);
    filter.FilterMaskIdHigh = (0x600 << 5);  // 0x200~0x7FF 범위
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    
    // CAN 시작 및 인터럽트 활성화
    if (HAL_CAN_Start(&hcan1) != HAL_OK) return false;
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) return false;
    
    // UDS 초기화
    uds_protocol_init();
    
    can_initialized = true;
    printf("[CAN] CAN Service 초기화 완료\n");
    return true;
}

//=============================================================================
// CAN 메시지 송신
//=============================================================================
bool can_service_transmit(const can_frame_t *frame)
{
    //유효성 검사
    if (!can_initialized || !frame) return false;
    
    // HAL CAN 구조체로 변환
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    
    // CAN 헤더 설정 
    tx_header.StdId = frame->id;                                    // 0x203
    tx_header.RTR = frame->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;     // CAN_RTR_DATA
    tx_header.IDE = CAN_ID_STD;                                     // 11비트 표준 ID
    tx_header.DLC = (frame->dlc > 8) ? 8 : frame->dlc;              // 8
    tx_header.TransmitGlobalTime = DISABLE;
    
    // HAL 라이브러리로 실제 전송
    HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan1, &tx_header, 
                                                    (uint8_t*)frame->data, &tx_mailbox);
    
    if (result == HAL_OK) {
        can_status.total_tx_count++;
        printf("[CAN] DTC transmitted successfully - ID: 0x%03X\n", frame->id);
        return true;
    } else {
        printf("[CAN] Transmission failed - Error: %d\n", result);
        return false;
    }
}

//=============================================================================
// DTC 브로드캐스트 (핵심 기능)
//=============================================================================
bool can_service_broadcast_dtc_event(uint16_t dtc_code, uint8_t status)
{
    can_frame_t frame; // CAN 프레임 구조체 생성

    //CAN 프레임 기본 정보 설정 
    frame.id = CAN_ID_BRAKE_DTC;        // 0x203 (브레이크 DTC ID)
    frame.dlc = 8;                      // 8바이트 데이터
    frame.rtr = false;                  // 데이터 프레임 (리모트 프레임 아님)
    
    // === 8바이트 CAN 데이터 패킹 (비트/바이트 연산) ===
    //데이터: [DTC코드 2(바이트)] [상태 1] [발생횟수 1] [타임스탬프 4]
    // [0][1] DTC 코드 분할 (16비트 → 2바이트)
    frame.data[0] = (dtc_code >> 8) & 0xFF; // 상위 8비트
    frame.data[1] = dtc_code & 0xFF;        // 하위 8비트
    
    /*비트 연산 상세:
    dtc_code = 0xC001 = 1100 0000 0000 0001 (16비트)
    (0xC001 >> 8) = 0x00C0 = 0000 0000 1100 0000
    상위 8비트
    (0x00C0 & 0xFF) = 0xC0 = 1100 0000
    frame.data[0] = 0xC0
    하위 8비트
    (0xC001 & 0xFF) = 0x01 = 0000 0001  
    frame.data[1] = 0x01*/

    // [2] DTC 상태
    frame.data[2] = status;

    // [3] 발생횟수 (하드코딩)
    frame.data[3] = 1;  
    
    // [4][5][6][7] 타임스탬프 분할 (32비트 → 4바이트)
    uint32_t timestamp = HAL_GetTick(); // HAL_GetTick() 시스템이 부팅된 후 지난 밀리초
    frame.data[4] = (timestamp >> 24) & 0xFF;
    frame.data[5] = (timestamp >> 16) & 0xFF;
    frame.data[6] = (timestamp >> 8) & 0xFF;
    frame.data[7] = timestamp & 0xFF;

    /* 타임스탬프 비트 연산 상세:
    예시) 25초 = 25000ms
    timestamp = 25000 = 0x000061A8 = 0000 0000 0000 0000 0110 0001 1010 1000
    
    (25000 >> 24) & 0xFF:
    25000 >> 24 = 0x00000000, & 0xFF = 0x00
    frame.data[4] = 0x00
    
    (25000 >> 16) & 0xFF:  
    25000 >> 16 = 0x00000006, & 0xFF = 0x00
    frame.data[5] = 0x00
    
    (25000 >> 8) & 0xFF:
    25000 >> 8 = 0x00000061, & 0xFF = 0x61
    frame.data[6] = 0x61
    
    25000 & 0xFF:
    25000 & 0xFF = 0xA8
    frame.data[7] = 0xA8
    
    데이터: [DTC코드 2(바이트)] [상태 1] [발생횟수 1] [타임스탬프 4]
    최종 CAN 데이터: [0xC0, 0x01, 0x01, 0x01, 0x00, 0x00, 0x61, 0xA8] */
    
    // 실제 CAN 전송 함수 호출
    return can_service_transmit(&frame);
}

//=============================================================================
// 브레이크 상태 전송
//=============================================================================
bool can_service_send_brake_status(const brake_can_data_t *status_data)
{
    can_frame_t frame;
    frame.id = CAN_ID_BRAKE_STATUS;     // 0x200
    frame.dlc = 8;
    frame.rtr = false;
    
    memcpy(frame.data, status_data->raw_data, 8);
    return can_service_transmit(&frame);
}

//=============================================================================
// CAN 수신 인터럽트 콜백
//=============================================================================
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
    
    can_frame_t rx_frame;
    rx_frame.id = rx_header.StdId;
    rx_frame.dlc = rx_header.DLC;
    rx_frame.rtr = (rx_header.RTR == CAN_RTR_REMOTE);
    memcpy(rx_frame.data, rx_data, 8);
    
    // UDS 진단 요청 처리
    if (rx_frame.id == CAN_ID_UDS_REQUEST) {
        can_service_handle_uds_request(&rx_frame);
    }
    
    can_status.total_rx_count++;
}


//=============================================================================
// UDS 요청 처리 (Single Frame만)
//=============================================================================
bool can_service_handle_uds_request(const can_frame_t *frame)
{
    if (frame->dlc < 2) return false;
    
    uint8_t pci = frame->data[0];
    uint8_t pci_type = (pci >> 4) & 0x0F;
    uint8_t data_len = pci & 0x0F;
    
    if (pci_type != 0) return false;  // Single Frame만 지원
    
    // UDS 요청 파싱
    uds_request_t uds_req;
    if (uds_parse_can_message(&frame->data[1], data_len, &uds_req)) {
        uds_response_t uds_resp;
        // UDS 프로토콜 모듈로 전달
        if (uds_process_request(&uds_req, &uds_resp)) {
            return can_service_send_uds_response(&uds_resp);
        }
    }
    return false;
}

//=============================================================================
// UDS 응답 전송
//=============================================================================
static bool can_service_send_uds_response(const uds_response_t *response)
{
    can_frame_t resp_frame;
    resp_frame.id = CAN_ID_UDS_RESPONSE;  // 0x7E8
    resp_frame.dlc = 8;
    resp_frame.rtr = false;
    
    memset(resp_frame.data, 0, 8);
    
    if (response->is_negative) {
        resp_frame.data[0] = 0x03;
        resp_frame.data[1] = 0x7F;
        resp_frame.data[2] = response->service_id;
        resp_frame.data[3] = response->nrc;
    } else {
        uint8_t total_len = 1 + response->data_length;
        if (total_len > 7) total_len = 7;
        
        resp_frame.data[0] = total_len;
        resp_frame.data[1] = response->service_id;
        memcpy(&resp_frame.data[2], response->data, 
               (response->data_length > 6) ? 6 : response->data_length);
    }
    
    return can_service_transmit(&resp_frame);
}

//=============================================================================
// 상태 조회 및 종료
//=============================================================================
bool can_service_get_status(can_status_t *status)
{
    if (!status) return false;
    *status = can_status;
    return true;
}

bool can_service_deinit(void)
{
    HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Stop(&hcan1);
    tja1051_set_mode(TJA1051_MODE_SILENT);  // Silent 모드로 전환
    can_initialized = false;
    return true;
}