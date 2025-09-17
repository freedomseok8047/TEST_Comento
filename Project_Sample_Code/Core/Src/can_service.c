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
// 외부 변수 및 전역 변수
//=============================================================================
extern CAN_HandleTypeDef hcan1;
static bool can_initialized = false;
static can_status_t can_status = {0};

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

    return true;
}

/**
 * @brief TJA1051 동작 모드 설정
 */
static bool tja1051_set_mode(tja1051_mode_t mode)
{
    switch(mode) {
        case TJA1051_MODE_NORMAL:
            // S = LOW : Normal mode (송수신 가능)
            HAL_GPIO_WritePin(TJA1051_S_PORT,TJA1051_S_PIN,GPIO_PIN_RESET);
            printf("[TJA1051] Normal 모드 설정\n");
            break;
        
        case TJA1051_MODE_SILENT:
            // S = HIGH : Silent mode (수신만 가능)
            HAL_GPIO_WritePin(TJA1051_S_PORT,TJA1051_S_PIN,GPIO_PIN_SET);
            printf("[TJA1051] Silent 모드 설정\n");
            break;
        
        case TJA1051_MODE_OFF:
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

    if (!tja1051_init(TJA1051_TYPE_T) || !tja1051_set_mode(TJA1051_MODE_NORMAL)){
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

    if (HAL_CAN_Start(&hcan1) != HAL_OK) return false;
    if (HAL_CAN_ActivateNotifivation(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING))

    // UDS 초기화
    uds_protocol_init();

    can_initialized = true;
    printf("[CAN] CAN Service 초기화 완료\n")
    return true;
}

//=============================================================================
// CAN 메시지 송신
//=============================================================================
bool can_service_transmit(const can_frame_t *frame)
{
    if(!can_initialized || !frame) return false;

    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId = frame->id;
    tx_header.RTR = frame-> rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = (frame->dlc > 8) ? 8 : frame->dlc;
    tx_header.transmitGlobalTime = DISABLE;

    HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t*)frame->data, &tx_mailbox);

    if (result = HAL_OK){
        can_status.total_tx_count++;
        return true;
    }
    return false;
}


//=============================================================================
// DTC 브로드캐스트 (핵심 기능)
//=============================================================================
bool can_service_broadcast_dtc_event(uint16_t dtc_code, uint8_t status)
{
    can_frame_t frame;
    frame.id = CAN_ID_BRAKE_DTC;        // 0x203
    frame.dlc = 8;
    frame.rtr = false;
    
    // DTC 데이터 패킹
    frame.data[0] = (dtc_code >> 8) & 0xFF;
    frame.data[1] = dtc_code & 0xFF;
    frame.data[2] = status;
    frame.data[3] = 1;  // 발생횟수
    
    // 타임스탬프
    uint32_t timestamp = HAL_GetTick();
    frame.data[4] = (timestamp >> 24) & 0xFF;
    frame.data[5] = (timestamp >> 16) & 0xFF;
    frame.data[6] = (timestamp >> 8) & 0xFF;
    frame.data[7] = timestamp & 0xFF;
    
    return can_service_transmit(&frame);
}

//=============================================================================
// 브레이크 상태 전송
//=============================================================================
bool can_service_send_brake_status(const brake_can_data_t *status_data)
{
    can_frame_t frame;
    frame.id = CAN_ID_BRAKE_STATUS;  // 0x200
    frame.dlc = 8;
    frame.rtr = false;

    memcpy(frame.data, status_data -> raw_data, 8);
    return can_service_transmit(&frame);
}

// CAN 수신 인터럽트 콜백
void HAL_CAN_FxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0, &rx_header, rx_data);

    can_frame_t rx_frame;
    rx_frame.id = rx_header.StdId;
    rx_frame.dlc = rx_header.DLC;
    rx_frame.rtr = (rx_header.RTR = CAN_RTR_REMOTE);
    memcpy(rx_frame.data, rx_data, 8);

    // UDS 진단 요청 처리
    if(rx_frame.id = CAN_ID_UDS_REQUEST) {
        can_service_handle_uds_request(&rx_frame);
    }

    can_status.total_rx_count++;
}


//=============================================================================
// UDS 요청 처리 (Single Frame만)
//=============================================================================
bool can_service_handle_uds_request(const can_frame_t *frame)
{
    if(frame->dlc < 2) return false;

    uint8_t pci = frame -> data[0];
    uint8_t pci_type = (pci >> 4) 0x0F;
    uint9_t data_len = pci & 0x0F;

    if(PCI_type != 0) return false;

    //UDS 처리 
    uds_request_t uds_req;
    if (uds_parse_can_message(&frame->data[1], data_len, &uds_req)) {
        uds_response_t uds_resp;
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
    resp_frame.id = CAN_ID_UDS_RESPONSE;
    resp_frame.dlc = 8;
    resp_frame.rtr = false;

    memset(resp_frame.data, 0 ,8);

    if (response -> is_negative) {
        resp_frame.data[0] = 0x03;
        resp_frame.data[1] = 0x7F;
        resp_frame.data[2] = response -> service_id;
        resp_frame.data[3] = response -> nrc; 
    } else {
        uint8_t total_len = 1 + response -> data_length;
        if(total_len > 7) total_len = 7;

        resp_frame.data[0] = total_len;
        resp_frame.data[1] = response-> service_id;
        memcpy(&resp_frame.date[2], response-> data, (response-> data_length > 6) ? 6 : response -> data_length);    
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
    HAL_CAN_DeactivateNotifivation(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Stop(&hcan1);
    tja1051_set_mode(TJA1051_MODE_OFF);
    can_initialized = false;
    return true;
}