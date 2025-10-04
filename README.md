# RTOS 기반 브레이크 ECU 시스템 동작 구조

## 목차
1. [초기화 단계](#1-초기화-단계)
2. [RTOS Task 구조](#2-rtos-task-구조-및-우선순위)
3. [PMIC 고장 감지 타임라인](#3-pmic-고장-감지--dtc-생성--can-전송-타임라인)
4. [UDS 진단 처리](#4-uds-진단-요청-처리-흐름)
5. [타이밍 요약](#5-주요-타이밍-요약)
6. [동기화 메커니즘](#6-mutex-보호-영역)
7. [시스템 상태 머신](#7-시스템-상태-머신)
8. [데이터 흐름](#8-데이터-흐름-다이어그램)
9. [메모리 맵](#9-메모리-맵)

---

## 1. 초기화 단계

시스템 부팅부터 RTOS 스케줄러 시작까지의 흐름입니다.

```
시스템 부팅
    ↓
HAL_Init()
    └─ 하드웨어 초기화
    └─ 클럭 설정
    └─ GPIO, DMA 초기화
    ↓
서비스 모듈 초기화 (순서 중요):
    ① dtc_manager_init()
        └─ DTC 테이블 초기화
        └─ 마스터 테이블 로드
    ② pmic_service_init(&hi2c1)
        └─ I2C1 핸들 연결
        └─ 상태 머신 초기화
    ③ uds_protocol_init()
        └─ 진단 세션 설정
        └─ 통계 변수 초기화
    ④ eeprom_service_init(&hspi1)
        └─ SPI1 핸들 연결
        └─ 25LC256 연결 확인
    ⑤ can_service_init(CAN_SPEED_500K)
        └─ TJA1051 트랜시버 초기화
        └─ CAN 필터 설정
    ↓
EEPROM에서 DTC 복원:
    └─ eeprom_service_get_dtc_count()
    └─ eeprom_service_read_dtc() (최대 6개)
    └─ dtc_add_code() (메모리에 복원)
    ↓
Task Manager 초기화:
    ① Mutex 생성
        ├─ DTC_MutexHandle
        ├─ EEPROM_MutexHandle
        └─ CAN_MutexHandle
    ② Event Flags 생성
        └─ SystemEventsHandle
    ③ Task 생성
        ├─ Task_1ms (osPriorityHigh)
        ├─ Task_5ms (osPriorityNormal)
        └─ Task_PMIC_Monitor (osPriorityRealtime)
    ↓
osKernelStart()
    └─ RTOS 스케줄러 시작
    └─ 제어권이 Task들로 이동
```

---

## 2. RTOS Task 구조 및 우선순위

3개의 Task가 우선순위에 따라 동작합니다.

```
┌─────────────────────────────────────────────────────────────┐
│  Task_PMIC_Monitor (osPriorityRealtime - 최고 우선순위)     │
├─────────────────────────────────────────────────────────────┤
│  동작 방식: 이벤트 대기 (블로킹)                             │
│                                                               │
│  osEventFlagsWait(EVENT_PMIC_IRQ | EVENT_EMERGENCY)          │
│      ↓                                                        │
│  if (EVENT_PMIC_IRQ):                                        │
│      └─ pmic_service_start_diagnosis()                       │
│         └─ I2C DMA로 레지스터 5개 순차 읽기                  │
│                                                               │
│  if (EVENT_EMERGENCY):                                       │
│      └─ dtc_add_code(DTC_BRAKE_SYSTEM_FAULT)                 │
│         └─ 긴급 상황 처리                                     │
└─────────────────────────────────────────────────────────────┘
                            ↑
                    Event Flags 트리거
                            ↑
┌─────────────────────────────────────────────────────────────┐
│  Task_1ms (osPriorityHigh - 빠른 샘플링)                    │
├─────────────────────────────────────────────────────────────┤
│  주기: 1ms (osDelay(1))                                      │
│                                                               │
│  동작:                                                        │
│  ① ADC 빠른 샘플링                                           │
│     └─ HAL_ADC_Start() → HAL_ADC_PollForConversion(1ms)     │
│     └─ 위험 전압 감지 시:                                    │
│        └─ task_set_emergency_event()                         │
│                                                               │
│  ② PMIC IRQ 핀 직접 체크                                     │
│     └─ HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)                   │
│     └─ LOW 감지 시:                                          │
│        └─ task_set_pmic_irq_event()                          │
│                                                               │
│  ③ 1초마다 상태 출력 (tick_count % 1000)                    │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  Task_5ms (osPriorityNormal - 주기적 작업)                  │
├─────────────────────────────────────────────────────────────┤
│  주기: 5ms (osDelay(5))                                      │
│                                                               │
│  동작:                                                        │
│  ① PMIC 진단 결과 확인 (매 5ms)                             │
│     └─ if (pmic_service_get_state() == COMPLETE):           │
│        ├─ osMutexWait(DTC_MutexHandle)                       │
│        ├─ pmic_service_analyze_faults(&pmic_data)           │
│        └─ osMutexRelease(DTC_MutexHandle)                    │
│                                                               │
│  ② CAN 주기 전송 (100ms = 20 * 5ms)                         │
│     └─ if (tick_count % 20 == 0):                           │
│        ├─ brake_status 데이터 준비                           │
│        ├─ osMutexWait(CAN_MutexHandle)                       │
│        ├─ can_service_send_brake_status()                    │
│        └─ osMutexRelease(CAN_MutexHandle)                    │
│                                                               │
│  ③ 시스템 상태 출력 (1초 = 200 * 5ms)                       │
│     └─ if (tick_count % 200 == 0):                          │
│        └─ printf("DTCs: %d\n", dtc_get_active_count())      │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. PMIC 고장 감지 → DTC 생성 → CAN 전송 타임라인

실제 고장 발생부터 CAN 전송까지의 상세 타임라인입니다.

### Phase 1: 고장 감지 (t=0ms ~ t=1ms)

```
t=0ms   PMIC 내부에서 과전압 감지
        ├─ Buck 출력 전압 > 설정값
        └─ 하드웨어 보호 회로 동작
            ↓
        PMIC IRQ 핀 출력 변화
        ├─ PB3 핀: HIGH → LOW (하강 에지)
        └─ 하드웨어 인터럽트 트리거
            ↓
        EXTI3_IRQHandler() 호출 (ISR)
        └─ HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3)
            └─ HAL_GPIO_EXTI_Callback(GPIO_PIN_3)
                ├─ printf("[IRQ] PMIC fault detected\n")
                └─ task_set_pmic_irq_event()
                    └─ osEventFlagsSet(SystemEventsHandle, EVENT_PMIC_IRQ)
                        ↓
                    RTOS 스케줄러 개입
                    └─ Task_PMIC_Monitor 깨우기 대기열 추가
```

### Phase 2: 진단 시작 (t=1ms ~ t=2ms)

```
t=1ms   RTOS 스케줄러가 컨텍스트 스위칭
        └─ Task_PMIC_Monitor 실행 (osPriorityRealtime)
            ↓
        osEventFlagsWait() 반환
        └─ flags = EVENT_PMIC_IRQ
            ↓
        pmic_service_start_diagnosis()
        ├─ pmic_service_reset_state()
        │   ├─ current_reg_index = 0
        │   ├─ pmic_data_ready = false
        │   └─ memset(&current_pmic_status, 0)
        │
        └─ pmic_start_register_read(0x05)
            ├─ pmic_reg_address = 0x05
            ├─ pmic_service_state = PMIC_SERVICE_READING
            └─ HAL_I2C_Mem_Read_DMA(hi2c1, 0x60<<1, 0x05, ...)
                ├─ DMA1_Stream0 설정
                ├─ I2C1 START 조건 전송
                └─ DMA 전송 시작 (비동기)
```

### Phase 3: I2C DMA 전송 (t=2ms ~ t=10ms)

```
t=2ms   I2C DMA 전송 완료 (레지스터 0x05)
        ├─ DMA1_Stream0_IRQHandler() 호출
        └─ HAL_DMA_IRQHandler(&hdma_i2c1_rx)
            └─ HAL_I2C_MemRxCpltCallback(&hi2c1)
                ↓
            pmic_service_register_received(i2c_rx_buffer[0])
            ├─ printf("[PMIC] Register received: 0x05 = 0x%02X\n", data)
            │
            ├─ pmic_process_register_data(0x05, data)
            │   └─ current_pmic_status.regs.system_status.raw = data
            │       └─ Union 비트필드에 저장
            │
            ├─ current_reg_index++ (0 → 1)
            │
            └─ pmic_start_register_read(0x06)
                └─ HAL_I2C_Mem_Read_DMA(hi2c1, 0x60<<1, 0x06, ...)

────────────────────────────────────────────────────────────

t=3ms   레지스터 0x06 읽기 완료
        └─ current_pmic_status.regs.power_good.raw = data

t=4ms   레지스터 0x07 읽기 완료
        └─ current_pmic_status.regs.uv_ov_fault.raw = data
            └─ 여기서 과전압 비트 발견!

t=5ms   레지스터 0x08 읽기 완료
        └─ current_pmic_status.regs.oc_fault.raw = data

t=6ms   레지스터 0x09 읽기 완료
        └─ current_pmic_status.regs.temp_fault.raw = data
```

### Phase 4: 고장 분석 및 DTC 생성 (t=10ms ~ t=12ms)

```
t=10ms  모든 레지스터 읽기 완료
        ├─ current_reg_index = 5 (>= PMIC_REG_COUNT)
        ├─ pmic_service_state = PMIC_SERVICE_COMPLETE
        ├─ pmic_data_ready = true
        │
        └─ pmic_service_analyze_faults(&current_pmic_status)
            ↓
        1단계: 모든 고장 상태 분석
        ├─ Power Good 체크
        ├─ UV/OV 고장 분석 ← 과전압 감지!
        ├─ OC 고장 분석
        └─ 온도 고장 분석
            ↓
        2단계: DTC 생성 (우선순위 순서)
        └─ dtc_add_code(DTC_BRAKE_PMIC_OV)
            ├─ detected_dtc_table에 추가
            ├─ DTC_Code = 0xC002
            ├─ active = 1
            ├─ occurrence_count = 1
            └─ eeprom_service_save_dtc(0xC002, "Buck Overvoltage Fault")
                └─ DMA 큐에 추가
```

### Phase 5: EEPROM 저장 (t=12ms ~ t=20ms)

```
t=12ms  SPI DMA WREN 명령
        └─ HAL_SPI_Transmit_DMA(0x06)

t=13ms  WREN 완료, WRITE 명령 시작
        └─ eeprom_write_dtc_log_internal()
            ├─ 명령: 0x02 (WRITE)
            ├─ 주소: 0x0000
            └─ 데이터: 64바이트 DTC 로그

t=14ms  WRITE 완료, Write Cycle 대기
        └─ HAL_Delay(5ms)

t=18ms  상태 확인
        └─ RDSR 명령으로 WIP 비트 체크
            └─ WIP = 0 (쓰기 완료)
                └─ eeprom_service_state = IDLE
```

### Phase 6: CAN 전송 (t=100ms)

```
t=100ms Task_5ms의 20번째 실행
        └─ can_service_send_brake_status()
            ├─ brake_status.active_dtc_count = 1
            ├─ CAN ID: 0x200
            ├─ DLC: 8
            └─ Data: [01][01][03][52][41][01][00][00]
                      │  │   │   │   └─ temperature
                      │  │   │   └───── pressure
                      │  │   └───────── (850 kPa)
                      │  └───────────── DTC count = 1
                      └─────────────── status = Normal
```

---

## 4. UDS 진단 요청 처리 흐름

외부 진단기에서 DTC 정보를 요청하는 시나리오입니다.

```
진단기 → CAN 요청 전송
═══════════════════════════════════════
CAN ID: 0x7E0 (UDS_REQUEST)
Data: [03][19][02][00][00][00][00][00]
      └┬┘└─┬─┘└┬┘
       │   │   └─ Subfunction: 0x02
       │   └───── Service: 0x19 (Read DTC)
       └───────── PCI: SF, Length=3
═══════════════════════════════════════
    ↓
CAN1_RX0_IRQHandler()
└─ HAL_CAN_RxFifo0MsgPendingCallback()
    └─ can_service_handle_uds_request()
        ↓
    uds_parse_can_message()
    ├─ service_id = 0x19
    └─ subfunction = 0x02
        ↓
    uds_process_request()
    └─ uds_service_read_dtc(0x02, 0xFF)
        ↓
    EEPROM에서 DTC 읽기 (블로킹):
    └─ eeprom_service_read_all_dtc(logs, 6)
        ├─ 주소: 0x0000
        ├─ HAL_SPI_Transmit(READ 명령)
        ├─ HAL_SPI_Receive(64바이트)
        └─ DTC 로그 파싱
            ↓
    UDS 응답 생성:
    ├─ service_id = 0x59 (0x19 + 0x40)
    ├─ data[0] = 0x08 (Status Mask)
    ├─ data[1-3] = 0x0C0002 (DTC Code)
    └─ data[4] = 0x01 (Status)
        ↓
    CAN 응답 전송:
═══════════════════════════════════════
CAN ID: 0x7E8 (UDS_RESPONSE)
Data: [06][59][08][0C][00][02][01][00]
      └┬┘└┬┘└┬┘└──┬───┬───┬─┘
       │  │  │    │   │   └─ Status
       │  │  │    └───┴───── DTC: 0x0C0002
       │  │  └────────────── Status Mask
       │  └───────────────── Service + 0x40
       └──────────────────── PCI: SF, Len=6
═══════════════════════════════════════
```

---

## 5. 주요 타이밍 요약

| 작업 | 소요 시간 | 주기/빈도 | 비고 |
|------|-----------|-----------|------|
| PMIC IRQ → 진단 시작 | < 1ms | 이벤트 발생 시 | RTOS 스위칭 포함 |
| 레지스터 1개 읽기 | 1~2ms | - | 100kHz I2C |
| 전체 5개 레지스터 읽기 | 5~10ms | - | 순차 읽기 |
| DTC 분석 및 생성 | < 1ms | - | CPU 처리만 |
| EEPROM 64바이트 쓰기 | ~5ms | DTC 발생 시 | Write Cycle |
| ADC 샘플링 | < 0.5ms | 1ms 주기 | Task_1ms |
| CAN 주기 전송 | < 1ms | 100ms 주기 | Task_5ms |
| UDS 요청 처리 | 10~50ms | 진단기 요청 시 | EEPROM 읽기 포함 |

---

## 6. Mutex 보호 영역

### DTC_MutexHandle

```
보호 대상: detected_dtc_table

사용 위치:
├─ Task_5ms: pmic_service_analyze_faults()
├─ Task_PMIC_Monitor: 긴급 상황 처리
└─ UDS: dtc_clear_all()

경쟁 조건 방지:
- 동시 DTC 추가 시도
- Clear 명령과 새 DTC 발생 충돌
```

### EEPROM_MutexHandle

```
보호 대상: EEPROM 하드웨어 & 서비스 상태

사용 위치:
├─ dtc_add_code(): eeprom_service_save_dtc()
├─ dtc_clear_all(): eeprom_service_clear_all_dtc()
└─ UDS: eeprom_service_read_all_dtc()

경쟁 조건 방지:
- DTC 저장 중 Read 요청
- SPI 하드웨어 동시 접근
```

### CAN_MutexHandle

```
보호 대상: CAN 하드웨어 & 전송 버퍼

사용 위치:
└─ Task_5ms: can_service_send_brake_status()

경쟁 조건 방지:
- 주기 전송과 UDS 응답 충돌
- TX 메일박스 동시 접근
```

---

## 7. 시스템 상태 머신

### PMIC 서비스

```
[IDLE] ──start_diagnosis()──→ [READING] ──완료──→ [COMPLETE]
  ↑                              │ (I2C DMA)         │
  └──────────────────────────────┴───────────────────┘
                            analyze_faults()
```

### EEPROM 서비스

```
[IDLE] ──save_dtc()──→ [WRITE_ENABLE] ──→ [WRITING] ──→ [READ_STATUS]
  ↑         (WREN)         (WRITE+64B)       (RDSR)           │
  └──────────────────────────────────────────────────────────┘
                          쓰기 완료 (WIP=0)
```

### UDS 프로토콜

```
[IDLE] ──CAN 요청──→ [PROCESSING] ──→ [SENDING_RESPONSE]
  ↑      (0x7E0)      (서비스 처리)     (0x7E8 전송)      │
  └────────────────────────────────────────────────────────┘
```

---

## 8. 데이터 흐름 다이어그램

```
┌─────────────────────────────────────────────────┐
│            PMIC (MP5475)                         │
│  - Buck A/B/C/D 전압 모니터링                   │
│  - 과전압/저전압/과전류 감지                     │
│  - IRQ 핀으로 고장 신호                         │
└────────────────┬────────────────────────────────┘
                 │ IRQ (PB3)
                 ↓
┌─────────────────────────────────────────────────┐
│         EXTI3 인터럽트 핸들러                   │
│  - 하강 에지 → Event Flags 설정                │
└────────────────┬────────────────────────────────┘
                 │ osEventFlagsSet()
                 ↓
┌─────────────────────────────────────────────────┐
│      Task_PMIC_Monitor (Realtime)               │
│  ① I2C DMA로 레지스터 5개 읽기                  │
│  ② Union 구조체에 저장                          │
│  ③ 비트필드로 고장 분석                         │
└────────────────┬────────────────────────────────┘
                 │ pmic_status_data_t
                 ↓
┌─────────────────────────────────────────────────┐
│            DTC Manager                           │
│  ① 고장별 DTC 코드 생성                         │
│  ② detected_dtc_table에 저장                    │
│  ③ EEPROM 저장 큐에 추가                        │
└────────┬────────────────────┬───────────────────┘
         │                    │
         │                    ↓
         │          ┌─────────────────────┐
         │          │  EEPROM Service     │
         │          │ - SPI DMA 비동기    │
         │          │ - 64바이트 로그     │
         │          └─────────────────────┘
         ↓
┌─────────────────────────────────────────────────┐
│            CAN Service                           │
│  ① DTC 이벤트 (0x203)                           │
│  ② 브레이크 상태 (0x200, 100ms)                 │
│  ③ UDS 응답 (0x7E8)                             │
└────────────────┬────────────────────────────────┘
                 │ CAN 버스
                 ↓
┌─────────────────────────────────────────────────┐
│         외부 시스템                              │
│  - 다른 ECU (DTC 수신)                          │
│  - 진단기 (UDS)                                  │
│  - 게이트웨이                                    │
└─────────────────────────────────────────────────┘
```

---

## 9. 메모리 맵

### EEPROM (25LC256, 32KB)

```
Address   Size   영역           설명
───────────────────────────────────────────────
0x0000    4KB    DTC_CURRENT    현재 DTC (64개)
0x1000    12KB   DTC_HISTORY    히스토리 (192개)
0x4000    4KB    CONFIG         설정 데이터
0x5000    4KB    CALIBRATION    캘리브레이션
0x6000    4KB    USER           사용자 영역
0x7000    4KB    RESERVED       예약
───────────────────────────────────────────────
```

### RAM (DTC 관련)

```
dtc_master_table[6]       384바이트 (const)
detected_dtc_table[6]     384바이트 (RAM)
current_pmic_status       8바이트 (Union)
dtc_save_queue[6]         384바이트 (Queue)
────────────────────────────────────────
총 RAM 사용량: ~1.5KB
```

## 부록: 코드 위치 참조

| 기능 | 파일 | 주요 함수 |
|------|------|-----------|
| 시스템 초기화 | main.c | main() |
| Task 생성 | task_manager.c | task_manager_start() |
| PMIC 진단 | pmic_service.c | pmic_service_start_diagnosis() |
| DTC 관리 | dtc_manager.c | dtc_add_code() |
| EEPROM 저장 | eeprom_service.c | eeprom_service_save_dtc() |
| CAN 전송 | can_service.c | can_service_broadcast_dtc_event() |
| UDS 처리 | uds_protocol.c | uds_process_request() |
| 인터럽트 | stm32f4xx_it.c | EXTI3_IRQHandler() |

