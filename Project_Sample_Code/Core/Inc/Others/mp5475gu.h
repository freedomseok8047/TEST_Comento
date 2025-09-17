/**
 * @file mp5475gu.h
 * @brief MP5475GU PMIC I2C 제어 헤더 파일
 * @author JUNSEOK LEE
 * @date 2025-09-05
 *
 * @note 브레이크 시스템용 PMIC 제어
 *       I2C DMA Interrupt 방식으로 동작
 */

#ifndef MP5475GU_H

#define MP5475GU_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief MP5475GU I2C Slave Address
 * @note 7-bit address, 데이터시트 기준
 */
/*
 * 요약: I2C 하드웨어가 마지막 비트로 읽기/쓰기를 판단하기 때문
 * 최하위 비트(LSB): Read/Write 구분 0 = Write / 1 = Read
 */
#define MP5475GU_I2C_ADDR			0x68 // 7bit address
#define MP5475GU_I2C_ADDR_WRITE 	(MP5475GU_I2C_ADDR << 1)
#define MP5475GU_I2C_ADDR_READ		((MP5475GU_I2C_ADDR << 1) | 1)

/**
 * @brief MP5475GU 레지스터 주소 정의
 * @note 브레이크 시스템에서 사용하는 주요 레지스터
 */
typedef enum {
	// 시스템 상태 레지스터 (읽기 전용)
	MP5475_REG_SYSTEM_STATUS	= 0x05,
	MP5475_REG_POWER_GOOD		= 0x06,
	MP5475_REG_UV_OV_FAULT		= 0x07,
	MP5475_REG_TEMP_FAULT		= 0x09,

	// Buck A 제어 레지스터 (0x10 ~ 0x17)
	MP5475_REG_BUCKA_CTRL1		= 0x10,
	MP5475_REG_BUCKA_CTRL2		= 0x11,
	MP5475_REG_BUCKA_CTRL3		= 0x12,
	MP5475_REG_BUCKA_VOUT_SEL	= 0x13,
	MP5475_REG_BUCKA_VREF_LOW	= 0x14,
	MP5475_REG_BUCKA_RESERVED1	= 0x15,
	MP5475_REG_BUCKA_RESERVED2	= 0x16,
	MP5475_REG_BUCKA_AVP		= 0x17,

	// Buck B 제어 레지스터 (0x18 ~ 0x1E)
	MP5475_REG_BUCKB_CTRL1		= 0x18,
	MP5475_REG_BUCKB_CTRL2		= 0x19,
	MP5475_REG_BUCKB_CTRL3		= 0x1A,
	MP5475_REG_BUCKB_VOUT_SEL	= 0x1B,
	MP5475_REG_BUCKB_VREF_LOW	= 0x1C,
	MP5475_REG_BUCKB_RESERVED1	= 0x1D,
	MP5475_REG_BUCKB_RESERVED2	= 0x1E,

	// Buck C 제어 레지스터 (0x20~0x27)
	MP5475_REG_BUCKC_CTRL1      = 0x20,    // Dual Phase, Soft Start 설정
	MP5475_REG_BUCKC_CTRL2      = 0x21,    // Shutdown Delay, Transition Rate
	MP5475_REG_BUCKC_CTRL3      = 0x22,    // Mode, Current Limit, OVP 설정
	MP5475_REG_BUCKC_VOUT_SEL   = 0x23,    // 출력 전압 선택
	MP5475_REG_BUCKC_VREF_LOW   = 0x24,    // 기준 전압 하위 바이트
	MP5475_REG_BUCKC_RESERVED1  = 0x25,    // 예약됨
	MP5475_REG_BUCKC_RESERVED2  = 0x26,    // 예약됨
	MP5475_REG_BUCKC_AVP        = 0x27,    // AVP 설정

	// Buck D 제어 레지스터 (0x28~0x2E)
	MP5475_REG_BUCKD_CTRL1      = 0x28,    // Soft Start 설정
	MP5475_REG_BUCKD_CTRL2      = 0x29,    // Shutdown Delay, Transition Rate
	MP5475_REG_BUCKD_CTRL3      = 0x2A,    // Mode, Current Limit, OVP 설정
	MP5475_REG_BUCKD_VOUT_SEL   = 0x2B,    // 출력 전압 선택
	MP5475_REG_BUCKD_VREF_LOW   = 0x2C,    // 기준 전압 하위 바이트
	MP5475_REG_BUCKD_RESERVED1  = 0x2D,    // 예약됨
	MP5475_REG_BUCKD_RESERVED2  = 0x2E,    // 예약됨

	// 시스템 제어 레지스터 (0x40~0x56)
	MP5475_REG_SYSTEM_ENABLE    = 0x40,    // 시스템/Buck Enable 제어
	MP5475_REG_SLAVE_ADDRESS    = 0x41,    // I2C Slave Address 설정
	MP5475_REG_FREQUENCY        = 0x42,    // 스위칭 주파수 설정
	MP5475_REG_SYSTEM_CONFIG    = 0x43,    // UVLO, Shutdown 설정
	MP5475_REG_POWER_GOOD_CFG   = 0x44,    // Power Good 설정
	MP5475_REG_FAULT_MASK       = 0x45,    // Fault Mask 설정

	// MTP 관련 레지스터
	MP5475_REG_MTP_CONFIG       = 0x50,    // MTP 설정 코드
	MP5475_REG_MTP_REVISION     = 0x51,    // MTP 리비전 번호
	MP5475_REG_MTP_PASSWORD     = 0x52,    // MTP 프로그램 패스워드
	MP5475_REG_REVISION_ID      = 0x54,    // 리비전 ID
	MP5475_REG_VENDOR_ID_BYTE0  = 0x55,    // 벤더 ID 바이트 0
	MP5475_REG_VENDOR_ID_BYTE1  = 0x56     // 벤더 ID 바이트 1
	} mp5475_register_t;

	/**
	 * @brief 브레이크 시스템 DTC 코드 정의
	 */
	typedef enum {
		DTC_BRAKE_PMIC_UV		= 0xC001,
		DTC_BRAKE_PMIC_OV		= 0xC002,
		DTC_BRAKE_PMIC_OC		= 0xC003,
		DTC_BRAKE_PMIC_TEMP		= 0xC004,
		DTC_BRAKE_COMM_ERROR	= 0xC005,
		DTC_BRAKE_SYSTEM_FAULT	= 0xC006
	} brake_dtc_code_t;

	/**
	 * @brief 시스템 상태 레지스터 (0x05) 비트필드
	 */
	typedef struct {
	    uint8_t pmr_good_raw    : 1;    // PMR Good Raw
	    uint8_t pmr_good        : 1;    // PMR Good Filtered
	    uint8_t fsm_state       : 2;    // FSM State
	    uint8_t reserved        : 4;    // 예약됨
	} mp5475_system_status_bits_t;

	/**
	 * @brief Power Good 레지스터 (0x06) 비트필드
	 */
	typedef struct {
	    uint8_t buckd_pg_raw    : 1;    // Buck D Power Good Raw
	    uint8_t buckc_pg_raw    : 1;    // Buck C Power Good Raw
	    uint8_t buckb_pg_raw    : 1;    // Buck B Power Good Raw
	    uint8_t bucka_pg_raw    : 1;    // Buck A Power Good Raw
	    uint8_t buckd_pg_filt   : 1;    // Buck D Power Good Filtered
	    uint8_t buckc_pg_filt   : 1;    // Buck C Power Good Filtered
	    uint8_t buckb_pg_filt   : 1;    // Buck B Power Good Filtered
	    uint8_t bucka_pg_filt   : 1;    // Buck A Power Good Filtered
	} mp5475_power_good_bits_t;

	/**
	 * @brief UV/OV 고장 레지스터 (0x07) 비트필드 (RC)
	 */
	typedef struct {
	    uint8_t buckd_ov        : 1;    // Buck D Over Voltage
	    uint8_t buckc_ov        : 1;    // Buck C Over Voltage
	    uint8_t buckb_ov        : 1;    // Buck B Over Voltage
	    uint8_t bucka_ov        : 1;    // Buck A Over Voltage
	    uint8_t buckd_uv        : 1;    // Buck D Under Voltage
	    uint8_t buckc_uv        : 1;    // Buck C Under Voltage
	    uint8_t buckb_uv        : 1;    // Buck B Under Voltage
	    uint8_t bucka_uv        : 1;    // Buck A Under Voltage
	} mp5475_uv_ov_fault_bits_t;

	/**
	 * @brief OC 고장 레지스터 (0x08) 비트필드 (RC)
	 */
	typedef struct {
	    uint8_t buckd_oc_warn   : 1;    // Buck D OC Warning
	    uint8_t buckc_oc_warn   : 1;    // Buck C OC Warning
	    uint8_t buckb_oc_warn   : 1;    // Buck B OC Warning
	    uint8_t bucka_oc_warn   : 1;    // Buck A OC Warning
	    uint8_t buckd_oc        : 1;    // Buck D Over Current
	    uint8_t buckc_oc        : 1;    // Buck C Over Current
	    uint8_t buckb_oc        : 1;    // Buck B Over Current
	    uint8_t bucka_oc        : 1;    // Buck A Over Current
	} mp5475_oc_fault_bits_t;

	/**
	 * @brief 온도/기타 고장 레지스터 (0x09) 비트필드 (WC)
	 */
	typedef struct {
	    uint8_t pmic_temp_shutdown  : 1;    // PMIC High Temperature Shutdown
	    uint8_t pmic_temp_warning   : 1;    // PMIC High Temperature Warning
	    uint8_t vdrv_ov             : 1;    // VDRV Over Voltage
	    uint8_t vbulk_ov            : 1;    // VBULK Over Voltage
	    uint8_t vr_fault            : 1;    // VR Fault
	    uint8_t ldo_1v1_fault       : 1;    // 1.1V LDO Fault
	    uint8_t ldo_1v8_fault       : 1;    // 1.8V LDO Fault
	    uint8_t reserved            : 1;    // 예약됨
	} mp5475_temp_fault_bits_t;

	/**
	 * @brief 시스템 Enable 레지스터 (0x40) 비트필드
	 */
	typedef struct {
	    uint8_t sfrst           : 1;    // Software Reset
	    uint8_t reserved1       : 1;    // 예약됨
	    uint8_t end             : 1;    // Buck D Enable
	    uint8_t enc             : 1;    // Buck C Enable
	    uint8_t enb             : 1;    // Buck B Enable
	    uint8_t ena             : 1;    // Buck A Enable
	    uint8_t reserved2       : 1;    // 예약됨
	    uint8_t sysen           : 1;    // System Enable
	} mp5475_system_enable_bits_t;

	/**
	 * @brief MP5475GU 8바이트 상태 데이터 구조체 (Union 사용)
	 * @note 요구사항: 8바이트 기능의 데이터를 사용하기 쉽게 Union, 비트필드 이용
	 */
	typedef union {
	    // 8바이트 원시 데이터
	    uint8_t raw_data[8];

	    // 구조화된 데이터 (주요 상태 레지스터)
	    struct {
	        mp5475_system_status_bits_t     system_status;      // 0x05
	        mp5475_power_good_bits_t        power_good;         // 0x06
	        mp5475_uv_ov_fault_bits_t       uv_ov_fault;        // 0x07
	        mp5475_oc_fault_bits_t          oc_fault;           // 0x08
	        mp5475_temp_fault_bits_t        temp_fault;         // 0x09
	        uint8_t                         reserved[3];        // 예약된 바이트
	    } status_regs;

	    // 시스템 제어 데이터
	    struct {
	        mp5475_system_enable_bits_t     system_enable;      // 0x40
	        uint8_t                         slave_address;      // 0x41
	        uint8_t                         frequency;          // 0x42
	        uint8_t                         system_config;      // 0x43
	        uint8_t                         power_good_cfg;     // 0x44
	        uint8_t                         fault_mask;         // 0x45
	        uint8_t                         reserved[2];        // 예약된 바이트
	    } control_regs;

	    // 16비트 워드 단위 접근
	    uint16_t word_data[4];

	    // 32비트 더블워드 단위 접근
	    uint32_t dword_data[2];

	    // 64비트 전체 데이터
	    uint64_t full_data;

	} mp5475_data_t;

	/**
	 * @brief 스위칭 주파수 설정 (레지스터 0x42)
	 */
	typedef enum {
	    MP5475_FREQ_500KHZ      = 0x0,      // 500kHz
	    MP5475_FREQ_750KHZ      = 0x1,      // 750kHz
	    MP5475_FREQ_1000KHZ     = 0x2,      // 1MHz
	    MP5475_FREQ_1250KHZ     = 0x3,      // 1.25MHz
	    MP5475_FREQ_1500KHZ     = 0x4,      // 1.5MHz
	    MP5475_FREQ_2000KHZ     = 0x5       // 2MHz
	} mp5475_frequency_t;

	/**
	 * @brief MP5475GU I2C 초기화
	 * @return true: 성공, false: 실패
	 */
	bool mp5475_init(void);

	/**
	 * @brief MP5475GU 레지스터 읽기 (DMA Interrupt 방식)
	 * @param reg_addr 레지스터 주소
	 * @param data 읽은 데이터 저장 포인터
	 * @return true: 성공, false: 실패
	 */
	bool mp5475_read_register(mp5475_register_t reg_addr, unit8_t *data);

	/**
	 * @brief MP5475GU 상태 레지스터 일괄 읽기 (0x05~0x09)
	 * @param data 8바이트 데이터 구조체 포인터
	 * @return true: 성공, false: 실패
	 */
	bool mp5475_read_status_registers(mp5475_data_t *data);

	/**
	 * @brief MP5475GU 레지스터 쓰기 (DMA Interrupt 방식)
	 * @param reg_addr 레지스터 주소
	 * @param data 쓸 데이터
	 * @return true: 성공, false: 실패
	 */
	bool mp5475_write_register(mp5475_register_t reg_addr, unit8_t data);

	/**
	 * @brief 시스템 Enable/Disable 제어
	 * @param enable true: Enable, false: Disable
	 * @return true: 성공, false: 실패
	 */
	bool mp5475_system_enable(bool enable);

	/**
	 * @brief Buck 채널별 Enable/Disable 제어
	 * @param channel Buck 채널 (A=0, B=1, C=2, D=3)
	 * @param enable true: Enable, false: Disable
	 * @return true: 성공, false: 실패
	 */
	bool mp5475_buck_enble(unit8_t channel, bool enable);

	/**
	 * @brief 스위칭 주파수 설정
	 * @param frequency 설정할 주파수
	 * @return true: 성공, false: 실패
	 */
	bool mp5475_set_frequency(mp5475_frequency_t frequency);

	/**
	 * @brief 고장 상태 확인 및 DTC 생성
	 * @param pmic_data PMIC 데이터 구조체 포인터
	 * @return brake_dtc_code_t 감지된 DTC 코드 (정상시 0)
	 */
	brake_dtc_code_t mp5475_check_faults(mp5474_data_t *pmic_data);

	/**
	 * @brief Power Good 상태 확인
	 * @param pmic_data PMIC 데이터 구조체 포인터
	 * @return true: 정상, false: 문제 있음
	 */
	bool mp5474_is_power_good(mp5475_data_t *pmic_data);

	/**
	 * @brief 소프트웨어 리셋 실행
	 * @return true: 성공, false: 실패
	 */
	bool mp5475_software_reset(void);

	/**
	 * @brief Vendor ID 확인 (디바이스 검증용)
	 * @param vendor_id Vendor ID 저장 포인터 (2바이트)
	 * @return true: 성공, false: 실패
	 */
	bool mp5474_read_vendor_id(uint16_t *vendor_id);

	/**
	 * @brief I2C DMA 완료 콜백 함수 (인터럽트에서 호출)
	 */
	void mp5474_i2c_dma_complete_callback(void);

	/**
	 * @brief I2C 에러 콜백 함수 (인터럽트에서 호출)
	 */
	void mp5475_i2c_error_callback(void);

	#endif // MP5475GU_H





























