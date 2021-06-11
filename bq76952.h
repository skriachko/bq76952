/*
* Description :   Header file of BQ76952 battery monitor
* Copyright (c) 2011 BroLab.  All right reserved.
* Author      :   Sergii Kriachko
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*/
#include <i2c_hal.h>
#include <inttypes.h>
#include <stddef.h>
#include "main.h"
typedef uint8_t byte;

enum bq76952_thermistor {
	TS1,
	TS2,
	TS3,
	HDQ,
	DCHG,
	DDSG
};

enum bq76952_fet {
	CHG,
	DCH,
	ALL
};

enum bq76952_fet_state {
	OFF,
	ON
};

enum bq76952_scd_thresh {
	SCD_10,
	SCD_20,
	SCD_40,
	SCD_60,
	SCD_80,
	SCD_100,
	SCD_125,
	SCD_150,
	SCD_175,
	SCD_200,
	SCD_250,
	SCD_300,
	SCD_350,
	SCD_400,
	SCD_450,
	SCD_500
};

// ======= bq76952 registers =======
// regulator output is used and REGIN is not supplied externally.
#define	REG0_CONFIG 0x9237
// Changes voltage regulator settings
#define	REG12_CONTROL 0x9236
// Alert pin config
#define	ALERT_PIN_CONFIG 0x92FC
#define	DEFAULT_ALARM_MASK_CONFIG 0x926D

#define	DA_CONFIGURATION 0x9303

// Configures the pack voltage threshold at which
// the device will enter SHUTDOWN mode.
#define	SHUTDOWN_STACK_VOLTAGE 0x9241

// VCell mode config - cells connected
#define	VCELL_MODE 0x9304
// Protection configuration, default 0x0002
#define PROTECTION_CONFIGURATION 0x925F
// Enable Protections A, default 0x88
#define	ENABLE_PROTECTIONS_A 0x9261
// Enable Protections B, default 0x00
#define	ENABLE_PROTECTIONS_B 0x9262
// default 0x98
#define	CHG_FET_PROTECTION_A 0x9265
#define	CHG_FET_PROTECTION_B 0x9266
#define	CHG_FET_PROTECTION_C 0x9267
// Enable Protections C, default 0x00
#define	ENABLE_PROTECTIONS_C 0x9263
// CHG FET Protections A, default 0x98: all CHG FET protections enabled (SCD, OCC, COV).
#define CHG_FET_PROTECTIONS_A 0x9265
// Base address for Interconnect Resistances cell 1..16 (16-bit each)
#define CELL_INTERCONNECT_RESISTANCE 0x9315

#define DSG_FET_PROTECTION_A 0x9269
#define DSG_FET_PROTECTION_B 0x926A
#define DSG_FET_PROTECTION_C 0x926B
#define SF_ALERT_MASK_A		0x926F
#define SF_ALERT_MASK_B		0x9270
#define SF_ALERT_MASK_C		0x9271
// Short Circuit Discharge threshold (by SRP-SRN)
#define SCD_THRESHOLD_CONFIG 0x9286
#define SCD_DELAY_CONFIG 0x9287
#define FET_OPTIONS 0x9308
#define FET_PREDISCHARGE_TIMEOUT 0x930E
#define FET_PREDISCHARGE_STOP_DELTA 0x930F


#define CC3_SAMPLES 0x9307
#define	TS1_CONFIG 0x92FD
#define	TS2_CONFIG 0x92FE
#define	TS3_CONFIG 0x92FF

// ------------- Security keys for UNSEAL and full access mode ----------
#define UNSEAL_KEY_STEP_1 0x0414
#define UNSEAL_KEY_STEP_2 0x3672
#define FULL_ACCESS_KEY_STEP_1 0x1234
#define FULL_ACCESS_KEY_STEP_2 0xABCD

// ============= Constant values ================
#define CELL_INTERCONNECT_RESISTANCE_MOHM 0

typedef union protection {
	struct {
		uint8_t SC_DCHG            :1;
		uint8_t OC2_DCHG           :1;
		uint8_t OC1_DCHG           :1;
		uint8_t OC_CHG             :1;
		uint8_t CELL_OV            :1;
		uint8_t CELL_UV            :1;
	} bits;
} bq76952_protection_t;

typedef union safety_status_c {
	struct {
		uint8_t OCD3           :1;
		uint8_t SCDL           :1;
		uint8_t OCDL           :1;
		uint8_t COVL           :1;
		uint8_t PTOS           :1;
	} bits;
} bq76952_safety_alert_c_t;


typedef union temperatureProtection {
	struct {
		uint8_t OVERTEMP_FET		:1;
		uint8_t OVERTEMP_INTERNAL	:1;
		uint8_t OVERTEMP_DCHG		:1;
		uint8_t OVERTEMP_CHG		:1;
		uint8_t UNDERTEMP_INTERNAL	:1;
		uint8_t UNDERTEMP_DCHG		:1;
		uint8_t UNDERTEMP_CHG		:1;
	} bits;
} bq76952_temp_t;

typedef union batteryStatus {
	struct {
		uint8_t SLEEP_MODE			:1;
		uint8_t BIT14_RESERVED		:1;
		uint8_t SHUTDOWN_PENDING	:1;
		uint8_t PERMANENT_FAULT		:1;
		uint8_t SAFETY_FAULT		:1;
		uint8_t FUSE_PIN			:1;
		uint8_t SECURITY_STATE		:2;
		uint8_t WR_TO_OTP_BLOCKED	:1;
		uint8_t WR_TO_OTP_PENDING	:1;
		uint8_t OPEN_WIRE_CHECK		:1;
		uint8_t WD_WAS_TRIGGERED	:1;
		uint8_t FULL_RESET_OCCURED	:1;
		uint8_t SLEEP_EN_ALLOWED	:1;
		uint8_t PRECHARGE_MODE		:1;
		uint8_t CONFIG_UPDATE_MODE	:1;
	} bits;
} bq76952_battery_status_t;

class bq76952 {
public:
	static bq76952& getInstance(void);
	static bq76952& create(I2C_HandleTypeDef *hi2c);
	void begin(void);
	void reset(void);
	void enterConfigUpdate(void);
	void exitConfigUpdate(void);
	bool isConnected(void);
	byte getMfgStatusInitRegister(void);
	int getCellVoltage(byte cellNumber);
	void getAllCellVoltages(int* cellArray);
	void getOnlyConnectedCellVoltages(int *cellArray);
	int getCurrent(void);
	int getCurrentNow(void);
	int getCurrentAvg(void);
	bool areFETs_Enabled(void);
	unsigned int getManufacturingStatus(void);
	unsigned int getStackVoltage(void);
	float getInternalTemp(void);
	float getThermistorTemp(bq76952_thermistor);
	bq76952_protection_t getProtectionStatus(void);
	bq76952_safety_alert_c_t getSafetyAlert_C(void);
	bq76952_temp_t getTemperatureStatus(void);
	void setFET(bq76952_fet, bq76952_fet_state);
	void setFET_ENABLE(void);
	bool isDischarging(void);
	bool isCharging(void);
	void setCellOvervoltageProtection(unsigned int, unsigned int);
	void setCellUndervoltageProtection(unsigned int, unsigned int);
	void setShortCircuitThreshold(void);
	void setProtectionConfiguration(void);
	void setShutdownStackVoltage(unsigned int voltage);
	void setChargingOvercurrentProtection(unsigned int, byte);
	void setChargingTemperatureMaxLimit(signed int, byte);
	void setDischargingOvercurrentProtection(unsigned int, byte);
	void setDischargingOvercurrentProtection_OCD3(int16_t mA);
	void setDischargingOvercurrentProtection_Recovery(int16_t mA);
	void setDischargingShortcircuitProtection(bq76952_scd_thresh, unsigned int);
	void setDischargingTemperatureMaxLimit(signed int, byte);
	unsigned int getDeviceNumber(void);
	unsigned int getHWVersion(void);
	bool checkSecurityKeys(void);
	unsigned int getCOVSnapshot(byte cell);
	bool Enter_FullAccessMode(void);
	bool configure_before_OTP_write(void);
	bool is_OTP_already_programmed(void);
	bool program_OTP(void);
	void setEnablePreRegulator(void);
	void setDA_Config(void);
	void setSF_AlertMask_A(void);
	void setSF_AlertMask_B(void);
	void setSF_AlertMask_C(void);
	void setEnableRegulator(bool, bool);
	void setAlertPinConfig(void);
	void setDefaultAlarmMaskConfig(void);
	void setVcellMode(uint16_t vcell_mode);
	void setEnableCHG_FET_Protection(void);
	void setEnableProtectionsA(void);
	void setEnableProtectionsB(void);
	void setEnableProtectionsC(void);
	void setCHGFETProtectionsA(byte);
	void setDSGFETProtectionsA(void);
	void setDSGFETProtectionsB(void);
	void setDSGFETProtectionsC(void);
	void setFET_Options(void);
	void setFET_PredischargeTimeout(void);
	void setFET_PredischargeStopDelta(void);
	void setCellInterconnectResistances(void);
	unsigned int getAlertRawStatusRegister(void);
	void setEnableTS1();
	void setEnableTS2();
	void setEnableTS3();
	unsigned int getAlertStatusRegister(void);
	static byte HandleAlarm(void);
	bq76952_battery_status_t getBatteryStatusRegister(void);
	unsigned int readDataMemory(unsigned int, int);
private:
	static I2C_HandleTypeDef *hi2c;
	bq76952();
	I2C_HAL i2c_hal;
	void initBQ(void);
	unsigned int directCommand(byte);
	void subCommand(unsigned int);
	int16_t subCommandResponseInt(byte);
	byte calculateChecksum(byte, byte);
	void writeDataMemory(unsigned int , int16_t, byte);
	void writeDataMemory_WIthoutConfigUpdate(unsigned int , int16_t, byte);
};
