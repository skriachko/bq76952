/*
 * Copyright (c) 2011 BroLab.  All right reserved.
 * Author      :   Sergii Kriachko
 * Description :   Source file of BQ76952 10-series multicell.
 * Up to 16 cells can be configured.
 * battery monitor and protector for STM32 platform.
 * Author      :   Sergii Kriachko
 * Date        :   11/06/2021
 * License     :   MIT
 * This code is published as open source software. Feel free to share/modify.
 * This code was verified on STM32WB55.
 */

#include <bq76952.h>
#include <i2c_hal.h>
#include "main.h"
#include "cmsis_os.h"

#define BQ_I2C_ADDR_WRITE   0x10
#define BQ_I2C_ADDR_READ    0x11

// BQ76952 commands / subcommands
#define CMD_DIR_SUBCMD_LOW            0x3E
#define CMD_DIR_SUBCMD_HI             0x3F
#define CMD_DIR_RESP_LEN              0x61
#define CMD_DIR_RESP_START            0x40
#define CMD_DIR_RESP_CHKSUM           0x60

// BQ76952 - Voltage measurement commands
#define CMD_READ_VOLTAGE_CELL_1   0x14
#define CMD_READ_VOLTAGE_CELL_2   0x16
#define CMD_READ_VOLTAGE_CELL_3   0x18
#define CMD_READ_VOLTAGE_CELL_4   0x1A
#define CMD_READ_VOLTAGE_CELL_5   0x1C
#define CMD_READ_VOLTAGE_CELL_6   0x1E
#define CMD_READ_VOLTAGE_CELL_7   0x20
#define CMD_READ_VOLTAGE_CELL_8   0x22
#define CMD_READ_VOLTAGE_CELL_9   0x24
#define CMD_READ_VOLTAGE_CELL_10  0x26
#define CMD_READ_VOLTAGE_CELL_11  0x28
#define CMD_READ_VOLTAGE_CELL_12  0x2A
#define CMD_READ_VOLTAGE_CELL_13  0x2C
#define CMD_READ_VOLTAGE_CELL_14  0x2E
#define CMD_READ_VOLTAGE_CELL_15  0x30
#define CMD_READ_VOLTAGE_CELL_16  0x32
#define CMD_READ_VOLTAGE_STACK    0x34
#define CMD_READ_VOLTAGE_PACK     0x36

// BQ76952 - Direct Commands
#define CMD_DIR_SPROTEC           0x02
#define CMD_DIR_SAFETY_STATUS_A   0x03
#define CMD_DIR_SAFETY_ALERT_C    0x06

#define CMD_DIR_STEMP             0x04
#define CMD_DIR_FTEMP             0x05
#define CMD_DIR_SFET              0x06
#define CMD_DIR_FFET              0x07
#define CMD_DIR_BATTERY_STATUS    0x12
#define CMD_DIR_VCELL_1           0x14
#define CMD_DIR_CC2_CUR           0x3A
#define CMD_DIR_ALARM_STATUS      0x62
#define CMD_DIR_ALARM_RAW_STATUS  0x64
#define CMD_DIR_ALARM_ENABLE      0x66
#define CMD_DIR_INT_TEMP          0x68
#define CMD_DIR_FET_STAT          0x7F
#define CMD_DEVICE_NUMBER         0x0001
#define CMD_FW_VERSION            0x0002
#define CMD_HW_VERSION            0x0003
#define CMD_CUV_SNAPSHOT          0x0080
#define CMD_COV_SNAPSHOT          0x0081
#define SUBCMD_OTP_WR_CHECK       0x00A0
#define SUBCMD_OTP_WRITE          0x00A1


// Alert Bits in BQ76952 registers
#define BIT_SA_SC_DCHG            7
#define BIT_SA_OC2_DCHG           6
#define BIT_SA_OC1_DCHG           5
#define BIT_SA_OC_CHG             4
#define BIT_SA_CELL_OV            3
#define BIT_SA_CELL_UV            2

#define BIT_SB_OTF                7
#define BIT_SB_OTINT              6
#define BIT_SB_OTD                5
#define BIT_SB_OTC                4
#define BIT_SB_UTINT              2
#define BIT_SB_UTD                1
#define BIT_SB_UTC                0

// MACRO functions
#define CELL_NO_TO_ADDR(cellNo) (0x14 + ((cellNo)*2))
#define LOW_BYTE(data) (byte)(data & 0x00FF)
#define HIGH_BYTE(data) (byte)((data >> 8) & 0x00FF)

// Enable it only once to program OTP
// It's OK to keep it because we check if it's already programmed on init
#define PROGRAM_OTP
// Program Regulators in OTP (PROGRAM_OTP should be enabled)
#define PROGRAM_REGULATORS

// Static variables
I2C_HandleTypeDef * bq76952::hi2c = 0;
static bq76952_protection_t prot;
static bq76952_safety_alert_c_t safety_alert_c;

//// LOW LEVEL FUNCTIONS ////
void bq76952::initBQ(void) {
	i2c_hal.begin();
}
#define F(x) x
// Send Direct command
unsigned int bq76952::directCommand(byte command) {
	i2c_hal.beginTransmission(BQ_I2C_ADDR_WRITE);
	i2c_hal.write(command);
	i2c_hal.endTransmission();
	i2c_hal.requestFrom(BQ_I2C_ADDR_READ, 2);
	while (!i2c_hal.available())
		;
	byte lsb = i2c_hal.read();
	byte msb = i2c_hal.read();

	return (unsigned int) (msb << 8 | lsb);
}

// Send Sub-command
void bq76952::subCommand(unsigned int data) {
	i2c_hal.beginTransmission(BQ_I2C_ADDR_WRITE);
	i2c_hal.write(CMD_DIR_SUBCMD_LOW);
	i2c_hal.write((byte) data & 0x00FF);
	i2c_hal.write((byte) (data >> 8) & 0x00FF);
	i2c_hal.endTransmission();
}

// Read subcommand response
int16_t bq76952::subCommandResponseInt(byte offset) {
	i2c_hal.beginTransmission(BQ_I2C_ADDR_WRITE);
	i2c_hal.write(CMD_DIR_RESP_START + offset);
	i2c_hal.endTransmission();

	i2c_hal.requestFrom(BQ_I2C_ADDR_READ, 2);
	while (!i2c_hal.available())
		;
	byte lsb = i2c_hal.read();
	byte msb = i2c_hal.read();

	int16_t tmp_val = (int) (msb << 8 | lsb);
	return tmp_val;
}

// Enter config update mode
void bq76952::enterConfigUpdate(void) {
	subCommand(0x0090);
	HAL_Delay(2);
}

// Exit config update mode
void bq76952::exitConfigUpdate(void) {
	subCommand(0x0092);
	HAL_Delay(1);
}

// Write Byte to Data memory of BQ76952
void bq76952::writeDataMemory(unsigned int addr, int16_t data,
		byte noOfBytes) {
	byte chksum = 0;

	chksum = calculateChecksum(chksum, LOW_BYTE(addr));
	chksum = calculateChecksum(chksum, HIGH_BYTE(addr));
	chksum = calculateChecksum(chksum, LOW_BYTE(data));
	chksum = calculateChecksum(chksum, HIGH_BYTE(data));

	enterConfigUpdate();
	i2c_hal.beginTransmission(BQ_I2C_ADDR_WRITE);
	i2c_hal.write(CMD_DIR_SUBCMD_LOW);
	i2c_hal.write(LOW_BYTE(addr));
	i2c_hal.write(HIGH_BYTE(addr));
	i2c_hal.write(LOW_BYTE(data));
	if (noOfBytes == 2)
		i2c_hal.write(HIGH_BYTE(data));
	i2c_hal.endTransmission();

	i2c_hal.beginTransmission(BQ_I2C_ADDR_WRITE);
	i2c_hal.write(CMD_DIR_RESP_CHKSUM);
	i2c_hal.write(chksum);
	if (noOfBytes == 1)
		i2c_hal.write(0x05);
	else // if size is 2
		i2c_hal.write(0x06);
	i2c_hal.endTransmission();
	exitConfigUpdate();
}

// Write 2 bytes to Data memory of BQ76952 without entering config update.
void bq76952::writeDataMemory_WIthoutConfigUpdate(unsigned int addr, int16_t data,
		byte noOfBytes) {
	byte chksum = 0;
	chksum = calculateChecksum(chksum, LOW_BYTE(addr));
	chksum = calculateChecksum(chksum, HIGH_BYTE(addr));
	chksum = calculateChecksum(chksum, LOW_BYTE(data));
	chksum = calculateChecksum(chksum, HIGH_BYTE(data));

	i2c_hal.beginTransmission(BQ_I2C_ADDR_WRITE);
	i2c_hal.write(CMD_DIR_SUBCMD_LOW);
	i2c_hal.write(LOW_BYTE(addr));
	i2c_hal.write(HIGH_BYTE(addr));
	i2c_hal.write(LOW_BYTE(data));
	if (noOfBytes == 2)
		i2c_hal.write(HIGH_BYTE(data));
	i2c_hal.endTransmission();

	i2c_hal.beginTransmission(BQ_I2C_ADDR_WRITE);
	i2c_hal.write(CMD_DIR_RESP_CHKSUM);
	i2c_hal.write(chksum);
	if (noOfBytes == 1)
		i2c_hal.write(0x05);
	else // if size is 2
		i2c_hal.write(0x06);
	i2c_hal.endTransmission();
}



// Reads Byte from Data memory of BQ76952
unsigned int bq76952::readDataMemory(unsigned int addr, int size) {
	i2c_hal.beginTransmission(BQ_I2C_ADDR_WRITE);
	i2c_hal.write(CMD_DIR_SUBCMD_LOW);
	i2c_hal.write(LOW_BYTE(addr));
	i2c_hal.write(HIGH_BYTE(addr));
	i2c_hal.endTransmission();

	HAL_Delay(2);
	i2c_hal.requestFrom(BQ_I2C_ADDR_READ, size);

	while (!i2c_hal.available())
		;
	byte lsb = i2c_hal.read();
	if (size == 1)
		return lsb;
	// If size is 2
	byte msb = i2c_hal.read();
	return (unsigned int) (msb << 8 | lsb);

}

// Calculate checksome = ~(sum of all bytes)
byte bq76952::calculateChecksum(byte oldChecksum, byte data) {
	if (!oldChecksum)
		oldChecksum = data;
	else
		oldChecksum = ~(oldChecksum) + data;
	return ~(oldChecksum);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////// API //////////////////
// Initialize singleton
bq76952& bq76952::create(I2C_HandleTypeDef *i2c)
{
	hi2c = i2c;	// store in static variable to be used in bq76952() constructor.
	return bq76952::getInstance(); // instantiate and return singleton
}

// returns Singleton, statically instantiated.
bq76952& bq76952::getInstance(void)
{
	static bq76952 instance;
    return instance;
}

bq76952::bq76952() : i2c_hal{bq76952::hi2c}
{
}

void bq76952::begin(void) {
	initBQ();
}

byte bq76952::HandleAlarm()
{
	byte alertReg;
	alertReg = bq76952::getInstance().getAlertStatusRegister();
	// Handle alarm here
	return alertReg;
}

bool bq76952::isConnected(void) {
	i2c_hal.beginTransmission(BQ_I2C_ADDR_WRITE);
	if (i2c_hal.endTransmission() == 0) {
		return true;
	} else {
		return false;
	}
}

// Reset the BQ chip
void bq76952::reset(void) {
	subCommand(0x0012);
}

// Read single cell voltage
byte bq76952::getMfgStatusInitRegister(void) {
	return readDataMemory(0x9343, 1);
}

// Read single cell voltage
int bq76952::getCellVoltage(byte cellNumber) {
	return directCommand(CELL_NO_TO_ADDR(cellNumber));
}

// Read All cell voltages in given array - Call like readAllCellVoltages(&myArray)
void bq76952::getAllCellVoltages(int *cellArray) {
	for (byte x = 0; x < 16; x++)
		cellArray[x] = getCellVoltage(x);
}

void bq76952::getOnlyConnectedCellVoltages(int *cellArray) {
	int allcells[16];
	if (!cellArray)
		return;
	getAllCellVoltages(allcells);
	// Pick only those that are connected
	cellArray[0] = allcells[0];
	cellArray[1] = allcells[1];
	cellArray[2] = allcells[2];
	cellArray[3] = allcells[3];
	cellArray[4] = allcells[5];
	cellArray[5] = allcells[7];
	cellArray[6] = allcells[9];
	cellArray[7] = allcells[11];
	cellArray[8] = allcells[13];
	cellArray[9] = allcells[15];
}

// Measure CC2 current
int bq76952::getCurrent(void) {
	return directCommand(CMD_DIR_CC2_CUR);
}

// Get current Now
int bq76952::getCurrentNow(void) {
	subCommand(0x0075);
	HAL_Delay(1);
	return subCommandResponseInt(22); // CC1 Current
}

// Get current Avg of CC3 Counts (offset 28)
int bq76952::getCurrentAvg(void) {
	subCommand(0x0075);
	HAL_Delay(1);
	return subCommandResponseInt(20); // CC3 Current
}

// Get Manufacturing Status (16 bits)
unsigned int bq76952::getManufacturingStatus(void) {
	subCommand(0x0057);
	HAL_Delay(1);
	return subCommandResponseInt(0); // Manufacturing Status subcommand
}

bool bq76952::areFETs_Enabled(void) {
	unsigned int fet_on = getManufacturingStatus();
	// bit 4: FETs are enabled for device operation,
	// otherwise the device is in FET Test mode.
	return ((fet_on & 0x10) != 0);
}

// Measure Stack voltage. Unit 10mV
unsigned int bq76952::getStackVoltage(void) {
	return directCommand(CMD_READ_VOLTAGE_STACK);
}

// Get Device number. Must return 0x7695
unsigned int bq76952::getDeviceNumber(void) {
	subCommand(CMD_DEVICE_NUMBER);
	HAL_Delay(1);
	return subCommandResponseInt(0);
}

// Get HW version
unsigned int bq76952::getHWVersion(void) {
	subCommand(CMD_HW_VERSION);
	HAL_Delay(1);
	return subCommandResponseInt(0);
}

// Get COV Snapshot.
// When a COV fault is triggered, a snapshot of all cell voltages is captured
// and can be accessed through this command.
unsigned int bq76952::getCOVSnapshot(byte cell) {
	subCommand(CMD_COV_SNAPSHOT);
	HAL_Delay(1);
	return subCommandResponseInt(cell);
}

bool bq76952::is_OTP_already_programmed(void)
{

#ifdef PROGRAM_REGULATORS
	byte reg0;
	byte reg12;
	// by default it's 0x00 in both and we should have 0xDD in
	// REG12_CONTROL and 0x01 in REG0_CONFIG when reg0, reg1 & reg2 enabled
	reg0 = readDataMemory(REG0_CONFIG, 1); // read reg0 configuration
	reg12 = readDataMemory(REG12_CONTROL, 1); // read reg1 and reg2 configuration

	if (reg12 || reg0)
#else
	uint8_t otc_delay; // Over temp
	// Check if we need to program OTP or if it is already programmed
	otc_delay = readDataMemory(0x929B, 1); // read OTC delay

	if (otc_delay != 2) // default is 2
#endif
	{
		return true; // no need to program OTP, reg0, reg1 and reg2 are already on
	}
	return false;
}

/* Security keys */
static uint16_t unseal_key_step_1 = 0;
static uint16_t unseal_key_step_2 = 0;
static uint16_t full_access_key_step_1 = 0;
static uint16_t full_access_key_step_2 = 0;

// Get Security keys
bool bq76952::checkSecurityKeys(void) {
	subCommand(0x0035);
	HAL_Delay(1);
	unseal_key_step_1 = subCommandResponseInt(0);
	subCommand(0x0035);
	HAL_Delay(1);
	unseal_key_step_2 = subCommandResponseInt(2);
	subCommand(0x0035);
	HAL_Delay(1);
	full_access_key_step_1 = subCommandResponseInt(4);
	subCommand(0x0035);
	HAL_Delay(1);
	full_access_key_step_2 = subCommandResponseInt(6);

	// Check full access keys
	if ((full_access_key_step_1 != FULL_ACCESS_KEY_STEP_1) ||
			(full_access_key_step_2 != FULL_ACCESS_KEY_STEP_2))
		return false; // Failed to read keys - Some or all flags are zero.
	else
		return true;
}

bool bq76952::Enter_FullAccessMode(void)
{
	bool sec_key_ok;
	bq76952_battery_status_t batt_st;

	sec_key_ok = checkSecurityKeys();
	// If full access keys are still default, change them
	if (!sec_key_ok)
	{
		// Change full access keys in RAM
		writeDataMemory_WIthoutConfigUpdate(0x925B, FULL_ACCESS_KEY_STEP_1, 2);
		writeDataMemory_WIthoutConfigUpdate(0x925D, FULL_ACCESS_KEY_STEP_2, 2);
	}

	// Enter FULLACCESS mode
	batt_st = getBatteryStatusRegister();
	if (batt_st.bits.SECURITY_STATE == 3) // Device is in SEALED mode
	{
		// We will be here most likely
		subCommand(unseal_key_step_1);
		subCommand(unseal_key_step_2);
		subCommand(full_access_key_step_1);
		subCommand(full_access_key_step_2);
	} else if (batt_st.bits.SECURITY_STATE == 2) // Device is in UNSEALED mode
	{
		subCommand(full_access_key_step_1);
		subCommand(full_access_key_step_2);
	} else if (batt_st.bits.SECURITY_STATE == 1) // Device is in FULLACCESS mode
	{
		// Don't do anything, we already in full access mode
	} else // 0 - device is not initialized yet
	{

	}
	// Check again
	batt_st = getBatteryStatusRegister();
	return (batt_st.bits.SECURITY_STATE == 1);
}

/*
 * All configurations in this function will be written to OTP.
 * Make sure they are correct because OTP can be written only once.
 * Well, actually twice, but that should be avoided.
 */
bool bq76952::configure_before_OTP_write(void)
{
	[[maybe_unused]]uint8_t otc_delay = 3; // default=2 (units 1s)
#ifdef ENTER_FULL_ACCESS_MODE
	bool full_access;

	full_access = Enter_FullAccessMode();
	if (!full_access)
		return false; // Failed to enter full access mode
#endif
#ifdef PROGRAM_REGULATORS
	// We don't enable reg0 because current to REG1 and REG2 is taken from REGIN (pin 36)
	setEnablePreRegulator();
	setEnableRegulator(true, true); // enable reg1 and reg2
#else
	// set OTC delay, picked randomly - just to test OTP
	writeDataMemory_WIthoutConfigUpdate(0x929B, otc_delay, 1);
#endif
	return true;
}

// Use this function only when all configurations are tested and OK to be flashed to OTP.
bool bq76952::program_OTP(void)
{
	bool already_programmed;
	byte opt_wr_check;
	byte otp_write_response; // same format as for opt_wr_check
	bq76952_battery_status_t batt_st;
	bool configured_ok;

	// Check if we need to program OTP or if it is already programmed
	already_programmed = is_OTP_already_programmed();
	if (already_programmed)
	{
		return false; // no need to program OTP, reg0, reg1 and reg2 are already on
	}

	// Set configurations that we want to be copied to OTP
	configured_ok = configure_before_OTP_write();
	if (!configured_ok)
	{
		return false;
	}

	// ========== Enter Config update
	enterConfigUpdate();
	// Initiate a self-check whether OTP writing can be accomplished.
	subCommand(SUBCMD_OTP_WR_CHECK);
	osDelay(1000);
	// Return bits
	// bit 7: Programming OK
	// bit 5: Locked - The device is not in FULLACCESS and CONFIG_UPDATE mode.
	// bit 4: No_SIG - Signature cannot be written
	// bit 3: No_DATA - Could not program data (indicating data has been programmed too many times; no XOR bits left)
	// bit 2: HighTemp - The measured internal temperature is above the allowed OTP programming temperature range
	// bit 1: LowVoltage - The measured stack voltage is below the allowed OTP programming voltage.
	// bit 0: HighVoltage - The measured stack voltage is above the allowed OTP programming voltage.
	opt_wr_check = subCommandResponseInt(0);
	batt_st = getBatteryStatusRegister();
	if (((opt_wr_check & 0x80) == 0) || batt_st.bits.WR_TO_OTP_BLOCKED)
		return false;	// We cannot write to OTP

	subCommand(SUBCMD_OTP_WRITE); /* !!! THIS SHOULD HAPPEN ONLY ONCE IN A WHOLE LIFE OF SINGLE PCB !!! */
	otp_write_response = subCommandResponseInt(0);
	if ((otp_write_response & 0x81) != 0x81)
		return false;	// We failed to write to OTP
	// taking approximately 200 μs per byte programmed
	osDelay(10); /* give it some time to program OTP */
	// =========== Exit Config update mode ==================
	exitConfigUpdate();
	return true;
}

// Measure chip temperature in °C
float bq76952::getInternalTemp(void) {
	float raw = directCommand(CMD_DIR_INT_TEMP) / 10.0;
	return (raw - 273.15);
}

// Measure thermistor temperature in °C
float bq76952::getThermistorTemp(bq76952_thermistor thermistor) {
	byte cmd;
	switch (thermistor) {
	case TS1:
		cmd = 0x70;
		break;
	case TS2:
		cmd = 0x72;
		break;
	case TS3:
		cmd = 0x74;
		break;
	case HDQ:
		cmd = 0x76;
		break;
	case DCHG:
		cmd = 0x78;
		break;
	case DDSG:
		cmd = 0x7A;
		break;
	}
	float raw = directCommand(cmd) / 10.0;
	return (raw - 273.15);
}
#define readBit(value, bit) (((value) >> (bit)) & 0x01)
// Check Primary Protection status

// Safety Status A register
bq76952_protection_t bq76952::getProtectionStatus(void) {
	byte regData = (byte) directCommand(CMD_DIR_SAFETY_STATUS_A);
	prot.bits.SC_DCHG = readBit(regData, BIT_SA_SC_DCHG);
	prot.bits.OC2_DCHG = readBit(regData, BIT_SA_OC2_DCHG);
	prot.bits.OC1_DCHG = readBit(regData, BIT_SA_OC1_DCHG);
	prot.bits.OC_CHG = readBit(regData, BIT_SA_OC_CHG);
	prot.bits.CELL_OV = readBit(regData, BIT_SA_CELL_OV);
	prot.bits.CELL_UV = readBit(regData, BIT_SA_CELL_UV);
	return prot;
}

// Safety Alert C register (direct cmd 0x06)
bq76952_safety_alert_c_t bq76952::getSafetyAlert_C(void) {
	byte regData = (byte) directCommand(CMD_DIR_SAFETY_ALERT_C);
	safety_alert_c.bits.OCD3 = readBit(regData, 7);
	safety_alert_c.bits.SCDL = readBit(regData, 6);
	safety_alert_c.bits.OCDL = readBit(regData, 5);
	safety_alert_c.bits.COVL = readBit(regData, 4);
	safety_alert_c.bits.PTOS = readBit(regData, 3);
	return safety_alert_c;
}

// Check Temperature Protection status
bq76952_temp_t bq76952::getTemperatureStatus(void) {
	bq76952_temp_t prot;
	byte regData = (byte) directCommand(CMD_DIR_FTEMP);
	prot.bits.OVERTEMP_FET = readBit(regData, BIT_SB_OTC);
	prot.bits.OVERTEMP_INTERNAL = readBit(regData, BIT_SB_OTINT);
	prot.bits.OVERTEMP_DCHG = readBit(regData, BIT_SB_OTD);
	prot.bits.OVERTEMP_CHG = readBit(regData, BIT_SB_OTC);
	prot.bits.UNDERTEMP_INTERNAL = readBit(regData, BIT_SB_UTINT);
	prot.bits.UNDERTEMP_DCHG = readBit(regData, BIT_SB_UTD);
	prot.bits.UNDERTEMP_CHG = readBit(regData, BIT_SB_UTC);
	return prot;
}

void bq76952::setFET(bq76952_fet fet, bq76952_fet_state state) {
	unsigned int subcmd;
	switch (state) {
	case OFF:
		switch (fet) {
		case DCH:
			subcmd = 0x0093;
			break;
		case CHG:
			subcmd = 0x0094;
			break;
		default:
			subcmd = 0x0095;
			break;
		}
		break;
	case ON:
		// Allows all FETs to be enabled if nothing else is blocking them
		subcmd = 0x0096;
		break;
	}
	subCommand(subcmd);
}
// Toggle FET_EN in Manufacturing Status. FET_EN = 1 means Firmware FET Control.
void bq76952::setFET_ENABLE(void) {
	subCommand(0x0022);
}


// is Charging FET ON?
bool bq76952::isCharging(void) {
	byte regData = (byte) directCommand(CMD_DIR_FET_STAT);
	if (regData & 0x01) {
		return true;
	}
	return false;
}

// is Discharging FET ON?
bool bq76952::isDischarging(void) {
	byte regData = (byte) directCommand(CMD_DIR_FET_STAT);
	if (regData & 0x04) {
		return true;
	}
	return false;
}

// Set user-defined overvoltage protection
void bq76952::setCellOvervoltageProtection(unsigned int mv, unsigned int ms) {
	byte thresh = (byte) (mv / 50.6);
	uint16_t dly = (uint16_t) (ms / 3.3) - 2;
	if (thresh < 20 || thresh > 110)
		thresh = 86;

	writeDataMemory(0x9278, thresh, 1);

	if (dly < 1 || dly > 2047)
		dly = 74;

	writeDataMemory(0x9279, dly, 2);
}

// Set user-defined undervoltage protection
void bq76952::setCellUndervoltageProtection(unsigned int mv, unsigned int ms) {
	byte thresh = (byte) (mv / 50.6);
	uint16_t dly = (uint16_t) (ms / 3.3) - 2;
	if (thresh < 20 || thresh > 90)
		thresh = 50;

	writeDataMemory(0x9275, thresh, 1);

	if (dly < 1 || dly > 2047)
		dly = 74;

	writeDataMemory(0x9276, dly, 2);
}

// SCD alert or fault triggers when the voltage across the SRN–SRP pins
// exceeds a programmable threshold VSCD. Default value = 0.
// 0=10mV, 1=20mV, 2=40mV, 3=60mV, 4=80mV, 5=100mV, 6=125mV...
// 0 = 10mV, 1 = 20mV, 2 = 40mV, 3 = 60mV, 4 = 80mV, 5 = 100mV
// For 0.01 Ohm sense resistor:
// For example, if we set register to 2, then we'll get short circuit trigger if
// I = V / R = 0.04 / 0.01 = 4A.
void bq76952::setShortCircuitThreshold() {
	//writeDataMemory(SCD_THRESHOLD_CONFIG, 0x02, 1);
	writeDataMemory(SCD_THRESHOLD_CONFIG, 2, 1); // 40A. (0=10mV, 1=20mV, 2=40mV, 3=60mV, 4=80mV, 5=100mV)
	writeDataMemory(SCD_DELAY_CONFIG, 30, 1); // 30*15 = 450us, with units 15us
}

/*
 * default 0x0002. 16-bits
 * bit 10:SCDL_CURR_RECOV: 0 = SCDL does not recover based on charge current.
 *        1 = SCDL recovers when current is greater than or equal to
 *            Protections:SCDL:Recovery Threshold for Protections:SCDL:Recovery Time.
 * bit 9: OCDL_CURR_RECOV: 0 = OCDL does not recover based on charge current.
 *        1 = OCDL recovers when current is greater than or equal to
 *            Protections:OCDL:Recovery Threshold for Protections:OCDL:Recovery Time.
 */
void bq76952::setProtectionConfiguration(void) {
	//writeDataMemory(PROTECTION_CONFIGURATION, 0x00, 2); // disable all
	writeDataMemory(PROTECTION_CONFIGURATION, 0x600, 2); // set only bit 9 (OCDL_CURR_RECOV)
}

// Configures the pack voltage threshold at which
// the device will enter SHUTDOWN mode.
// Unit: 10mV
void bq76952::setShutdownStackVoltage(unsigned int voltage) {
	writeDataMemory(SHUTDOWN_STACK_VOLTAGE, voltage, 2);
}

// Set user-defined charging current protection
// Sets the Overcurrent in Charge Protection threshold for the sense resistor voltage
// in units of 2mV.
void bq76952::setChargingOvercurrentProtection(unsigned int mv, byte ms) {
	byte thresh = (byte) mv / 2;
	byte dly = (byte) (ms / 3.3) - 2;
	if (thresh < 2 || thresh > 62)
		thresh = 2;

	writeDataMemory(0x9280, thresh, 1);

	if (dly < 1 || dly > 127)
		dly = 4;
	writeDataMemory(0x9281, dly, 1);
}

// Set user-defined discharging current protection
// Sets the Overcurrent in Discharge 1st and 2nd Tier Protection threshold for the sense
// resistor voltage in units of 2mV. Sense resistor in e-bike = 0.01 Ohm.
// Then Threshold_mA = mv / 0.01.
void bq76952::setDischargingOvercurrentProtection(unsigned int mv, byte ms) {
	byte thresh = (byte) mv / 2;
	byte dly = (byte) (ms / 3.3) - 2;
	if (thresh < 2 || thresh > 100)
		thresh = 2;

	writeDataMemory(0x9282, thresh, 1); // OCD1 1st Tier Protection threshold
	HAL_Delay(2);
	// At the same time program OCD2 2st Tier Protection threshold
	writeDataMemory(0x9284, thresh, 1);

	if (dly < 1 || dly > 127) {
		dly = 1;
	}
	writeDataMemory(0x9283, dly, 1);
}

// DA should be configured to userA as mA
void bq76952::setDischargingOvercurrentProtection_OCD3(int16_t mA) {
	writeDataMemory(0x928A, mA, 2); // Discharge current protection: OCD3 3st Tier Protection threshold
}

/*
 * This sets the OCD recovery threshold for Overcurrent in Discharge 1st, 2nd, and 3rd Tier Protections.
 * Measured current must be greater than or equal to this threshold for Protections:Recovery:Time to recover.
 * Note the sign of current when configuring this parameter; by default it requires charge current above this
 * threshold. OCD protection.
 * default 200mA (units: mA). 16 bits signed
 */
void bq76952::setDischargingOvercurrentProtection_Recovery(int16_t mA) {
	writeDataMemory(0x928D, mA, 2); // OCD recovery (mA)
}


// Set user-defined discharging current protection
void bq76952::setDischargingShortcircuitProtection(bq76952_scd_thresh thresh,
		unsigned int us) {
	byte dly = (byte) (us / 15) + 1;
	writeDataMemory(0x9286, thresh, 1);
	if (dly < 1 || dly > 31)
		dly = 2;
	writeDataMemory(0x9287, dly, 1);
}

// Set user-defined charging over temperature protection
void bq76952::setChargingTemperatureMaxLimit(signed int temp, byte sec) {
	if (temp < -40 || temp > 120)
		temp = 55;

	writeDataMemory(0x929A, temp, 1);

	if (sec < 0 || sec > 255)
		sec = 2;

	writeDataMemory(0x929B, sec, 1);
}

// Set user-defined discharging over temperature protection
void bq76952::setDischargingTemperatureMaxLimit(signed int temp, byte sec) {
	if (temp < -40 || temp > 120)
		temp = 60;

	writeDataMemory(0x929D, temp, 1);

	if (sec < 0 || sec > 255)
		sec = 2;
	writeDataMemory(0x929E, sec, 1);
}

// Set user-defined byte value to register
// bit0: REG0_EN pre-regulator. This must be enabled if the
void bq76952::setEnablePreRegulator() {
	writeDataMemory(REG0_CONFIG, 0x01, 1);
}
// Current measurement units (USER_AMPS_1/0), TINT_FETT, TINT_EN, USER_VOLTS_CV.
void bq76952::setDA_Config(void) {
	// bit 4 TINT_FETT = 0: Internal temperature is not used for "FET Temperature"
	// bit 3 TINT_EN = 0: Internal temperature is not used for "Cell Temperature"
	// bit 2 USER_VOLTS_CV = 0 => Millivolt (1 mV) units are selected for user-volts
	// bits 1-0 USER_AMPS = 1 => Milliamp (1 mA) units are selected for user-amps
	writeDataMemory(DA_CONFIGURATION, 0x01, 1);
}

// selects which protections influence the setting of AlarmRawStatus()
// bits: 7: SCD, 6: OCD2, 5: OCD1, 4: OCC, 3: COV, 2: CUV
void bq76952::setSF_AlertMask_A(void) {
	writeDataMemory(SF_ALERT_MASK_A, 0x00, 1);
}

// selects which protections influence the setting of AlarmRawStatus()
// bits: 7: OTF, 6: OTINT, 5: OTD, 4: OTC, 2: UTINT, 1: UTD, 0: UTC
void bq76952::setSF_AlertMask_B(void) {

	writeDataMemory(SF_ALERT_MASK_B, 0x00, 1);
}

// selects which protections influence the setting of AlarmRawStatus()
void bq76952::setSF_AlertMask_C(void) {

	writeDataMemory(SF_ALERT_MASK_C, 0x00, 1);
}

// Set voltage regulator settings
void bq76952::setEnableRegulator(bool enable_reg1, bool enable_reg2) {
	byte reg12 = 0xCC; // set reg1 and reg2 to 3.3V (bits 3-1, 7-5)
	reg12 |= (enable_reg1)? 0x01: 0;
	reg12 |= (enable_reg2)? 0x10: 0;

	writeDataMemory(REG12_CONTROL, reg12, 1);
}

// Set alert configurations
void bq76952::setAlertPinConfig(void) {
	// bit 1-0: b10 = ALERT
	// bit 3-2: b10 = Thermistor temperature measurement, reported but not used for protections
	// bit 5-4: b10 = selects the Custom Temperature Model
	writeDataMemory(ALERT_PIN_CONFIG, 0x2A, 1);
}

// Set Default Alarm Mask (0x926D)
// bits: 15: SSBC, 14: SSA, 13: PF, 12: MSK_SFALERT, 11: MSK_PFALERT, 10: INITSTART, 9: INITCOMP
// 7: FULLSCAN, 6: XCHG, 5: XDSG, 4: SHUTV, 3: FUSE, 2: CB, 1: ADSCAN, 0: WAKE
void bq76952::setDefaultAlarmMaskConfig(void) {
	writeDataMemory(DEFAULT_ALARM_MASK_CONFIG, 0xF800, 2); // default is 0xF800
}

// Set Vcell mode
void bq76952::setVcellMode(uint16_t vcell_mode) {
	writeDataMemory(VCELL_MODE, vcell_mode, 2);
}
// Configure all CHG FET protection registers (A, B and C) at once
void bq76952::setEnableCHG_FET_Protection(void) {
	writeDataMemory(CHG_FET_PROTECTION_A, 0x00, 1);
	writeDataMemory(CHG_FET_PROTECTION_B, 0x00, 1);
	writeDataMemory(CHG_FET_PROTECTION_C, 0x00, 1);
}

// Enable protections A
// bit 7: SCD Short Circuit in Discharge Protection
// bit 6: OCD2 Overcurrent in Discharge 2nd Tier Protection
// bit 5: OCD1 Overcurrent in Discharge 1st Tier Protection
// bit 4: OCC Overcurrent in Charge Protection
// bit 3: COV Cell Overvoltage Protection
// bit 2: CUV Cell Undervoltage Protection
void bq76952::setEnableProtectionsA(void) {
#ifdef RELEASE
	writeDataMemory(ENABLE_PROTECTIONS_A, 0xFC, 1); // enable all
#else
	//writeDataMemory(ENABLE_PROTECTIONS_A, 0x00, 1); // disable all
	//writeDataMemory(ENABLE_PROTECTIONS_A, 0x10, 1); // enable only bit 4: Overcurrent in Charge Protection
	//writeDataMemory(ENABLE_PROTECTIONS_A, 0xF0, 1); // enable SCD, OCD2, OCD1, OCC.
	writeDataMemory(ENABLE_PROTECTIONS_A, 0xFC, 1); // Enable all
#endif
}

// Enable all protections B
void bq76952::setEnableProtectionsB(void) {
	writeDataMemory(ENABLE_PROTECTIONS_B, 0xF7, 1);
}

// Enable all protections C
void bq76952::setEnableProtectionsC(void) {
	writeDataMemory(ENABLE_PROTECTIONS_C, 0x80, 1);
}

// CHG FET Protections A, default 0x98.
// If all zeros - all protections (COV, OCC, SCD) are disabled
void bq76952::setCHGFETProtectionsA(byte val) {
	writeDataMemory(CHG_FET_PROTECTIONS_A, val, 1);
}

// Cell Interconnect Resistances cell 0..15.
// default 0 mOhm
void bq76952::setCellInterconnectResistances(void) {
	byte cell;

	for (cell = 0; cell < 16; cell++) {
		writeDataMemory(CELL_INTERCONNECT_RESISTANCE + (cell * 2), CELL_INTERCONNECT_RESISTANCE_MOHM, 2);
	}
}

// bit=1 means DSG FET is disabled when protection is triggered.
// bit=0 means DSG FET is not disabled when protection is triggered
// bit 7: SCD (Short Circuit in Discharge Protection)
// bit 6: OCD2 (Overcurrent in Discharge 2nd Tier Protection)
// bit 5: OCD1 (Overcurrent in Discharge 1st Tier Protection)
// bit 2: CUV (Cell Undervoltage Protection)
// default is 0xE4
void bq76952::setDSGFETProtectionsA(void) {
#ifdef RELEASE
	writeDataMemory(DSG_FET_PROTECTION_A, 0xE4, 1);
#else
	//writeDataMemory(DSG_FET_PROTECTION_A, 0, 1); // disable all protections
	//writeDataMemory(DSG_FET_PROTECTION_A, 0x20, 1); // enable only OCD1 bit 5
    //writeDataMemory(DSG_FET_PROTECTION_A, 0x24, 1);   // enable CUV and OCD1
	//writeDataMemory(DSG_FET_PROTECTION_A, 0xE0, 1); // enable SCD, OCD2, OCD1, CUV
	//writeDataMemory(DSG_FET_PROTECTION_A, 0x04, 1); // enable only CUV (bit 2)
	writeDataMemory(DSG_FET_PROTECTION_A, 0xA4, 1);   // enable CUV (bit 2) and OCD1(bit 5), SCD(bit 7)
#endif
}

// bit=1 means DSG FET is disabled when protection is triggered.
// bit=0 means DSG FET is not disabled when protection is triggered
// bit 7: OTF (FET Overtemperature)
// bit 6: OTINT (Internal Overtemperature)
// bit 5: OTD (Overtemperature in Discharge)
// bit 2: UTINT (Internal Undertemperature)
// bit 1: UTD (Undertemperature in Discharge)
// default is 0xE6
void bq76952::setDSGFETProtectionsB(void) {
#ifdef RELEASE
	writeDataMemory(DSG_FET_PROTECTION_B, 0xE6, 1);
#else
	writeDataMemory(DSG_FET_PROTECTION_B, 0, 1); // disable all protections
#endif
}

// bit=1 means DSG FET is disabled when protection is triggered.
// bit=0 means DSG FET is not disabled when protection is triggered
// bit 7: OCD3 (Overcurrent in Discharge 3rd Tier Protection)
// bit 6: SCDL (Short Circuit in Discharge Latch)
// bit 5: OCDL (Overcurrent in Discharge Latch)
// bit 1: HWDF (Host Watchdog Fault)
// default is 0xE2
void bq76952::setDSGFETProtectionsC(void) {
#ifdef RELEASE
	writeDataMemory(DSG_FET_PROTECTION_C, 0xE2, 1);
#else
	//writeDataMemory(DSG_FET_PROTECTION_C, 0, 1); // disable all protections
	//writeDataMemory(DSG_FET_PROTECTION_C, 0x80, 1); // Enable OCD3 in DSG FET Protections C
	writeDataMemory(DSG_FET_PROTECTION_C, 0x80, 1); // enable OCD3
#endif
}

/*
 *
 * default 0x0D
 */
void bq76952::setFET_Options(void) {

	/*
	 * To reduce inrush current when the DSG FET turns on, the PDSG FET can be
	 * enabled for a short time first to charge up the load through a higher-resistance
	 * path. This bit enables this operation.
	 */
	writeDataMemory(FET_OPTIONS, 0x1D, 1); // Enable PDSG_EN, FET_CTRL_EN, HOST_FET_EN, SFET
}

/*
 * default 5 * 10ms = 50ms. Max 255.
 */
void bq76952::setFET_PredischargeTimeout(void) {

	//writeDataMemory(FET_PREDISCHARGE_TIMEOUT, 200, 1); // 2 sec (with 10ms unit)
	writeDataMemory(FET_PREDISCHARGE_TIMEOUT, 0, 1); // No timeout
}

/*
 * default 50 * 10mV = 500mV. Max 255.
 */
void bq76952::setFET_PredischargeStopDelta(void) {

	writeDataMemory(FET_PREDISCHARGE_STOP_DELTA, 100, 1); // 500mV (with 10mV unit)
}




// Set TS1 to measure Cell Temperature
void bq76952::setEnableTS1(void) {
	//writeDataMemory(TS1_CONFIG, 0x07, 1);
	writeDataMemory(TS1_CONFIG, 0x00, 1);
}
// Set TS2 to measure Cell Temperature
void bq76952::setEnableTS2(void) {
	writeDataMemory(TS2_CONFIG, 0x00, 1); // TS2 not used
}
// Set TS3 to measure Cell Temperature
void bq76952::setEnableTS3(void) {
	//writeDataMemory(TS3_CONFIG, 0x07, 1);
	writeDataMemory(TS3_CONFIG, 0x00, 1);
}


// Read alert status register
// FIXME parse bits
unsigned int bq76952::getAlertStatusRegister(void) {
	unsigned int regData = (unsigned int) directCommand(CMD_DIR_ALARM_STATUS);
	return regData;
}

// Provides the present (unlatched) value of the bits described Alarm Raw Status
unsigned int bq76952::getAlertRawStatusRegister(void) {
	unsigned int regData = (unsigned int) directCommand(CMD_DIR_ALARM_RAW_STATUS);
	return regData;
}

// Get Battery status
bq76952_battery_status_t bq76952::getBatteryStatusRegister(void) {
	bq76952_battery_status_t batt_stat;
	unsigned int regData = (unsigned int) directCommand(CMD_DIR_BATTERY_STATUS);
  	batt_stat.bits.SLEEP_MODE = readBit(regData, 15);
	batt_stat.bits.SHUTDOWN_PENDING = readBit(regData, 13);
	batt_stat.bits.PERMANENT_FAULT = readBit(regData, 12);
	batt_stat.bits.SAFETY_FAULT = readBit(regData, 11);
	batt_stat.bits.FUSE_PIN = readBit(regData, 10);
	batt_stat.bits.SECURITY_STATE = (((regData) >> (8)) & 0x03); // bits 9-8
	batt_stat.bits.WR_TO_OTP_BLOCKED = readBit(regData, 7);
	batt_stat.bits.WR_TO_OTP_PENDING = readBit(regData, 6);
	batt_stat.bits.OPEN_WIRE_CHECK = readBit(regData, 5);
	batt_stat.bits.WD_WAS_TRIGGERED = readBit(regData, 4);
	batt_stat.bits.FULL_RESET_OCCURED = readBit(regData, 3);
	batt_stat.bits.SLEEP_EN_ALLOWED = readBit(regData, 2);
	batt_stat.bits.PRECHARGE_MODE = readBit(regData, 1);
	batt_stat.bits.CONFIG_UPDATE_MODE = readBit(regData, 0);
	return batt_stat;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
///// exposed C functions /////
////////////////////////////////////////////////////////////////////////////////////////////////////////
extern "C" {
// Call this function with I2C interface wired to BQ76952
// Example how to call this function:
// 	bq76952_init(&hi2c3);
void bq76952_init(I2C_HandleTypeDef *hi2c) {
	 [[maybe_unused]]unsigned int hwVersion;
	 [[maybe_unused]]unsigned int devNumber;

	bq76952& bms = bq76952::create(hi2c);
	bms.begin();

	//bms.reset();
	HAL_Delay(1000);

	// ======= E-bite master MCU configurations =======
	//bms.setShutdownStackVoltage(2); // set shutdown voltage to 10mV for debug (default is 6V)
	devNumber = bms.getDeviceNumber();
	hwVersion = bms.getHWVersion();
#ifdef PROGRAM_OTP
	bms.program_OTP();
#endif

#ifdef RELEASE
	bms.setVcellMode(0xAAAF);	// Configure all 10 Vcells connected according to schematic
#else
	bms.setVcellMode(0xAAAF);	// Configure all 10 Vcells connected according to schematic
	//bms.setVcellMode(0xA000);	// Configure only 10th and 9th Vcell for debug only
#endif
	//bms.setEnablePreRegulator();
	//bms.setEnableRegulator(true, true); // enable reg1 and reg2
	bms.setDA_Config(); // use mV and mA units
	// Enable all protections in Enabled Protections A, B and C
	bms.setEnableProtectionsA();
	//bms.setEnableProtectionsB();
	bms.setEnableProtectionsC();
	bms.setEnableCHG_FET_Protection(); // Disable all CHG FET protections (A,B,C)

	bms.setProtectionConfiguration();
	//bms.setSF_AlertMask_A();
	//bms.setSF_AlertMask_B();
	//bms.setSF_AlertMask_C();
	bms.setEnableTS1();
	//bms.setEnableTS3();
	//bms.setAlertPinConfig(); // but it's probably OK to keep it
	//bms.setDefaultAlarmMaskConfig();
	bms.setShortCircuitThreshold();
	//bms.setCellInterconnectResistances();

	bms.setFET_Options();
	bms.setFET_PredischargeTimeout();
	bms.setFET_PredischargeStopDelta();

	bms.setDischargingOvercurrentProtection(4, 255); // 1A
	bms.setDischargingOvercurrentProtection_Recovery(-3000); // => 0.9A
	bms.setDischargingOvercurrentProtection_OCD3(-1000); // 100mA (default -4000 userA)

	//bms.setDischargingOvercurrentProtection(10, 1); // 20mV => 0.02 / 0.01 = 2A
	//bms.setDischargingOvercurrentProtection(100, 1); // 100 = 200mV => 20A
	//auto tmp = bms.readDataMemory(0x9282, 1); // just to verify that we get what we wrote
	//bms.setChargingOvercurrentProtection(18, 4); // 18mV => 0.018 / 0.01 = 1.8A
//	bms.setChargingOvercurrentProtection(62, 4); // (max is 12.4A=62) Should be 25 => 5A (150W charging)
	//bms.setChargingOvercurrentProtection(25, 4); // (max is 12.4A=62) Should be 25 => 5A (150W charging)
	//bms.setChargingOvercurrentProtection(5, 4); // 1A for test only!
	//bms.setChargingOvercurrentProtection(2, 4); // 400mA for test only!

	bms.setCellOvervoltageProtection(4200, 100);
	//bms.setCellUndervoltageProtection(3200, 100);
	bms.setCellUndervoltageProtection(1500, 100);
	// Disable all discharge protections
	bms.setDSGFETProtectionsA();
	bms.setDSGFETProtectionsB();
	bms.setDSGFETProtectionsC();

	HAL_Delay(500); // wait for BQ to initialize
	// Returns 0xFF for some reasons. Most likely bug in lib API when reading memory
	if (!bms.areFETs_Enabled())
	{
		bms.setFET_ENABLE();	// Let BQ control FETs
	}

	//bms.setFET(DCH, ON);	// Turn DSG FET on
	HAL_Delay(500); // let it enable FETs
	bms.getAlertRawStatusRegister();
}

// Called from EXTI1_IRQHandler() when bq76952 sets an alert pin.
void bq76952_handle_alarm(void)
{
	bq76952::HandleAlarm();
}

void bq76952_check_batt_status(void)
{
	int cellArray[16];
	[[maybe_unused]]bool isDischarging;
	[[maybe_unused]]bool isCharging;
	bq76952_battery_status_t batt_status;
	[[maybe_unused]]unsigned int alarmStatus;
	[[maybe_unused]]unsigned int cov[16];
	int i;

	bq76952& bms = bq76952::getInstance();
	osDelay(50);
	bms.getAllCellVoltages(cellArray);
	alarmStatus = bms.getAlertStatusRegister();
	isDischarging = bms.isDischarging();
	isCharging = bms.isCharging();
	batt_status = bms.getBatteryStatusRegister();
	// Determine if any RAM configuration changes were lost due to a reset.
	if (batt_status.bits.FULL_RESET_OCCURED) {
		// Handle unexpected BQ reset
	}

	// ===========================
	// These two lines are just for testing if we read what we wrote before
	[[maybe_unused]]auto tmp_debug1 = bms.readDataMemory(0x9236, 1); // read config
	[[maybe_unused]]uint8_t tmp_debug2 = bms.readDataMemory(0x9286, 1); // read config

	[[maybe_unused]]unsigned int manufacturing_status = bms.getManufacturingStatus(); // FET_EN: bit 4
	[[maybe_unused]]auto current_now = bms.getCurrentNow();
	[[maybe_unused]]bq76952_temp_t tStat = bms.getTemperatureStatus();
	[[maybe_unused]]auto temp=bms.getInternalTemp();
	[[maybe_unused]]auto stackV = bms.getStackVoltage();
	[[maybe_unused]]auto cellv=bms.getCellVoltage(15);
	[[maybe_unused]]auto alert_raw = bms.getAlertRawStatusRegister();
	bq76952_protection_t status = bms.getProtectionStatus(); // safety status A register
	[[maybe_unused]]bq76952_safety_alert_c_t safety_alert_c = bms.getSafetyAlert_C();
	// If any cell overvoltage was detected, read snapshot of all cell voltages
	if (status.bits.CELL_OV) {
		for (i = 0; i < 16; i++) {
		cov[i] = bms.getCOVSnapshot(i);
		}
	}
	// This is a Short Circuit alarm in discharge path.
	if (status.bits.SC_DCHG) {
		// FIXME Handle it
	}

}


}



