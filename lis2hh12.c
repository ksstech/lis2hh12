/*
 * lis2hh12.c - Copyright (c) 2022-23 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "endpoints.h"

#if (halHAS_LIS2HH12 > 0)

#include "hal_i2c_common.h"
#include "printfx.h"
#include "rules.h"
#include "syslog.h"
#include "systiming.h"
#include "x_errors_events.h"

#define	debugFLAG					0xF000

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ############################################# Macros ############################################


// #################################### SI7006/13/20/21 Addresses ##################################

#define	lis2hh12ADDR0				0x1E
#define	LIS2HH12_T_SNS				1000

// ################################ Forward function declaration ###################################


// ######################################### Constants #############################################

const gpio_config_t lis2hh12IntPin = { 1ULL<<GPIO_NUM_36, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE};

const u16_t fs_scale[4] = { 2000, -1, 4000, 8000 };
const u16_t odr_scale[8] = { 0, 10, 50, 100, 200, 400, 800, -1 };

// ###################################### Local variables ##########################################

lis2hh12_t sLIS2HH12 = { 0 };

// #################################### Local ONLY functions #######################################

void lis2hh12ReadRegs(u8_t Reg, u8_t * pRxBuf, size_t RxSize) {
	xRtosSemaphoreTake(&sLIS2HH12.mux, portMAX_DELAY);
	halI2C_Queue(sLIS2HH12.psI2C, i2cWR_B, &Reg, sizeof(Reg), pRxBuf, RxSize, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	xRtosSemaphoreGive(&sLIS2HH12.mux);
}

void lis2hh12WriteReg(u8_t reg, u8_t val) {
	u8_t u8Buf[2] = { reg, val };
	xRtosSemaphoreTake(&sLIS2HH12.mux, portMAX_DELAY);
	halI2C_Queue(sLIS2HH12.psI2C, i2cW, u8Buf, sizeof(u8Buf), NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	xRtosSemaphoreGive(&sLIS2HH12.mux);
}

/**
 * @brief	perform a Write-Read-Modify-Write transaction, also updates local register value
 */
void lis2hh12UpdateReg(u8_t reg, u8_t * pRxBuf, u8_t _and, u8_t _or) {
	xRtosSemaphoreTake(&sLIS2HH12.mux, portMAX_DELAY);
	halI2C_Queue(sLIS2HH12.psI2C, i2cWRMW_BD, &reg, sizeof(reg), pRxBuf, 1, (i2cq_p1_t) (u32_t) _and, (i2cq_p2_t) (u32_t) _or);
	xRtosSemaphoreGive(&sLIS2HH12.mux);
}

// #################################### Interrupt support ##########################################

int lis2hh12EventHandler(void) {
	if (sLIS2HH12.Reg.ctrl3.int1_inact)
		return halGDI_GetState(0) ? kwMOVEMENT : kwINACTIVE;
	return kwNULL;					// no/unknown event
}

// #################################  IRMACOS sensor task support ##################################

int	lis2hh12Sense(epw_t * psEWP) {
	IF_SYSTIMER_START(debugTIMING, stLIS2HH12);
	lis2hh12ReadRegs(lis2hh12STATUS, (u8_t *) &sLIS2HH12.Reg.STATUS, 7);
	IF_SYSTIMER_STOP(debugTIMING, stLIS2HH12);
	x64_t X64;
	X64.x32[1].f32 = (float) fs_scale[sLIS2HH12.Reg.ctrl4.fs] / 65536000.0;
	X64.x32[0].f32 = (float) sLIS2HH12.Reg.u16OUT_X * X64.x32[1].f32;
	vCV_SetValueRaw(&table_work[URI_LIS2HH12_X].var, X64);
	X64.x32[0].f32 = (float) sLIS2HH12.Reg.u16OUT_Y * X64.x32[1].f32;
	vCV_SetValueRaw(&table_work[URI_LIS2HH12_Y].var, X64);
	X64.x32[0].f32 = (float) sLIS2HH12.Reg.u16OUT_Z * X64.x32[1].f32;
	vCV_SetValueRaw(&table_work[URI_LIS2HH12_Z].var, X64);
//	P("lis2hh12  [ %-'B ]\r\n", 7, &sLIS2HH12.Reg.STATUS);
	IF_P(ioB1GET(dbgLIS2HH12) && sLIS2HH12.Reg.status.ZYXor, "LIS2HH12 ZYX overrun");
	return erSUCCESS;
}

// ################################ Rules configuration support ####################################

/**
 * Mode	0 = normal ths dur odr hr [etc]
 *		1 = [In]/Active
 *		2 = freefall
 *		3 = orientation 4D portrait/landscape
 *		4 = orientation 6D
 *		5 = stream using FIFO
 */
enum {
	lis2hh12M_NORMAL,
	lis2hh12M_MOVEMENT,
	lis2hh12M_FREEFAAL,
	lis2hh12M_ORIENT4D,
	lis2hh12M_ORIENT6D,
	lis2hh12M_STREAM,
};

int	lis2hh12ConfigMode (struct rule_t * psR, int Xcur, int Xmax, int EI) {
	// mode /lis2hh12 idx ths dur odr hr
	u8_t AI = psR->ActIdx;
	i32_t mode = psR->actPar1[AI];
	i32_t ths = psR->para.x32[AI][0].i32;
	i32_t dur = psR->para.x32[AI][1].i32;
	i32_t odr = psR->para.x32[AI][2].i32;
	i32_t hr = psR->para.x32[AI][3].i32;
	IF_P(debugTRACK && ioB1GET(dbgMode), "lis2hh12: Xcur=%d Xmax=%d ths=%ld dur=%ld odr=%ld hr=%ld\r\n", Xcur, Xmax, ths, dur, odr, hr);

	if (OUTSIDE(lis2hh12M_NORMAL, mode, lis2hh12M_STREAM) ||
		OUTSIDE(0, ths, 127) ||
		OUTSIDE(0, dur, 255) ||
		OUTSIDE(0, odr, 7) ||
		OUTSIDE(0, hr, 1)) {
		RETURN_MX("Invalid ths/dur/odr/hr specified", erINV_PARA);
	}
	do {
		lis2hh12WriteReg(lis2hh12ACT_THS, sLIS2HH12.Reg.ACT_THS = ths);
		lis2hh12WriteReg(lis2hh12ACT_DUR, sLIS2HH12.Reg.ACT_DUR = dur);
		sLIS2HH12.Reg.ctrl1.hr = hr;
		sLIS2HH12.Reg.ctrl1.odr = odr;
		lis2hh12WriteReg(lis2hh12CTRL1, sLIS2HH12.Reg.CTRL1);
		IF_P(debugTRACK && ioB1GET(dbgMode), "lis2hh12: THS=0x%02X  DUR=0x%02X  CTRL1=ox%02X\r\n",
				sLIS2HH12.Reg.ACT_THS,sLIS2HH12.Reg.ACT_DUR, sLIS2HH12.Reg.CTRL1);
	} while (++Xcur < Xmax);
	return erSUCCESS;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	lis2hh12Identify(i2c_di_t * psI2C) {
	sLIS2HH12.psI2C = psI2C;
	psI2C->Type = i2cDEV_LIS2HH12;
	psI2C->Speed = i2cSPEED_400;
	psI2C->TObus = 25;
	psI2C->Test = 1;
	u8_t U8;
//	int iRV = lis2hh12WriteReg(lis2hh12CTRL6, 0x80);	// force REBOOT, must wait till cleared...
	int iRV = lis2hh12WriteReg(lis2hh12CTRL5, 0x40);	// force SOFT RESET, just in case...
	if (iRV < erSUCCESS) goto exit;

	iRV = lis2hh12ReadRegs(lis2hh12WHO_AM_I, &U8, sizeof(U8));
	if (iRV < erSUCCESS) goto exit;
	if (U8 != lis2hh12WHOAMI_NUM) return erINV_WHOAMI;
	psI2C->IDok = 1;
	psI2C->Test = 0;
exit:
	return iRV;
}

int	lis2hh12Config(i2c_di_t * psI2C) {
	if (!psI2C->IDok) return erINV_STATE;

	sLIS2HH12.psI2C->CFGok = 0;
//	psI2C->CFGok = 0;
	int iRV = lis2hh12ReadRegs(lis2hh12ACT_THS, &sLIS2HH12.Reg.ACT_THS, 20);
//	int iRV = lis2hh12ReadRegs(lis2hh12ACT_THS, &sLIS2HH12.Reg.ACT_THS, sizeof(lis2hh12_reg_t));
	if (iRV < erSUCCESS) goto exit;
	#if 1
	iRV = lis2hh12UpdateReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, 0xFF, 0x3F); 	// XYZen ODR=100Hz BDU
	if (iRV < erSUCCESS) goto exit;

	iRV = lis2hh12UpdateReg(lis2hh12CTRL3, &sLIS2HH12.Reg.CTRL3, 0xFF, 0x20);	// INT1_INACT
	if (iRV < erSUCCESS) goto exit;

	#elif 0
	iRV = lis2hh12UpdateReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, 0xFF, 0x3F); 	// XYZen ODR=100Hz BDU
	if (iRV < erSUCCESS) goto exit;

	iRV = lis2hh12UpdateReg(lis2hh12IG_CFG1, &sLIS2HH12.Reg.IG_CFG1, 0xFF, 0x3F);	// Z?IE, Y?IE & X?IE
	if (iRV < erSUCCESS) goto exit;

	iRV = lis2hh12UpdateReg(lis2hh12CTRL3, &sLIS2HH12.Reg.CTRL3, 0xFF, 0x28);		// INT1_INACT & INT1_IG1
	if (iRV < erSUCCESS) goto exit;

	#elif 0
	iRV = lis2hh12WriteReg(lis2hh12CTRL1, sLIS2HH12.Reg.CTRL1 = 0x3F); 			// 100Hz BDU XYZen
	if (iRV < erSUCCESS) goto exit;

	iRV = lis2hh12WriteReg(lis2hh12ACT_THS, sLIS2HH12.Reg.ACT_THS = 0x20);		// 32/128 = 1/4G
	if (iRV < erSUCCESS) goto exit;

	iRV = lis2hh12WriteReg(lis2hh12ACT_DUR, sLIS2HH12.Reg.ACT_DUR = 0x0C);		// (8*12)/100Hz = 0.96s
	if (iRV < erSUCCESS) goto exit;

	iRV = lis2hh12WriteReg(lis2hh12CTRL3, sLIS2HH12.Reg.CTRL3 = 0x20);			// INT1_INACT
	if (iRV < erSUCCESS) goto exit;
	#endif
	sLIS2HH12.psI2C->CFGok = 1;
//	psI2C->CFGok = 1;
	if (psI2C->CFGerr) {
		IF_SYSTIMER_INIT(debugTIMING, stLIS2HH12, stMICROS, "LIS2HH12", 500, 1500);
		const gpio_config_t int_pin_cfg = { .pin_bit_mask = 1ULL<<GPIO_NUM_36,
			.mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_ENABLE, .intr_type = GPIO_INTR_POSEDGE };
		ESP_ERROR_CHECK(gpio_config(&int_pin_cfg));
		halGPIO_IRQconfig(lis2hh12IRQ_PIN, lis2hh12IntHandler, NULL);
	}
exit:
	return iRV;
}

int	lis2hh12Diags(i2c_di_t * psI2C) { return erSUCCESS; }

// ######################################### Reporting #############################################

void lis2hh12ReportAll(report_t * psR) {
	halI2C_DeviceReport(psR, sLIS2HH12.psI2C);
	wprintfx(psR, "\tACT_THS: 0x%02X (%dmg) \r\n", sLIS2HH12.Reg.ACT_THS, sLIS2HH12.Reg.ACT_THS*(fs_scale[sLIS2HH12.Reg.ctrl4.fs]/128));
	wprintfx(psR, "\tACT_DUR: 0x%02X (%ds) \r\n", sLIS2HH12.Reg.ACT_DUR, sLIS2HH12.Reg.ACT_DUR);
	wprintfx(psR, "\tCTRL1: 0x%02X  hr=%d  odr=%d (%dHz) bdu=%d  Zen=%d  Yen=%d  Xen=%d\r\n", sLIS2HH12.Reg.CTRL1,
		sLIS2HH12.Reg.ctrl1.hr, sLIS2HH12.Reg.ctrl1.odr, odr_scale[sLIS2HH12.Reg.ctrl1.odr],
		sLIS2HH12.Reg.ctrl1.bdu,
		sLIS2HH12.Reg.ctrl1.zen, sLIS2HH12.Reg.ctrl1.yen, sLIS2HH12.Reg.ctrl1.xen);
	wprintfx(psR, "\tCTRL4: 0x%02X  bw=%d  fs=%d (%dG)  bws_odr=%d  IAinc=%d  I2Cen=%d  sim=%d\r\n", sLIS2HH12.Reg.CTRL4,
		sLIS2HH12.Reg.ctrl4.bw, sLIS2HH12.Reg.ctrl4.fs, fs_scale[sLIS2HH12.Reg.ctrl4.fs]/1000,
		sLIS2HH12.Reg.ctrl4.bw_scale_odr,
		sLIS2HH12.Reg.ctrl4.if_add_inc,
		sLIS2HH12.Reg.ctrl4.i2c_enable,
		sLIS2HH12.Reg.ctrl4.sim);
	wprintfx(psR, "\tCTRL5: 0x%02X  debug=%d  reset=%d  dec=%d  st=%d  HLactive=%d  pp_od=%d\r\n", sLIS2HH12.Reg.CTRL5,
		sLIS2HH12.Reg.ctrl5.debug, sLIS2HH12.Reg.ctrl5.soft_reset, sLIS2HH12.Reg.ctrl5.dec,
		sLIS2HH12.Reg.ctrl5.st, sLIS2HH12.Reg.ctrl5.h_lactive, sLIS2HH12.Reg.ctrl5.pp_od);
//	wprintfx(psR, "I1: 0x%02x  I2: 0x%02x\r\n", sLIS2HH12.Reg.IG_SRC1, sLIS2HH12.Reg.IG_SRC2);
}
#endif
