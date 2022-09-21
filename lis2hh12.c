/*
 * lis12hh12.c
 * Copyright (c0 2022 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "lis2hh12.h"
#include "hal_gpio.h"
#include "hal_variables.h"
#include "endpoints.h"
#include "options.h"
#include "printfx.h"
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
	halI2C_Queue(sLIS2HH12.psI2C, i2cWR_B, &Reg, sizeof(Reg),
			pRxBuf, RxSize, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	xRtosSemaphoreGive(&sLIS2HH12.mux);
}

void lis2hh12WriteReg(u8_t reg, u8_t val) {
	u8_t u8Buf[2] = { reg, val };
	xRtosSemaphoreTake(&sLIS2HH12.mux, portMAX_DELAY);
	halI2C_Queue(sLIS2HH12.psI2C, i2cW, u8Buf, sizeof(u8Buf), NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	xRtosSemaphoreGive(&sLIS2HH12.mux);
}

/**
 * perform a Write-Modify-Read transaction, also updates local register value
 */
void lis2hh12UpdateReg(u8_t reg, u8_t * pRxBuf, u8_t _and, u8_t _or) {
	xRtosSemaphoreTake(&sLIS2HH12.mux, portMAX_DELAY);
	halI2C_Queue(sLIS2HH12.psI2C, i2cWRMW_B, &reg, sizeof(reg), pRxBuf, 1,
			(i2cq_p1_t) (uint32_t) _and, (i2cq_p2_t) (uint32_t) _or);
	xRtosSemaphoreGive(&sLIS2HH12.mux);
}

// #################################### Interrupt support ##########################################

int lis2hh12EventHandler(void) {
	if (sLIS2HH12.Reg.ctrl3.int1_inact)
		return halGPDI_GetState(0) ? kwMOVEMENT : kwINACTIVE;
	return kwNULL;					// no/unknown event
}

// #################################  IRMACOS sensor task support ##################################

int	lis2hh12ReadHdlrAccel(epw_t * psEWP) {
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
	if (ioB1GET(dbgLIS2HH12)) {
		if (sLIS2HH12.Reg.status.zyxor)
			P("LIS2HH12 ZYX overrun");
	}
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
int	lis2hh12Identify(i2c_di_t * psI2C_DI) {
	psI2C_DI->TRXmS	= 50;
	psI2C_DI->CLKuS = 400;
	psI2C_DI->Test = 1;
	sLIS2HH12.psI2C = psI2C_DI;

	u8_t U8;
	int iRV;
	lis2hh12ReadRegs(lis2hh12WHO_AM_I, &U8, sizeof(U8));
	IF_EXIT_X(U8 != lis2hh12WHOAMI_NUM, erFAILURE);
	psI2C_DI->Type		= i2cDEV_LIS2HH12;
	psI2C_DI->Speed		= i2cSPEED_400;
	psI2C_DI->DevIdx 	= 0;
	iRV = erSUCCESS;
exit:
	psI2C_DI->Test = 0;
	return iRV ;
}

int	lis2hh12Config(i2c_di_t * psI2C_DI) {
#if 1
	// enable device
	lis2hh12UpdateReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, 0xFF, 0x3F); 	// XYZen ODR=100Hz BDU
	// enable In/Activity interrupt
	lis2hh12UpdateReg(lis2hh12CTRL3, &sLIS2HH12.Reg.CTRL3, 0xFF, 1 << 5);	// INT1_INACT
#else
	lis2hh12WriteReg(lis2hh12CTRL1, sLIS2HH12.Reg.CTRL1 = 0x3F); 			// ODR = 10Hz
	lis2hh12WriteReg(lis2hh12CTRL3, sLIS2HH12.Reg.CTRL3 = 0x20);			// INT1_INACT
#endif

	epw_t * psEWP = &table_work[URI_LIS2HH12_X];
	psEWP->var.def = SETDEF_CVAR(0, 0, vtVALUE, cvF32, 1);
	psEWP->Tsns = psEWP->Rsns = LIS2HH12_T_SNS;
	psEWP->uri = URI_LIS2HH12_X;

	psEWP = &table_work[URI_LIS2HH12_Y];
	psEWP->var.def = SETDEF_CVAR(0, 0, vtVALUE, cvF32, 1);
	psEWP->Tsns = psEWP->Rsns = LIS2HH12_T_SNS;
	psEWP->uri = URI_LIS2HH12_Y;

	psEWP = &table_work[URI_LIS2HH12_Z];
	psEWP->var.def = SETDEF_CVAR(0, 0, vtVALUE, cvF32, 1);
	psEWP->Tsns = psEWP->Rsns = LIS2HH12_T_SNS;
	psEWP->uri = URI_LIS2HH12_Z;

	IF_SYSTIMER_INIT(debugTIMING, stLIS2HH12, stMICROS, "LIS2HH12", 500, 1500);
	return erSUCCESS ;
}

int lis2hh12ReConfig(i2c_di_t * psI2C_DI) { return erSUCCESS; }

int	lis2hh12Diags(i2c_di_t * psI2C_DI) { return erSUCCESS; }

// ######################################### Reporting #############################################

void lis2hh12ReportAll(void) {
	halI2C_DeviceReport(sLIS2HH12.psI2C);
	P("\tACT_THS: 0x%02X (%dmg) \r\n", sLIS2HH12.Reg.ACT_THS, sLIS2HH12.Reg.ACT_THS*(fs_scale[sLIS2HH12.Reg.ctrl4.fs]/128));
	P("\tACT_DUR: 0x%02X (%ds) \r\n", sLIS2HH12.Reg.ACT_DUR, sLIS2HH12.Reg.ACT_DUR);
	P("\tCTRL1: 0x%02X  hr=%d  odr=%d (%dHz) bdu=%d  Zen=%d  Yen=%d  Xen=%d\r\n", sLIS2HH12.Reg.CTRL1,
		sLIS2HH12.Reg.ctrl1.hr, sLIS2HH12.Reg.ctrl1.odr, odr_scale[sLIS2HH12.Reg.ctrl1.odr],
		sLIS2HH12.Reg.ctrl1.bdu,
		sLIS2HH12.Reg.ctrl1.zen, sLIS2HH12.Reg.ctrl1.yen, sLIS2HH12.Reg.ctrl1.xen);
	P("\tCTRL4: 0x%02X  bw=%d  fs=%d (%dG)  bws_odr=%d  IAinc=%d  I2Cen=%d  sim=%d\r\n", sLIS2HH12.Reg.CTRL4,
		sLIS2HH12.Reg.ctrl4.bw, sLIS2HH12.Reg.ctrl4.fs, fs_scale[sLIS2HH12.Reg.ctrl4.fs]/1000,
		sLIS2HH12.Reg.ctrl4.bw_scale_odr,
		sLIS2HH12.Reg.ctrl4.if_add_inc,
		sLIS2HH12.Reg.ctrl4.i2c_enable,
		sLIS2HH12.Reg.ctrl4.sim);
	P("\tCTRL5: 0x%02X  debug=%d  reset=%d  dec=%d  st=%d  HLactive=%d  pp_od=%d\r\n", sLIS2HH12.Reg.CTRL5,
		sLIS2HH12.Reg.ctrl5.debug, sLIS2HH12.Reg.ctrl5.soft_reset, sLIS2HH12.Reg.ctrl5.dec,
		sLIS2HH12.Reg.ctrl5.st, sLIS2HH12.Reg.ctrl5.h_lactive, sLIS2HH12.Reg.ctrl5.pp_od);
//	P("I1: 0x%02x  I2: 0x%02x\r\n", sLIS2HH12.Reg.IG_SRC1, sLIS2HH12.Reg.IG_SRC2);
}
