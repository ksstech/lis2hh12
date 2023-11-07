/*
 * lis2hh12.c - Copyright (c) 2022-23 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_config.h"

#if (halHAS_LIS2HH12 > 0)
#include "endpoints.h"
#include "hal_i2c_common.h"
#include "lis2hh12.h"
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

// ################################ Forward function declaration ###################################


// ######################################### Constants #############################################

const u16_t fs_scale[4] = { 2000, -1, 4000, 8000 };
const u16_t odr_scale[8] = { 0, 10, 50, 100, 200, 400, 800, -1 };

// ###################################### Local variables ##########################################

lis2hh12_t sLIS2HH12 = { 0 };
u32_t lis2hh12IRQsOK, lis2hh12IRQsLost;

// #################################### Local ONLY functions #######################################

int lis2hh12ReadRegs(u8_t Reg, u8_t * pRxBuf, size_t RxSize) {
	xRtosSemaphoreTake(&sLIS2HH12.mux, portMAX_DELAY);
	IF_SYSTIMER_START(debugTIMING, stLIS2HH12);
	int iRV = halI2C_Queue(sLIS2HH12.psI2C, i2cWR_B, &Reg, sizeof(Reg), pRxBuf, RxSize, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stLIS2HH12);
	xRtosSemaphoreGive(&sLIS2HH12.mux);
	return iRV;
}

int lis2hh12WriteReg(u8_t reg, u8_t val) {
	u8_t u8Buf[2] = { reg, val };
	xRtosSemaphoreTake(&sLIS2HH12.mux, portMAX_DELAY);
	IF_SYSTIMER_START(debugTIMING, stLIS2HH12);
	int iRV = halI2C_Queue(sLIS2HH12.psI2C, i2cW, u8Buf, sizeof(u8Buf), NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stLIS2HH12);
	xRtosSemaphoreGive(&sLIS2HH12.mux);
	return iRV;
}

/**
 * @brief	perform a Write-Read-Modify-Write transaction, also updates local register value
 */
int lis2hh12UpdateReg(u8_t reg, u8_t * pRxBuf, u8_t _and, u8_t _or) {
	xRtosSemaphoreTake(&sLIS2HH12.mux, portMAX_DELAY);
	IF_SYSTIMER_START(debugTIMING, stLIS2HH12);
	int iRV = halI2C_Queue(sLIS2HH12.psI2C, i2cWRMW, &reg, sizeof(reg), pRxBuf, 1, (i2cq_p1_t) (u32_t) _and, (i2cq_p2_t) (u32_t) _or);
	IF_SYSTIMER_STOP(debugTIMING, stLIS2HH12);
	xRtosSemaphoreGive(&sLIS2HH12.mux);
	return iRV;
}

// #################################### Interrupt support ##########################################

/**
 *	@brief	Check each input, generate event for every input pulsed
 *	@brief	Called in context of the I2C task
 */
void lis2hh12ReadHandler(void * Arg) {
	//lis2hh12ReportIG_SRC(NULL);
}

void lis2hh12ReadTrigger(void * Arg) {
	u8_t Reg = lis2hh12IG_SRC1;
	xRtosSemaphoreTake(&sLIS2HH12.mux, portMAX_DELAY);
	IF_SYSTIMER_START(debugTIMING, stLIS2HH12);
	halI2C_Queue(sLIS2HH12.psI2C, i2cWRC, &Reg, sizeof(Reg), &sLIS2HH12.Reg.IG_SRC1,
		SO_MEM(lis2hh12_reg_t, IG_SRC1), (i2cq_p1_t)lis2hh12ReadHandler, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stLIS2HH12);
	xRtosSemaphoreGive(&sLIS2HH12.mux);
}

void IRAM_ATTR lis2hh12IntHandler(void * Arg) {
	#define lis2hh12REQ_TASKS (taskI2C_MASK|taskEVENTS_MASK)
	EventBits_t xEBrun = xEventGroupGetBitsFromISR(TaskRunState);
	if ((xEBrun & lis2hh12REQ_TASKS) == lis2hh12REQ_TASKS) {
		xTaskNotifyFromISR(EventsHandle, 1 << evtLIS2HH12IRQ, eSetBits, NULL);
		++lis2hh12IRQsOK;
	} else {
		++lis2hh12IRQsLost;
	}
}

int lis2hh12EventHandler(void) {
	if (sLIS2HH12.Reg.ctrl3.int1_inact) return halGDI_GetState(0) ? kwMOVEMENT : kwINACTIVE;
	return kwNULL;					// no/unknown event
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
	psI2C->CFGok = 0;
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
	psI2C->CFGok = 1;
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

int lis2hh12ReportIG_SRC(report_t * psR) {
	static const char bmIG1_SRC1[] = "XLXHYLYHZLZHIA";
	int iRV = wprintfx(psR, "\tIG_SRC1: x%02X", sLIS2HH12.Reg.IG_SRC1);
	for(u8_t i = 0; i < 7; ++i) {
		if (sLIS2HH12.Reg.IG_SRC1 & (1 << i)) iRV += wprintfx(psR, "%2s ", &bmIG1_SRC1[i*2]);
	}
	iRV += wprintfx(psR, strCRLF);
	return iRV;
}

int lis2hh12ReportCfg(report_t * psR) {
	int iRV = wprintfx(psR, "\tACT_THS: x%02X (%dmg) \r\n", sLIS2HH12.Reg.ACT_THS, sLIS2HH12.Reg.ACT_THS*(fs_scale[sLIS2HH12.Reg.ctrl4.fs]/128));
	iRV +=  wprintfx(psR, "\tACT_DUR: x%02X (%ds) \r\n", sLIS2HH12.Reg.ACT_DUR, sLIS2HH12.Reg.ACT_DUR);
	iRV +=  wprintfx(psR, "\tCTRL1: x%02X  hr=%d  odr=%d/%dHz  bdu=%d  Zen=%d  Yen=%d  Xen=%d\r\n",
		sLIS2HH12.Reg.CTRL1, sLIS2HH12.Reg.ctrl1.hr, sLIS2HH12.Reg.ctrl1.odr, odr_scale[sLIS2HH12.Reg.ctrl1.odr],
		sLIS2HH12.Reg.ctrl1.bdu, sLIS2HH12.Reg.ctrl1.zen, sLIS2HH12.Reg.ctrl1.yen, sLIS2HH12.Reg.ctrl1.xen);
	iRV +=  wprintfx(psR, "\tCTRL4: x%02X  bw=%d  fs=%d/%dG  bws_odr=%d  IAinc=%d  I2Cen=%d  sim=%d\r\n",
		sLIS2HH12.Reg.CTRL4, sLIS2HH12.Reg.ctrl4.bw, sLIS2HH12.Reg.ctrl4.fs, fs_scale[sLIS2HH12.Reg.ctrl4.fs]/1000,
		sLIS2HH12.Reg.ctrl4.bw_scale_odr, sLIS2HH12.Reg.ctrl4.if_add_inc, sLIS2HH12.Reg.ctrl4.i2c_enable, sLIS2HH12.Reg.ctrl4.sim);
	iRV +=  wprintfx(psR, "\tCTRL5: x%02X  dbg=%d  rst=%d  dec=%d  st=%d  HLactive=%d  pp_od=%d\r\n",
		sLIS2HH12.Reg.CTRL5, sLIS2HH12.Reg.ctrl5.debug, sLIS2HH12.Reg.ctrl5.soft_reset, sLIS2HH12.Reg.ctrl5.dec,
		sLIS2HH12.Reg.ctrl5.st, sLIS2HH12.Reg.ctrl5.h_lactive, sLIS2HH12.Reg.ctrl5.pp_od);
	return iRV;
}

int lis2hh12ReportAll(report_t * psR) {
	int iRV = halI2C_DeviceReport(psR, sLIS2HH12.psI2C);
	iRV += lis2hh12ReportCfg(psR);
	iRV += lis2hh12ReportIG_SRC(psR);
	return iRV;
}
#endif
