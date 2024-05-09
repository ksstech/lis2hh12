// lis2hh12.c - Copyright (c) 2022-24 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"

#if (HAL_LIS2HH12 > 0)
#include "endpoints.h"
#include "hal_i2c_common.h"
#include "hal_options.h"
#include "lis2hh12.h"
#include "printfx.h"
#include "rules.h"
#include "syslog.h"
#include "systiming.h"
#include "x_errors_events.h"

// ############################################# Macros ############################################

#define	debugFLAG					0xF000
#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ######################################### Constants #############################################

const u16_t fs_scale[4] = { 2000, -1, 4000, 8000 };
const u16_t odr_scale[8] = { 0, 10, 50, 100, 200, 400, 800, -1 };

// ###################################### Local variables ##########################################

lis2hh12_t sLIS2HH12 = { 0 };
u32_t lis2hh12IRQok, lis2hh12IRQlost, lis2hh12IRQfifo, lis2hh12IRQig1, lis2hh12IRQig2;
u32_t lis2hh12IRQdrdy, lis2hh12IRQdrdyErr;

// #################################### Local ONLY functions #######################################

int lis2hh12WriteReg(u8_t reg, u8_t * pU8, u8_t val) {
	u8_t u8Buf[2] = { reg, val };
	int iRV = halI2C_Queue(sLIS2HH12.psI2C, i2cW_B, u8Buf, sizeof(u8Buf), NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	if (pU8) *pU8 = val;								// Optionally, store data at location...
	return iRV;
}

int lis2hh12ReadRegs(u8_t reg, u8_t * pU8, size_t size) {
	IF_myASSERT(debugTRACK, INRANGE(lis2hh12TEMP_L, reg, lis2hh12ZH_REF) && size);
	return halI2C_Queue(sLIS2HH12.psI2C, i2cWR_B, &reg, sizeof(reg), pU8, size, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

/**
 * @brief	perform a Write-Read-Modify-Write transaction, also updates local register value
 */
int lis2hh12UpdateReg(u8_t reg, u8_t * pU8, u8_t _and, u8_t _or) {
	IF_myASSERT(debugTRACK, INRANGE(lis2hh12TEMP_L, reg, lis2hh12ZH_REF) && pU8 && halCONFIG_inSRAM(pU8));
	return halI2C_Queue(sLIS2HH12.psI2C, i2cWRMW, &reg, sizeof(reg), pU8, 1, (i2cq_p1_t) (u32_t) _and, (i2cq_p2_t) (u32_t) _or);
}

f32_t lis2hh12ConvCoord(i32_t Val) {
	return (float) Val * (sLIS2HH12.Reg.ctrl4.fs == 0 ? 0.000061 : (sLIS2HH12.Reg.ctrl4.fs == 2) ? 0.000122 : 0.000244);
}

// ################################### Configuration support #######################################

int lis2hh12SetInactivity(u8_t ths, u8_t dur) {
	int iRV ;
	iRV = lis2hh12WriteReg(lis2hh12ACT_THS, &sLIS2HH12.Reg.ACT_THS, ths);	// #of FSD/128 mG
	if (iRV < erSUCCESS)
		goto exit;
	iRV = lis2hh12WriteReg(lis2hh12ACT_DUR, &sLIS2HH12.Reg.ACT_DUR, dur);	// # of (8/ODR) sec
	if (iRV < erSUCCESS)
		goto exit;
	u8_t stat = (ths > 0 || dur > 0) ? 0x20 : 0x00;		// INT1_INACT en/disable?
	iRV = lis2hh12UpdateReg(lis2hh12CTRL3, &sLIS2HH12.Reg.CTRL3, 0xDF, stat);
exit:
	return iRV;
}

int lis2hh12SetOutputDataRate(e_odr_t odr) {
	return lis2hh12UpdateReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, 0x8F, odr << 4);
}

int lis2hh12EnableAxis(e_axis_t axis) {
	return lis2hh12UpdateReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, 0xF8, axis);
}

int lis2hh12SetBDU(bool stat) {
	return lis2hh12UpdateReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, 0xF7, stat << 3);
}

int lis2hh12SetHR(bool hr) {
	return lis2hh12UpdateReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, 0x7F, hr << 7);
}

int lis2hh12SetFS(e_fs_t fs) {
	return lis2hh12UpdateReg(lis2hh12CTRL4, &sLIS2HH12.Reg.CTRL4, 0xCF, fs << 4);
}

int lis2hh12SetBW(e_bw_t bw) {
	return lis2hh12UpdateReg(lis2hh12CTRL4, &sLIS2HH12.Reg.CTRL4, 0x3F, bw << 6);
}

int lis2hh12ConfigFIFO(e_fm_t mode, u8_t thres) {
	int iRV = lis2hh12WriteReg(lis2hh12FIFO_CTRL, &sLIS2HH12.Reg.FIFO_CTRL, (mode << 5) | (thres & 0x1F));
	if (iRV < erSUCCESS)
		return iRV;
	u8_t mask, flag;
	if (mode > fmBYPASS && thres > 0) {
		mask = 0x7F;
		flag = 0x80;
	} else {
		mask = 0x7F;
		flag = 0x80;
	}
	return lis2hh12UpdateReg(lis2hh12CTRL3, &sLIS2HH12.Reg.CTRL3, mask, flag);
}

// #################################### Interrupt support ##########################################

/**
 *	@brief	DRDY IRQ handling
 */
void lis2hh12IntDRDY(void * Arg) {
	if (sLIS2HH12.Reg. status.ZYXda) {							// Data Available?
		lis2hh12ReadRegs(lis2hh12OUT_X_L, &sLIS2HH12.Reg.u8OUT_X[0], lis2hh12OUT_Z_H-lis2hh12OUT_X_L+1);
		++lis2hh12IRQdrdy;
	} else {
		++lis2hh12IRQdrdyErr;
	}
}

/**
 *	@brief	FIFO IRQ handling
 */
void lis2hh12IntFIFO(void * Arg) {
	u8_t count = sLIS2HH12.Reg.fifo_src.fss + sLIS2HH12.Reg.fifo_src.ovr;
	myASSERT(INRANGE(1, count, 32) && sLIS2HH12.Reg.fifo_src.fth);
	while (count) {
		lis2hh12ReadRegs(lis2hh12OUT_X_L, &sLIS2HH12.Reg.u8OUT_X[0], lis2hh12OUT_Z_H-lis2hh12OUT_X_L+1);
		PX("#%d: x=%hd  Y=%hd  Z=%hd\r\n", count, sLIS2HH12.Reg.i16OUT_X, sLIS2HH12.Reg.i16OUT_Y, sLIS2HH12.Reg.i16OUT_Z);
		--count;
	}
	++lis2hh12IRQfifo;
}

/**
 *	@brief	IG1 IRQ handling
 */
void lis2hh12IntIG1(void * Arg) { ++lis2hh12IRQig1; }

/**
 *	@brief	IG2 IRQ handling
 */
void lis2hh12IntIG2(void * Arg) { ++lis2hh12IRQig2; }

/**
 * IRQ context
 */
void IRAM_ATTR lis2hh12IntHandler(void * Arg) {
	int iRV1 = 0, iRV2 = 0, iRV3 = 0, iRV4 = 0, count = 0;
	u8_t Reg;
	if (sLIS2HH12.Reg.ctrl3.int1_drdy) {					// DRDY on INT1 enabled?
		Reg = lis2hh12STATUS;
		iRV1 = halI2C_Queue(sLIS2HH12.psI2C, i2cWRC, &Reg, sizeof(Reg), &sLIS2HH12.Reg.STATUS,
			SO_MEM(lis2hh12_reg_t, STATUS), (i2cq_p1_t)lis2hh12IntDRDY, (i2cq_p2_t) Arg);
		++count;
	}
	if ((sLIS2HH12.Reg.CTRL3 & 0xC6) && sLIS2HH12.Reg.fifo_ctrl.fmode) {	// FIFO interrupts on INT1 enabled
		Reg = lis2hh12FIFO_SRC;
		iRV2 = halI2C_Queue(sLIS2HH12.psI2C, i2cWRC, &Reg, sizeof(Reg), &sLIS2HH12.Reg.FIFO_SRC,
			SO_MEM(lis2hh12_reg_t, FIFO_SRC), (i2cq_p1_t)lis2hh12IntFIFO, (i2cq_p2_t) Arg);
		++count;
	}
	if (sLIS2HH12.Reg.ctrl3.int1_ig1) {
		Reg = lis2hh12IG_SRC1;
		iRV3 = halI2C_Queue(sLIS2HH12.psI2C, i2cWRC, &Reg, sizeof(Reg), &sLIS2HH12.Reg.IG_SRC1,
			SO_MEM(lis2hh12_reg_t, IG_SRC1), (i2cq_p1_t)lis2hh12IntIG1, (i2cq_p2_t) Arg);
		++count;
	}
	if (sLIS2HH12.Reg.ctrl3.int1_ig2) {
		Reg = lis2hh12IG_SRC2;
		iRV4 = halI2C_Queue(sLIS2HH12.psI2C, i2cWRC, &Reg, sizeof(Reg), &sLIS2HH12.Reg.IG_SRC2,
			SO_MEM(lis2hh12_reg_t, IG_SRC2), (i2cq_p1_t)lis2hh12IntIG2, (i2cq_p2_t) Arg);
		++count;
	}
	if (count) {
		lis2hh12IRQok += count;
	} else {
		++lis2hh12IRQlost;
	}
	if (iRV1 == pdTRUE || iRV2 == pdTRUE || iRV3 == pdTRUE || iRV4 == pdTRUE) {
		portYIELD_FROM_ISR(); 
	}
}

// ################### Identification, Diagnostics & Configuration functions #######################

void test(int i) {
	lis2hh12ReadRegs(lis2hh12FIFO_CTRL, &sLIS2HH12.Reg.FIFO_CTRL, 1);
	PX("i=%d\tFIFO_CTRL=x%02X\r\n", i, sLIS2HH12.Reg.FIFO_CTRL);
}

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
	int iRV = lis2hh12WriteReg(lis2hh12CTRL6, NULL, 0x80);	// REBOOT
	vTaskDelay(30);
//	int iRV = lis2hh12WriteReg(lis2hh12CTRL5, NULL, 0x40);	// SOFT RESET
	if (iRV < erSUCCESS)
		return iRV;
	iRV = lis2hh12ReadRegs(lis2hh12WHO_AM_I, &U8, sizeof(U8));
	if (iRV < erSUCCESS)
		return iRV;
	if (U8 != lis2hh12WHOAMI_NUM)
		return erINV_WHOAMI;
	psI2C->IDok = 1;
	psI2C->Test = 0;
	return iRV;
}

int	lis2hh12Config(i2c_di_t * psI2C) {
	if (psI2C->IDok == 0) return erINV_STATE;
	psI2C->CFGok = 0;

	int iRV;
	iRV = lis2hh12WriteReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, 0x1F);		// hr=0 odr=10 bdu=1 en=7
	if (iRV < erSUCCESS) goto exit;

	iRV = lis2hh12WriteReg(lis2hh12CTRL2, &sLIS2HH12.Reg.CTRL2, 0x00);		// HPF default
	if (iRV < erSUCCESS) goto exit;

	iRV = lis2hh12WriteReg(lis2hh12CTRL3, &sLIS2HH12.Reg.CTRL3, 0x00);		// INT1 default
//	iRV = lis2hh12WriteReg(lis2hh12CTRL3, &sLIS2HH12.Reg.CTRL3, 0x01);		// INT1 DRDY
	if (iRV < erSUCCESS) goto exit;

	iRV = lis2hh12WriteReg(lis2hh12CTRL4, &sLIS2HH12.Reg.CTRL4, 0x04);		// IF_ADD_INCR default
	if (iRV < erSUCCESS) goto exit;

	iRV = lis2hh12WriteReg(lis2hh12CTRL5, &sLIS2HH12.Reg.CTRL5, 0x03);		// Open drain & active low
	if (iRV < erSUCCESS) goto exit;

	iRV = lis2hh12WriteReg(lis2hh12FIFO_CTRL, &sLIS2HH12.Reg.FIFO_CTRL, 0x00);	// default
	if (iRV < erSUCCESS) goto exit;

	iRV = lis2hh12WriteReg(lis2hh12IG_CFG1, &sLIS2HH12.Reg.IG_CFG1, 0x00);	// default
	if (iRV < erSUCCESS) goto exit;

	psI2C->CFGok = 1;
	if (!psI2C->CFGerr) {
		IF_SYSTIMER_INIT(debugTIMING, stLIS2HH12, stMICROS, "LIS2HH12", 500, 1500);
		const gpio_config_t irq_pin_cfg = {
			.pin_bit_mask = 1ULL<<GPIO_NUM_36,
			.mode = GPIO_MODE_INPUT,
			.pull_up_en = GPIO_PULLUP_ENABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_NEGEDGE,
		};
		ESP_ERROR_CHECK(gpio_config(&irq_pin_cfg));
		halGPIO_IRQconfig(lis2hh12IRQ_PIN, lis2hh12IntHandler, NULL);
	}
exit:
	IF_SL_ERROR(iRV < erSUCCESS, iRV);
	return iRV;
}

int	lis2hh12Diags(i2c_di_t * psI2C) { return erSUCCESS; }

// ######################################### Reporting #############################################

int lis2hh12ReportCfg(report_t * psR) {
	int iRV = wprintfx(psR, "\tTEMP: (x%04X) Val=%d\r\n", sLIS2HH12.Reg.i16TEMP, sLIS2HH12.Reg.i16TEMP);
	i32_t Val = (sLIS2HH12.Reg.ACT_THS * fs_scale[sLIS2HH12.Reg.ctrl4.fs]) / 128;
	iRV += wprintfx(psR, "\tACT_THS: (x%02X) %dmg \r\n", sLIS2HH12.Reg.ACT_THS, Val);
	if (sLIS2HH12.Reg.ctrl1.odr)
		Val = (sLIS2HH12.Reg.ACT_DUR * 8) / odr_scale[sLIS2HH12.Reg.ctrl1.odr];
	else
		Val = 0;
	iRV +=  wprintfx(psR, "\tACT_DUR: (x%02X) %ds \r\n", sLIS2HH12.Reg.ACT_DUR, Val);

	iRV +=  wprintfx(psR, "\tCTRL1: (x%02X)  hr=%d  odr=%d/%dHz  bdu=%d  Zen=%d  Yen=%d  Xen=%d\r\n",
		sLIS2HH12.Reg.CTRL1, sLIS2HH12.Reg.ctrl1.hr, sLIS2HH12.Reg.ctrl1.odr, odr_scale[sLIS2HH12.Reg.ctrl1.odr],
		sLIS2HH12.Reg.ctrl1.bdu, sLIS2HH12.Reg.ctrl1.zen, sLIS2HH12.Reg.ctrl1.yen, sLIS2HH12.Reg.ctrl1.xen);

	iRV +=  wprintfx(psR, "\tCTRL2: (x%02X)  dfc=%d  hpm=%d  fds=%d  hpis1=%d  hpis2=%d\r\n",
		sLIS2HH12.Reg.CTRL2, sLIS2HH12.Reg.ctrl2.dfc, sLIS2HH12.Reg.ctrl2.hpm,
		sLIS2HH12.Reg.ctrl2.fds, sLIS2HH12.Reg.ctrl2.hpis1, sLIS2HH12.Reg.ctrl2.hpis2);

	iRV +=  wprintfx(psR, "\tCTRL3: (x%02X)  fifo_en=%d  stop_fth=%d  I1inact=%d  I1ig2=%d  I1ig1=%d  I1ovr=%d  I1fth=%d  I1drdy=%d\r\n",
		sLIS2HH12.Reg.CTRL3, sLIS2HH12.Reg.ctrl3.fifo_en, sLIS2HH12.Reg.ctrl3.stop_fth, sLIS2HH12.Reg.ctrl3.int1_inact,
		sLIS2HH12.Reg.ctrl3.int1_ig2, sLIS2HH12.Reg.ctrl3.int1_ig1, sLIS2HH12.Reg.ctrl3.int1_ovr, sLIS2HH12.Reg.ctrl3.int1_fth, sLIS2HH12.Reg.ctrl3.int1_drdy);

	iRV +=  wprintfx(psR, "\tCTRL4: (x%02X)  bw=%d  fs=%d/%dG  bws_odr=%d  IAInc=%d  I2Cen=%d  sim=%d\r\n",
		sLIS2HH12.Reg.CTRL4, sLIS2HH12.Reg.ctrl4.bw, sLIS2HH12.Reg.ctrl4.fs, fs_scale[sLIS2HH12.Reg.ctrl4.fs]/1000,
		sLIS2HH12.Reg.ctrl4.bw_scale_odr, sLIS2HH12.Reg.ctrl4.if_add_inc, sLIS2HH12.Reg.ctrl4.i2c_enable, sLIS2HH12.Reg.ctrl4.sim);

	iRV +=  wprintfx(psR, "\tCTRL5: (x%02X)  dbg=%d  rst=%d  dec=%d  st=%d  HLactive=%d  pp_od=%d\r\n",
		sLIS2HH12.Reg.CTRL5, sLIS2HH12.Reg.ctrl5.debug, sLIS2HH12.Reg.ctrl5.soft_reset, sLIS2HH12.Reg.ctrl5.dec,
		sLIS2HH12.Reg.ctrl5.st, sLIS2HH12.Reg.ctrl5.h_lactive, sLIS2HH12.Reg.ctrl5.pp_od);

	iRV +=  wprintfx(psR, "\tCTRL6: (x%02X)  boot=%d  I2boot=%d  I2ig2=%d  I2ig1=%d  I2empty=%d  I2fth=%d  I2drdy=%d\r\n",
		sLIS2HH12.Reg.CTRL6, sLIS2HH12.Reg.ctrl6.boot, sLIS2HH12.Reg.ctrl6.int2_boot, sLIS2HH12.Reg.ctrl6.int2_ig2,
		sLIS2HH12.Reg.ctrl6.int2_ig1, sLIS2HH12.Reg.ctrl6.int2_empty, sLIS2HH12.Reg.ctrl6.int2_fth, sLIS2HH12.Reg.ctrl6.int2_drdy);

	iRV +=  wprintfx(psR, "\tCTRL7: (x%02X)  I2dcrm=%d  I1dcrm=%d  I2lir=%d  I1lir=%d  IG2_4d=%d  IG1_4d=%d\r\n",
		sLIS2HH12.Reg.CTRL7, sLIS2HH12.Reg.ctrl7.dcrm2, sLIS2HH12.Reg.ctrl7.dcrm1, sLIS2HH12.Reg.ctrl7.lir2,
		sLIS2HH12.Reg.ctrl7.lir1, sLIS2HH12.Reg.ctrl7._4d_ig2, sLIS2HH12.Reg.ctrl7._4d_ig1);

	iRV +=  wprintfx(psR, "\tSTATUS: (x%02X)  ZYXor=%d  Zor=%d  Yor=%d  Xor=%d  ZYXda=%d  Zda=%d  Yda=%d  Xda=%d\r\n",
		sLIS2HH12.Reg.STATUS, sLIS2HH12.Reg.status.ZYXor, sLIS2HH12.Reg.status.Zor, sLIS2HH12.Reg.status.Yor, sLIS2HH12.Reg.status.Xor,
		sLIS2HH12.Reg.status.ZYXda, sLIS2HH12.Reg.status.Zda, sLIS2HH12.Reg.status.Yda, sLIS2HH12.Reg.status.Xda);

	iRV +=  wprintfx(psR, "\tOUT_X=%hd  OUT_Y=%hd  OUT_Z=%hd\r\n",
		sLIS2HH12.Reg.i16OUT_X, sLIS2HH12.Reg.i16OUT_Y, sLIS2HH12.Reg.i16OUT_Z);

//	iRV +=  wprintfx(psR, "\t: (x%02X)  =%d  =%d  =%d  =%d  =%d  =%d  =%d  =%d\r\n",
//		sLIS2HH12.Reg., sLIS2HH12.Reg.., sLIS2HH12.Reg.., sLIS2HH12.Reg.., sLIS2HH12.Reg..,
//		sLIS2HH12.Reg.., sLIS2HH12.Reg.., sLIS2HH12.Reg.., sLIS2HH12.Reg..);
	return iRV;
}

int lis2hh12ReportFIFO(report_t * psR) {
	static char * fifoMode[] = { "Bypass", "FIFO", "Stream", "StoF", "BtoS", "5=inv", "6=inv", "BtoF" };
	int iRV = wprintfx(psR, "\tFIFO_CTRL: (x%02X)  mode=%d/%s  thres=%d\r\n", sLIS2HH12.Reg.FIFO_CTRL,
		sLIS2HH12.Reg.fifo_ctrl.fmode, fifoMode[sLIS2HH12.Reg.fifo_ctrl.fmode], sLIS2HH12.Reg.fifo_ctrl.fth);
	iRV += wprintfx(psR, "\tFIFO_SRC: (x%02X)  fth=%d  ovr=%d  empty=%d  fss=%d\r\n",
		sLIS2HH12.Reg.FIFO_SRC, sLIS2HH12.Reg.fifo_src.fth, sLIS2HH12.Reg.fifo_src.ovr,
		sLIS2HH12.Reg.fifo_src.empty, sLIS2HH12.Reg.fifo_src.fss);
	return iRV;
}

int lis2hh12ReportIG1(report_t * psR) {
	int iRV = wprintfx(psR, "\tIG_CFG1: (x%02X)  aoi=%d  d6=%d  Zh=%d  Zl=%d  Yh=%d  Yl=%d  Xh=%d  Xl=%d\r\n",
		sLIS2HH12.Reg.IG_CFG1, sLIS2HH12.Reg.ig_cfg1.aoi, sLIS2HH12.Reg.ig_cfg1.d6, sLIS2HH12.Reg.ig_cfg1.zh, sLIS2HH12.Reg.ig_cfg1.zl,
		sLIS2HH12.Reg.ig_cfg1.yh, sLIS2HH12.Reg.ig_cfg1.yl, sLIS2HH12.Reg.ig_cfg1.xh, sLIS2HH12.Reg.ig_cfg1.xl);
	iRV += wprintfx(psR, "\tIG_SRC1: (x%02X)  ia=%d  Zh=%d  Zl=%d  Yh=%d  Yl=%d  Xh=%d  Xl=%d\r\n",
		sLIS2HH12.Reg.IG_SRC1, sLIS2HH12.Reg.ig_src1.ia, sLIS2HH12.Reg.ig_src1.zh, sLIS2HH12.Reg.ig_src1.zl,
		sLIS2HH12.Reg.ig_src1.yh, sLIS2HH12.Reg.ig_src1.yl, sLIS2HH12.Reg.ig_src1.xh, sLIS2HH12.Reg.ig_src1.xl);
	iRV +=  wprintfx(psR, "\tIG1: THS X=%d  Y=%d  Z=%d  DUR=%d\r\n",
		sLIS2HH12.Reg.IG_THS_X1, sLIS2HH12.Reg.IG_THS_Y1, sLIS2HH12.Reg.IG_THS_Z1, sLIS2HH12.Reg.IG_DUR1);
	return iRV;
}

int lis2hh12ReportIG2(report_t * psR) {
	int iRV = wprintfx(psR, "\tIG_CFG2: (x%02X)  aoi=%d  d6=%d  Zh=%d  Zl=%d  Yh=%d  Yl=%d  Xh=%d  Xl=%d\r\n",
		sLIS2HH12.Reg.IG_CFG2, sLIS2HH12.Reg.ig_cfg2.aoi, sLIS2HH12.Reg.ig_cfg2.d6, sLIS2HH12.Reg.ig_cfg2.zh, sLIS2HH12.Reg.ig_cfg2.zl,
		sLIS2HH12.Reg.ig_cfg2.yh, sLIS2HH12.Reg.ig_cfg2.yl, sLIS2HH12.Reg.ig_cfg2.xh, sLIS2HH12.Reg.ig_cfg2.xl);
	iRV += wprintfx(psR, "\tIG_SRC2: (x%02X)  ia=%d  Zh=%d  Zl=%d  Yh=%d  Yl=%d  Xh=%d  Xl=%d\r\n",
		sLIS2HH12.Reg.IG_SRC2, sLIS2HH12.Reg.ig_src2.ia, sLIS2HH12.Reg.ig_src2.zh, sLIS2HH12.Reg.ig_src2.zl,
		sLIS2HH12.Reg.ig_src2.yh, sLIS2HH12.Reg.ig_src2.yl, sLIS2HH12.Reg.ig_src2.xh, sLIS2HH12.Reg.ig_src2.xl);
	iRV +=  wprintfx(psR, "\tIG2: THS=%d  DUR=%d\r\n",
		sLIS2HH12.Reg.IG_THS2, sLIS2HH12.Reg.IG_DUR2);
	return iRV;
}

int lis2hh12ReportAll(report_t * psR) {
	int iRV = halI2C_DeviceReport(psR, sLIS2HH12.psI2C);
	iRV += lis2hh12ReportCfg(psR);
	iRV += lis2hh12ReportFIFO(psR);
	iRV += lis2hh12ReportIG1(psR);
	iRV += lis2hh12ReportIG2(psR);
	iRV +=  wprintfx(psR, "\tREF_X=%d  REF_Y=%d  REF_Z=%d\r\n",
		sLIS2HH12.Reg.u16REF_X, sLIS2HH12.Reg.u16REF_Y, sLIS2HH12.Reg.u16REF_Z);
	iRV += wprintfx(psR, "\tIRQs OK=%lu  Lost=%lu  DRDY=%lu  DRDYerr=%lu  FIFO=%lu  IG1=%lu  IG2=%lu\r\n",
		lis2hh12IRQok, lis2hh12IRQlost, lis2hh12IRQdrdy, lis2hh12IRQdrdyErr, lis2hh12IRQfifo, lis2hh12IRQig1, lis2hh12IRQig2);
	return iRV;
}
#endif
