// lis2hh12.c - Copyright (c) 2022-24 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"

#if (HAL_LIS2HH12 > 0)
#include "endpoints.h"
#include "hal_i2c_common.h"
#include "hal_memory.h"
#include "hal_options.h"
#include "lis2hh12.h"
#include "report.h"
#include "rules.h"
#include "syslog.h"
#include "systiming.h"
#include "errors_events.h"

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
u32_t lis2hh12IRQok, lis2hh12IRQlost, lis2hh12IRQfifo, lis2hh12IRQig1, lis2hh12IRQig2, lis2hh12IRQinact, lis2hh12IRQboot;
u32_t lis2hh12IRQdrdy, lis2hh12IRQdrdyErr;

// #################################### Local ONLY functions #######################################

int lis2hh12WriteReg(u8_t reg, u8_t * pU8, u8_t val) {
	IF_myASSERT(debugPARAM, INRANGE(lis2hh12TEMP_L, reg, lis2hh12ZH_REF));
	u8_t u8Buf[2] = { reg, val };
	int iRV = halI2C_Queue(sLIS2HH12.psI2C, i2cW_B, u8Buf, sizeof(u8Buf), NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	if (pU8) {
 		IF_myASSERT(debugPARAM, halMemoryRAM(pU8));
 		*pU8 = val;										// Optionally, store data at location...
	}
	return iRV;
}

int lis2hh12ReadRegs(u8_t reg, u8_t * pU8, size_t size) {
	IF_myASSERT(debugPARAM, INRANGE(lis2hh12TEMP_L, reg, lis2hh12ZH_REF) && halMemoryRAM(pU8) && size);
	return halI2C_Queue(sLIS2HH12.psI2C, i2cWR_B, &reg, sizeof(reg), pU8, size, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

/**
 * @brief		perform a Write-Read-Modify-Write transaction, also updates local register value
 * @param[in]	reg - register to be addressed
 * @param[in]	pU8 - pointer to u8_t buffer location to be updated
 * @param[in]	_and - mask to AND value read with (Step 1)
 * @param[in]	_or - mask to OR value read with (Step 2) before writing back to device
 * @return		result from halI2C_Queue()
 */
int lis2hh12UpdateReg(u8_t reg, u8_t * pU8, u8_t _and, u8_t _or) {
	IF_myASSERT(debugPARAM, INRANGE(lis2hh12TEMP_L, reg, lis2hh12ZH_REF) && halMemoryRAM(pU8));
	return halI2C_Queue(sLIS2HH12.psI2C, i2cWRMW, &reg, sizeof(reg), pU8, 1, (i2cq_p1_t) (u32_t) _and, (i2cq_p2_t) (u32_t) _or);
}

f32_t lis2hh12ConvCoord(i32_t Val) {
	return (float) Val * (sLIS2HH12.Reg.ctrl4.fs == 0 ? 0.000061 : (sLIS2HH12.Reg.ctrl4.fs == 2) ? 0.000122 : 0.000244);
}

// ################################### Configuration support #######################################

int lis2hh12EnableAxis(lis2hh12_axis_t Axis) { return lis2hh12UpdateReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, 0xF8, Axis); }

int lis2hh12SetBDU(bool State) { return lis2hh12UpdateReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, 0xF7, State << 3); }

int lis2hh12SetODR(lis2hh12_odr_t Rate) { return lis2hh12UpdateReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, 0x8F, Rate << 4); }

int lis2hh12SetHR(bool State) { return lis2hh12UpdateReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, 0x7F, State << 7); }

int lis2hh12SetScale(lis2hh12_fs_t Scale) { return lis2hh12UpdateReg(lis2hh12CTRL4, &sLIS2HH12.Reg.CTRL4, 0xCF, Scale << 4); }

int lis2hh12SetDecimation(lis2hh12_deci_t Samples) { return lis2hh12UpdateReg(lis2hh12CTRL5, &sLIS2HH12.Reg.CTRL5, 0xCF, Samples << 4); }

int lis2hh12GetDRDY(void) { return lis2hh12ReadRegs(lis2hh12STATUS, &sLIS2HH12.Reg.STATUS, sizeof(lis2hh12_status_t)); }

// ######################################## Utility APIs ###########################################

int lis2hh12SoftReset(void) { return lis2hh12UpdateReg(lis2hh12CTRL5, &sLIS2HH12.Reg.CTRL5, 0xBF, 1 << 6); }

int lis2hh12SetBoot(void) { return lis2hh12UpdateReg(lis2hh12CTRL6, &sLIS2HH12.Reg.CTRL6, 0x7F, 1 << 7); }

// ################################# Filter configuration support ###################################

int lis2hh12SetFilterIntPath(lis2hh12_intpath_t IntPath) { return lis2hh12UpdateReg(lis2hh12CTRL2, &sLIS2HH12.Reg.CTRL2, 0xFC, IntPath); }

int lis2hh12SetFilterOutPath(lis2hh12_outpath_t OutPath) {	// logic to be checked...
	int iRV = lis2hh12UpdateReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, 0x7F, (OutPath == lis2hh12_outpathLOPASS) ? 0x01 : 0x00);
	if (iRV > erFAILURE)
		iRV = lis2hh12UpdateReg(lis2hh12CTRL2, &sLIS2HH12.Reg.CTRL2, 0xFB, (OutPath == lis2hh12_outpathHIPASS) ? 0x01 : 0x00); 
	return iRV;
}

int lis2hh12SetFilterHiPassBW(lis2hh12_hp_bw_t HiPassBW) { return lis2hh12UpdateReg(lis2hh12CTRL2, &sLIS2HH12.Reg.CTRL2, 0x87, HiPassBW); }

int lis2hh12SetFilterLoPassBW(lis2hh12_lp_bw_t LoPassBW) { return lis2hh12UpdateReg(lis2hh12CTRL2, &sLIS2HH12.Reg.CTRL2, 0x9F, LoPassBW); }

int lis2hh12SetFilterAAliasBW(lis2hh12_aa_bw_t AAliasBW) { return lis2hh12UpdateReg(lis2hh12CTRL4, &sLIS2HH12.Reg.CTRL4, 0x37, AAliasBW); }

int lis2hh12SetFilterReference(i16_t RefVal) {
	i16_t i16Array[3] = { RefVal, RefVal, RefVal };
	return halI2C_Queue(sLIS2HH12.psI2C, i2cW, (u8_t *)&i16Array[0], sizeof(i16Array), NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

int lis2hh12SetInactivity(u8_t ths, u8_t dur) {
	int iRV = lis2hh12WriteReg(lis2hh12ACT_THS, &sLIS2HH12.Reg.ACT_THS, ths);	// #of FSD/128 mG
	if (iRV < erSUCCESS)							return iRV;
	iRV = lis2hh12WriteReg(lis2hh12ACT_DUR, &sLIS2HH12.Reg.ACT_DUR, dur);	// # of (8/ODR) sec
	if (iRV < erSUCCESS)							return iRV;
	u8_t stat = (ths > 0 || dur > 0) ? 0x20 : 0x00;		// INT1_INACT en/disable?
	return lis2hh12UpdateReg(lis2hh12CTRL3, &sLIS2HH12.Reg.CTRL3, 0xDF, stat);
}

int lis2hh12SetBW(e_bw_t bw) { return lis2hh12UpdateReg(lis2hh12CTRL4, &sLIS2HH12.Reg.CTRL4, 0x3F, bw << 6); }

int lis2hh12ConfigFIFO(e_fm_t mode, u8_t thres) {
	int iRV = lis2hh12WriteReg(lis2hh12FIFO_CTRL, &sLIS2HH12.Reg.FIFO_CTRL, (mode << 5) | (thres & 0x1F));
	if (iRV < erSUCCESS)							return iRV;
	u8_t mask, flag;
	if (mode > fmBYPASS && thres > 0) {				// AMM verify logic below, untested!!!
		mask = 0x7F;
		flag = 0x80;
	} else {
		mask = 0x7F;
		flag = 0x80;
	}
	return lis2hh12UpdateReg(lis2hh12CTRL3, &sLIS2HH12.Reg.CTRL3, mask, flag);
}

// #################################### Reporting support ##########################################

int lis2hh12ReportTEMP(report_t * psR) {
	return wprintfx(psR, "\tTEMP: (x%04X) Val=%d" strNL, sLIS2HH12.Reg.i16TEMP, sLIS2HH12.Reg.i16TEMP);
}

int lis2hh12ReportActDur(report_t * psR) {
	i32_t Val = sLIS2HH12.Reg.ctrl1.odr ? (sLIS2HH12.Reg.ACT_DUR * 8) / odr_scale[sLIS2HH12.Reg.ctrl1.odr] : 0;
	return wprintfx(psR, "\tACT_DUR: (x%02X) %ds " strNL, sLIS2HH12.Reg.ACT_DUR, Val);
}

int lis2hh12ReportActThr(report_t * psR) {
	i32_t Val = (sLIS2HH12.Reg.ACT_THS * fs_scale[sLIS2HH12.Reg.ctrl4.fs]) / 128;
	return wprintfx(psR, "\tACT_THS: (x%02X) %dmg " strNL, sLIS2HH12.Reg.ACT_THS, Val);
}

int li2hh12ReportCTRL1(report_t * psR) {
	return wprintfx(psR, "\tCTRL1: (x%02X)  hr=%d  odr=%d/%dHz  bdu=%d  Zen=%d  Yen=%d  Xen=%d" strNL,
		sLIS2HH12.Reg.CTRL1, sLIS2HH12.Reg.ctrl1.hr, sLIS2HH12.Reg.ctrl1.odr, odr_scale[sLIS2HH12.Reg.ctrl1.odr],
		sLIS2HH12.Reg.ctrl1.bdu, sLIS2HH12.Reg.ctrl1.zen, sLIS2HH12.Reg.ctrl1.yen, sLIS2HH12.Reg.ctrl1.xen);
}

int li2hh12ReportCTRL2(report_t * psR) {
	return wprintfx(psR, "\tCTRL2: (x%02X)  dfc=%d  hpm=%d  fds=%d  hpis1=%d  hpis2=%d" strNL,
		sLIS2HH12.Reg.CTRL2, sLIS2HH12.Reg.ctrl2.dfc, sLIS2HH12.Reg.ctrl2.hpm,
		sLIS2HH12.Reg.ctrl2.fds, sLIS2HH12.Reg.ctrl2.hpis1, sLIS2HH12.Reg.ctrl2.hpis2);
}

int li2hh12ReportCTRL3(report_t * psR) {
	return wprintfx(psR, "\tCTRL3: (x%02X)  fifo_en=%d  stop_fth=%d  I1inact=%d  I1ig2=%d  I1ig1=%d  I1ovr=%d  I1fth=%d  I1drdy=%d" strNL,
		sLIS2HH12.Reg.CTRL3, sLIS2HH12.Reg.ctrl3.fifo_en, sLIS2HH12.Reg.ctrl3.stop_fth, sLIS2HH12.Reg.ctrl3.int1_inact,
		sLIS2HH12.Reg.ctrl3.int1_ig2, sLIS2HH12.Reg.ctrl3.int1_ig1, sLIS2HH12.Reg.ctrl3.int1_ovr, sLIS2HH12.Reg.ctrl3.int1_fth, sLIS2HH12.Reg.ctrl3.int1_drdy);
}

int li2hh12ReportCTRL4(report_t * psR) {
	return wprintfx(psR, "\tCTRL4: (x%02X)  bw=%d  fs=%d/%dG  bws_odr=%d  IAInc=%d  I2cdis=%d  sim=%d" strNL,
		sLIS2HH12.Reg.CTRL4, sLIS2HH12.Reg.ctrl4.bw, sLIS2HH12.Reg.ctrl4.fs, fs_scale[sLIS2HH12.Reg.ctrl4.fs]/1000,
		sLIS2HH12.Reg.ctrl4.bw_scale_odr, sLIS2HH12.Reg.ctrl4.if_add_inc, sLIS2HH12.Reg.ctrl4.i2c_disable, sLIS2HH12.Reg.ctrl4.sim);
}

int li2hh12ReportCTRL5(report_t * psR) {
	return wprintfx(psR, "\tCTRL5: (x%02X)  dbg=%d  rst=%d  dec=%d  st=%d  HLactive=%d  pp_od=%d" strNL,
		sLIS2HH12.Reg.CTRL5, sLIS2HH12.Reg.ctrl5.debug, sLIS2HH12.Reg.ctrl5.soft_reset, sLIS2HH12.Reg.ctrl5.dec,
		sLIS2HH12.Reg.ctrl5.st, sLIS2HH12.Reg.ctrl5.h_lactive, sLIS2HH12.Reg.ctrl5.pp_od);
}

int li2hh12ReportCTRL6(report_t * psR) {
	return wprintfx(psR, "\tCTRL6: (x%02X)  boot=%d  I2boot=%d  I2ig2=%d  I2ig1=%d  I2empty=%d  I2fth=%d  I2drdy=%d" strNL,
		sLIS2HH12.Reg.CTRL6, sLIS2HH12.Reg.ctrl6.boot, sLIS2HH12.Reg.ctrl6.int2_boot, sLIS2HH12.Reg.ctrl6.int2_ig2,
		sLIS2HH12.Reg.ctrl6.int2_ig1, sLIS2HH12.Reg.ctrl6.int2_empty, sLIS2HH12.Reg.ctrl6.int2_fth, sLIS2HH12.Reg.ctrl6.int2_drdy);
}

int li2hh12ReportCTRL7(report_t * psR) {
	return wprintfx(psR, "\tCTRL7: (x%02X)  I2dcrm=%d  I1dcrm=%d  I2lir=%d  I1lir=%d  IG2_4d=%d  IG1_4d=%d" strNL,
		sLIS2HH12.Reg.CTRL7, sLIS2HH12.Reg.ctrl7.dcrm2, sLIS2HH12.Reg.ctrl7.dcrm1, sLIS2HH12.Reg.ctrl7.lir2,
		sLIS2HH12.Reg.ctrl7.lir1, sLIS2HH12.Reg.ctrl7._4d_ig2, sLIS2HH12.Reg.ctrl7._4d_ig1);
}

int li2hh12ReportSTATUS(report_t * psR) {
	return wprintfx(psR, "\tSTATUS: (x%02X)  ZYXor=%d  Zor=%d  Yor=%d  Xor=%d  ZYXda=%d  Zda=%d  Yda=%d  Xda=%d" strNL,
		sLIS2HH12.Reg.STATUS, sLIS2HH12.Reg.status.ZYXor, sLIS2HH12.Reg.status.Zor, sLIS2HH12.Reg.status.Yor, sLIS2HH12.Reg.status.Xor,
		sLIS2HH12.Reg.status.ZYXda, sLIS2HH12.Reg.status.Zda, sLIS2HH12.Reg.status.Yda, sLIS2HH12.Reg.status.Xda);
}

int lis2hh12ReportOUTxyz(report_t * psR) {
	return wprintfx(psR, "\tOUT_X=%hd  OUT_Y=%hd  OUT_Z=%hd" strNL, sLIS2HH12.Reg.i16OUT_X, sLIS2HH12.Reg.i16OUT_Y, sLIS2HH12.Reg.i16OUT_Z);
}

int lis2hh12ReportFIFO_CTRL(report_t * psR) {
	static char * fifoMode[] = { "Bypass", "FIFO", "Stream", "StoF", "BtoS", "5=inv", "6=inv", "BtoF" };
	return wprintfx(psR, "\tFIFO_CTRL: (x%02X)  mode=%s(%hhu)  thres=%hhu" strNL, sLIS2HH12.Reg.FIFO_CTRL,
		fifoMode[sLIS2HH12.Reg.fifo_ctrl.fmode], sLIS2HH12.Reg.fifo_ctrl.fmode, sLIS2HH12.Reg.fifo_ctrl.fth);
}

int lis2hh12ReportFIFO_SRC(report_t * psR) {
	return wprintfx(psR, "\tFIFO_SRC: (x%02X)  fth=%d  ovr=%d  empty=%d  fss=%d" strNL, sLIS2HH12.Reg.FIFO_SRC,
		sLIS2HH12.Reg.fifo_src.fth, sLIS2HH12.Reg.fifo_src.ovr, sLIS2HH12.Reg.fifo_src.empty, sLIS2HH12.Reg.fifo_src.fss);
}

int lis2hh12ReportIGx(report_t * psR, bool X) {
	lis2hh12_ig_cfg_t CfgX = X ? sLIS2HH12.Reg.ig_cfg2 : sLIS2HH12.Reg.ig_cfg1;
	lis2hh12_ig_src_t SrcX = X ? sLIS2HH12.Reg.ig_src2 : sLIS2HH12.Reg.ig_src1;
	lis2hh12_ig_dur_t DurX = X ? sLIS2HH12.Reg.ig_dur2 : sLIS2HH12.Reg.ig_dur1;
	int iRV = wprintfx(psR, "\tIG_CFG%d: (x%02X)  aoi=%d  d6=%d  Zh=%d  Zl=%d  Yh=%d  Yl=%d  Xh=%d  Xl=%d" strNL,
		X, CfgX, CfgX.aoi, CfgX.d6, CfgX.zh, CfgX.zl, CfgX.yh, CfgX.yl, CfgX.xh, CfgX.xl);
	iRV += wprintfx(psR, "\tIG_SRC%d: (x%02X)  ia=%d  Zh=%d  Zl=%d  Yh=%d  Yl=%d  Xh=%d  Xl=%d" strNL,
		X, SrcX, SrcX.ia, SrcX.zh, SrcX.zl, SrcX.yh, SrcX.yl, SrcX.xh, SrcX.xl);
	iRV += wprintfx(psR, "\tIG%d: (x%02X)  wait=%d  dur=%d  ths", X, DurX, DurX.wait, DurX.ths);
	if (X) iRV += wprintfx(psR, "=%d" strNL, sLIS2HH12.Reg.IG_THS2);
	else iRV += wprintfx(psR, ": X=%d  Y=%d  Z=%d" strNL, sLIS2HH12.Reg.IG_THS_X1, sLIS2HH12.Reg.IG_THS_Y1, sLIS2HH12.Reg.IG_THS_Z1);
	return iRV;
}

int lis2hh12ReportREFxyz(report_t * psR) {
	return wprintfx(psR, "\tREF_X=%d  REF_Y=%d  REF_Z=%d" strNL, sLIS2HH12.Reg.u16REF_X, sLIS2HH12.Reg.u16REF_Y, sLIS2HH12.Reg.u16REF_Z);
}

int lis2hh12ReportCounters(report_t * psR) {
	return wprintfx(psR, "\tIRQs OK=%lu  Lost=%lu  DRDY=%lu  DRDYerr=%lu  FIFO=%lu  IG1=%lu  IG2=%lu  INACT=%lu  BOOT=%lu" strNL, lis2hh12IRQok,
		lis2hh12IRQlost, lis2hh12IRQdrdy, lis2hh12IRQdrdyErr, lis2hh12IRQfifo, lis2hh12IRQig1, lis2hh12IRQig2, lis2hh12IRQinact, lis2hh12IRQboot);
}

// #################################### Interrupt support ##########################################

/**
 *	@brief	DRDY IRQ handling
 */
void lis2hh12IntDRDY(void * Arg) {
	lis2hh12_t * psDev = (lis2hh12_t *) Arg;
	PX("(x%02X)  X=%hd  Y=%hd  Z=%hd" strNL, psDev->Reg.STATUS, psDev->Reg.i16OUT_X, psDev->Reg.i16OUT_Y, psDev->Reg.i16OUT_Z);
	if (psDev->Reg.STATUS & 0x0F) {										// Data Available?
		++lis2hh12IRQdrdy;
	} else {
		++lis2hh12IRQdrdyErr;
	}
}

/**
 *	@brief	FIFO IRQ handling
 */
void lis2hh12IntFIFO(void * Arg) {
	lis2hh12_t * psDev = (lis2hh12_t *) Arg;
	lis2hh12ReportFIFO_SRC(NULL);
	#define SO_FIFO_BLK (SO_MEM(lis2hh12_reg_t, i16OUT) + sizeof(lis2hh12_fifo_ctrl_t) + sizeof(lis2hh12_fifo_src_t))
	volatile u8_t Count = psDev->Reg.fifo_src.fss;
	while (Count) {
		lis2hh12ReadRegs(lis2hh12OUT_X_L, &psDev->Reg.u8OUT_X[0], SO_FIFO_BLK);
		Count = psDev->Reg.fifo_src.fss;
		PX("#%d: X=%hd  Y=%hd  Z=%hd" strNL, Count, psDev->Reg.i16OUT_X, psDev->Reg.i16OUT_Y, psDev->Reg.i16OUT_Z);
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
 * @brief		Stage 1 CTRLx/INTx decoder handler (not running in ISR level)
 * @param[in]	pointer to device config/status structure
 * 
 */
void IRAM_ATTR lis2hh12IRQ_1(void * Arg) {
	u8_t Reg = 0;
	lis2hh12_t * psDev = (lis2hh12_t *) Arg;
	if (psDev->Reg.ctrl3.int1_drdy || psDev->Reg.ctrl6.int2_drdy) {			// DRDY on INTx enabled?
		Reg = lis2hh12STATUS;
		halI2C_Queue(psDev->psI2C, i2cWRC, &Reg, sizeof(Reg), &psDev->Reg.STATUS, SO_MEM(lis2hh12_reg_t, STATUS)+SO_MEM(lis2hh12_reg_t, i16OUT), (i2cq_p1_t) lis2hh12IntDRDY, (i2cq_p2_t) Arg);
	}
	if ((psDev->Reg.ctrl3.int1_fth && psDev->Reg.ctrl6.int2_fth) ||			// FIFO threshold on INTx?
		(psDev->Reg.ctrl3.int1_ovr || psDev->Reg.ctrl6.int2_empty)) {		// FIFO overflow on INT1 or empty on INT2?
		Reg = lis2hh12FIFO_SRC;
		halI2C_Queue(psDev->psI2C, i2cWRC, &Reg, sizeof(Reg), &psDev->Reg.FIFO_SRC, SO_MEM(lis2hh12_reg_t, FIFO_SRC), (i2cq_p1_t)lis2hh12IntFIFO, (i2cq_p2_t) Arg);
	}
	if (psDev->Reg.ctrl3.int1_ig1 || psDev->Reg.ctrl6.int2_ig1) {
		Reg = lis2hh12IG_SRC1;
		halI2C_Queue(psDev->psI2C, i2cWRC, &Reg, sizeof(Reg), &psDev->Reg.IG_SRC1, SO_MEM(lis2hh12_reg_t, IG_SRC1), (i2cq_p1_t)lis2hh12IntIG1, (i2cq_p2_t) Arg);
	}
	if (psDev->Reg.ctrl3.int1_ig2 || psDev->Reg.ctrl6.int2_ig2) {
		Reg = lis2hh12IG_SRC2;
		halI2C_Queue(psDev->psI2C, i2cWRC, &Reg, sizeof(Reg), &psDev->Reg.IG_SRC2, SO_MEM(lis2hh12_reg_t, IG_SRC2), (i2cq_p1_t)lis2hh12IntIG2, (i2cq_p2_t) Arg);
	}
	if (psDev->Reg.ctrl3.int1_inact) {										// INACT on INT1 enabled
		Reg = 1;															// only used for counter below....
		++lis2hh12IRQinact;
	}
	if (psDev->Reg.ctrl6.int2_boot) {										// BOOT on INT2 enabled
		Reg = 1;															// only used for counter below....
		++lis2hh12IRQboot;
	}
	if (Reg) ++lis2hh12IRQok; else ++lis2hh12IRQlost;
}

/**
 * @brief		Stage 0 INTx CTRLx IRQ handler
 * @param[in]	pointer to device config/status structure
 * @note		
 */
void IRAM_ATTR lis2hh12IRQ_0(void * Arg) {
	#define pcf8574REQ_TASKS	(taskI2C_MASK)
	EventBits_t xEBrun = xEventGroupGetBitsFromISR(TaskRunState);
	if ((xEBrun & pcf8574REQ_TASKS) != pcf8574REQ_TASKS) {
		++lis2hh12IRQlost;
		return;
	}
	lis2hh12_t * psDev = (lis2hh12_t *) Arg;
	u8_t Reg = lis2hh12CTRL3;
	int iRV1 = halI2C_Queue(psDev->psI2C, i2cWR, &Reg, sizeof(Reg), &psDev->Reg.CTRL3, SO_MEM(lis2hh12_reg_t, CTRL3), (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	Reg = lis2hh12CTRL6;
	int iRV2 = halI2C_Queue(psDev->psI2C, i2cWRC, &Reg, sizeof(Reg), &psDev->Reg.CTRL6, SO_MEM(lis2hh12_reg_t, CTRL6), (i2cq_p1_t)lis2hh12IRQ_1, (i2cq_p2_t) Arg);
	if (iRV1 == pdTRUE || iRV2 == pdTRUE) portYIELD_FROM_ISR();
}

// ################### Identification, Diagnostics & Configuration functions #######################

void test(int i) {
	lis2hh12ReadRegs(lis2hh12FIFO_CTRL, &sLIS2HH12.Reg.FIFO_CTRL, 1);
	PX("i=%d\tFIFO_CTRL=x%02X" strNL, i, sLIS2HH12.Reg.FIFO_CTRL);
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
	if (iRV < erSUCCESS)							return iRV;
	vTaskDelay(pdMS_TO_TICKS(30));
//	int iRV = lis2hh12WriteReg(lis2hh12CTRL5, NULL, 0x40);	// SOFT RESET
	iRV = lis2hh12ReadRegs(lis2hh12WHO_AM_I, &U8, sizeof(U8));
	if (iRV < erSUCCESS)							return iRV;
	if (U8 != lis2hh12WHOAMI_NUM)					return erINV_WHOAMI;
	psI2C->IDok = 1;
	psI2C->Test = 0;
	return iRV;
}

int	lis2hh12Config(i2c_di_t * psI2C) {
	if (psI2C->IDok == 0) return erINV_STATE;
	psI2C->CFGok = 0;
#if (0)
	// x80=HR	x70=ODR		x08=BDU		x04=Zen		x02=Yen		x01=Xen
	int iRV = lis2hh12WriteReg(lis2hh12CTRL1, &sLIS2HH12.Reg.CTRL1, makeCTRL1(0,1,1,1,1,1));
	if (iRV < erSUCCESS) goto exit;

	//	x80=RSVD	x60=DFC		x18=HPM		x04=FDS		x02=HPIS1	x01=HPIS2
	iRV = lis2hh12WriteReg(lis2hh12CTRL2, &sLIS2HH12.Reg.CTRL2, makeCTRL2(0,0,0,0,0));
	if (iRV < erSUCCESS) goto exit;

	//	0x80=Fen	x40=STOP	x20=INACT	x10=IG2		x08=IG1		x04=OVR		x02=FTH		x01=DRDY
	iRV = lis2hh12WriteReg(lis2hh12CTRL3, &sLIS2HH12.Reg.CTRL3, makeCTRL3(1,0,1,1,1,1,1,1));
	if (iRV < erSUCCESS) goto exit;

	//	xC0=BW		x30=FS		x08=BWman	x04=INCR	x02=I2Cdis	x01=SIM
	iRV = lis2hh12WriteReg(lis2hh12CTRL4, &sLIS2HH12.Reg.CTRL4, makeCTRL4(0,0,0,1,0,0));
	if (iRV < erSUCCESS) goto exit;

	//	x80=DGB		x40=RST		x30=DEC		x0C=TST		x02=HLact	x01=ODen
	iRV = lis2hh12WriteReg(lis2hh12CTRL5, &sLIS2HH12.Reg.CTRL5, makeCTRL5(0,0,0,0,1,1));
	if (iRV < erSUCCESS) goto exit;

	//	x80=BOOT	x40=RSVD	x20=I1Boot	x10=I2IG2	x08=I2IG1	x04=I2EMPTY	x02=I2FTH	x01=I2DRDY
	iRV = lis2hh12WriteReg(lis2hh12CTRL6, &sLIS2HH12.Reg.CTRL6, makeCTRL6(0,1,1,1,1,1,1));
	if (iRV < erSUCCESS) goto exit;

	//	xC0=RSVD	x20=I2DCRM	x10=I1DCRM	x08=I2LIR	x04=I1LIR	x02=I2_4D	x01=I1_4D
	iRV = lis2hh12WriteReg(lis2hh12CTRL7, &sLIS2HH12.Reg.CTRL7, makeCTRL7(0,0,0,0,0,0));
	if (iRV < erSUCCESS) goto exit;

	//	xE0=FMode	x1F=FTH
	iRV = lis2hh12WriteReg(lis2hh12FIFO_CTRL, &sLIS2HH12.Reg.FIFO_CTRL, makeFIFOC(fmSTREAM,8));
	if (iRV < erSUCCESS) goto exit;

	//	x80=AOI		x40=D6		x20=ZH		x10=ZL		x08=YH		x04=YL		x02=XH		x01=XL
	iRV = lis2hh12WriteReg(lis2hh12IG_CFG1, &sLIS2HH12.Reg.IG_CFG1, makeIGxCFG(0,0,1,1,1,1,1,1));
	if (iRV < erSUCCESS) goto exit;
	iRV = lis2hh12WriteReg(lis2hh12IG_CFG2, &sLIS2HH12.Reg.IG_CFG2, makeIGxCFG(0,0,0,0,0,0,0,0));
	if (iRV < erSUCCESS) goto exit;
#elif (1)
	//	xC0=BW		x30=FS		x08=BWman	x04=INCR	x02=I2Cdis	x01=SIM
	int iRV = lis2hh12WriteReg(lis2hh12CTRL4, &sLIS2HH12.Reg.CTRL4, makeCTRL4(0,0,0,1,0,0));
	if (iRV < erSUCCESS) goto exit;

#endif
	psI2C->CFGok = 1;
	if (psI2C->CFGerr == 0) {
		const gpio_config_t irq_pin_cfg = {
			.pin_bit_mask = (1ULL << lis2hh12IRQ_PIN), .mode = GPIO_MODE_INPUT,
			.pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_LOW_LEVEL,
		};
		ESP_ERROR_CHECK(gpio_config(&irq_pin_cfg));
		halGPIO_IRQconfig(lis2hh12IRQ_PIN, lis2hh12IRQ_0, &sLIS2HH12);
	}
exit:
	if (iRV < erSUCCESS) SL_ERROR(iRV);
	return iRV;
}

int	lis2hh12Diags(i2c_di_t * psI2C) { return erSUCCESS; }

// ######################################### Reporting #############################################

int lis2hh12ReportAll(report_t * psR) {
	int iRV = halI2C_DeviceReport(psR, sLIS2HH12.psI2C);
	iRV += lis2hh12ReportTEMP(psR);

	iRV += lis2hh12ReportActDur(psR);
	iRV += lis2hh12ReportActThr(psR);

	iRV += li2hh12ReportCTRL1(psR);
	iRV += li2hh12ReportCTRL2(psR);
	iRV += li2hh12ReportCTRL3(psR);
	iRV += li2hh12ReportCTRL4(psR);
	iRV += li2hh12ReportCTRL5(psR);
	iRV += li2hh12ReportCTRL6(psR);
	iRV += li2hh12ReportCTRL7(psR);

	iRV += li2hh12ReportSTATUS(psR);

	iRV += lis2hh12ReportOUTxyz(psR);
	iRV += lis2hh12ReportREFxyz(psR);

	iRV += lis2hh12ReportFIFO_CTRL(psR);
	iRV += lis2hh12ReportFIFO_SRC(psR);

	iRV += lis2hh12ReportIGx(psR, 0);
	iRV += lis2hh12ReportIGx(psR, 1);
	iRV += lis2hh12ReportCounters(psR);
	return iRV;
}
#endif
