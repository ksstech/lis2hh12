/*
 * Copyright 2022 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#include	"lis2hh12.h"
#include	<string.h>

#include	"hal_variables.h"
#include	"endpoints.h"
#include	"options.h"
#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"
#include	"x_errors_events.h"

#define	debugFLAG					0xF001

#define	debugCONFIG					(debugFLAG & 0x0001)
#define	debugCONVERT				(debugFLAG & 0x0002)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

/* ##################################### Developer notes ###########################################
	Add auto ranging support if sum of raw values close to 0 or above 100,000
	Scale gain factor up or down...
*/

// ############################################# Macros ############################################

#define	lis2hh12I2C_LOGIC			0					// 0 = delay, 1= stretch, 2= stages

// #################################### SI7006/13/20/21 Addresses ##################################

#define	lis2hh12ADDR0				0x1E
#define	LIS2HH12_T_SNS				1000

// ################################ Forward function declaration ###################################


// ######################################### Constants #############################################


// ###################################### Local variables ##########################################

lis2hh12_t sLIS2HH12 = { 0 };

// #################################### Local ONLY functions #######################################

int lis2hh12ReadReg(uint8_t Reg, uint8_t * pRxBuf) {
	return halI2C_Queue(sLIS2HH12.psI2C, i2cWR_B, &Reg, sizeof(Reg),
			pRxBuf, sizeof(uint8_t), (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

int lis2hh12WriteReg(uint8_t reg, uint8_t val) {
	uint8_t u8Buf[2] = { reg, val };
	return halI2C_Queue(sLIS2HH12.psI2C, i2cW_B, u8Buf, sizeof(u8Buf), NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

#if (lis2hh12I2C_LOGIC == 1)		// read and convert in 1 go...
int	lis2hh12ReadHdlr(epw_t * psEWP) {
	IF_SYSTIMER_START(debugTIMING, stLIS2HH12);
	for (uint8_t Reg = 0; Reg < 4; ++Reg) {
		lis2hh12ReadReg(Reg+lis2hh12DATA_CH1_0, (uint8_t *) &sLIS2HH12.Reg.ch[Reg]);
	}
	IF_SYSTIMER_STOP(debugTIMING, stLIS2HH12);
	IF_PRINT(debugCONVERT, "lis2hh12  [ %-'B ]\n", 4, sLIS2HH12.Reg.ch);
	x64_t X64;
	///
	vCV_SetValue(&table_work[URI_LIS2HH12_X].var, X64);
	return erSUCCESS;
}
#elif (lis2hh12I2C_LOGIC == 2)		// clock stretching
	#error "Clock stretching not supported"
#elif (lis2hh12I2C_LOGIC == 3)		// 3 step read -> wait -> convert
	#error "3 Step read -> wait -> convert not supported"
#endif

// ################################ Rules configuration support ####################################

int	lis2hh12ConfigMode (struct rule_t * psR, int Xcur, int Xmax) {
	// mode /lis2hh12 idx gain time rate
	uint8_t	AI = psR->ActIdx;
	int gain = psR->para.x32[AI][0].i32;
	int time = psR->para.x32[AI][1].i32;
	int rate = psR->para.x32[AI][2].i32;
	IF_PRINT(debugCONFIG && ioB1GET(ioMode), "mode 'LIS2HH12' Xcur=%d Xmax=%d gain=%d time=%d rate=%d\n", Xcur, Xmax, gain, time, rate);

	if (OUTSIDE(0, gain, 7, int) || OUTSIDE(0, time, 7, int) || OUTSIDE(0, rate, 7, int) || gain==4 || gain==5)
		ERR_RETURN("Invalid gain / time / rate specified", erSCRIPT_INV_PARA);
	int iRV = erSUCCESS;
	return iRV;
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

	uint8_t U8;
	int iRV = lis2hh12ReadReg(lis2hh12WHO_AM_I, &U8);
	if (iRV != erSUCCESS)
		goto exit;
	if (U8 != lis2hh12WHOAMI_NUM)
		goto exit_err;
	psI2C_DI->Type		= i2cDEV_LIS2HH12;
	psI2C_DI->Speed		= i2cSPEED_400;
	psI2C_DI->DevIdx 	= 0;
	goto exit;
exit_err:
	iRV = erFAILURE;
exit:
	psI2C_DI->Test = 0;
	return iRV ;
}

int	lis2hh12Config(i2c_di_t * psI2C_DI) {
	epw_t * psEWP = &table_work[URI_LIS2HH12_X];
	psEWP->var.def.cv.vc = 1;
	psEWP->var.def.cv.vs = vs32B;
	psEWP->var.def.cv.vf = vfFXX;
	psEWP->var.def.cv.vt = vtVALUE;
	psEWP->Tsns = psEWP->Rsns = LIS2HH12_T_SNS;
	psEWP->uri = URI_LIS2HH12_X;

	psEWP = &table_work[URI_LIS2HH12_Y];
	psEWP->var.def.cv.vc = 1;
	psEWP->var.def.cv.vs = vs32B;
	psEWP->var.def.cv.vf = vfFXX;
	psEWP->var.def.cv.vt = vtVALUE;
	psEWP->Tsns = psEWP->Rsns = LIS2HH12_T_SNS;
	psEWP->uri = URI_LIS2HH12_Y;

	psEWP = &table_work[URI_LIS2HH12_Z];
	psEWP->var.def.cv.vc = 1;
	psEWP->var.def.cv.vs = vs32B;
	psEWP->var.def.cv.vf = vfFXX;
	psEWP->var.def.cv.vt = vtVALUE;
	psEWP->Tsns = psEWP->Rsns = LIS2HH12_T_SNS;
	psEWP->uri = URI_LIS2HH12_Z;

	psEWP = &table_work[URI_LIS2HH12_T];
	psEWP->var.def.cv.vc = 1;
	psEWP->var.def.cv.vs = vs32B;
	psEWP->var.def.cv.vf = vfFXX;
	psEWP->var.def.cv.vt = vtVALUE;
	psEWP->Tsns = psEWP->Rsns = LIS2HH12_T_SNS;
	psEWP->uri = URI_LIS2HH12_T;

#if (lis2hh12I2C_LOGIC == 3)
	sLIS2HH12.timer = xTimerCreate("lis2hh12", pdMS_TO_TICKS(5), pdFALSE, NULL, lis2hh12TimerHdlr);
#endif
	IF_SYSTIMER_INIT(debugTIMING, stLIS2HH12, stMILLIS, "LIS2HH12", 1, 20);
	return erSUCCESS ;
}

int lis2hh12ReConfig(i2c_di_t * psI2C_DI) { return erSUCCESS; }

int	lis2hh12Diags(i2c_di_t * psI2C_DI) { return erSUCCESS; }

// ######################################### Reporting #############################################

void lis2hh12ReportAll(void) {
	halI2C_DeviceReport(sLIS2HH12.psI2C);
}
