/*
 * Copyright 2022 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#include <stdint.h>

#include "endpoint_struct.h"
#include "hal_i2c.h"

#ifdef __cplusplus
	extern "C" {
#endif

// ########################################### Macros ##############################################

#define lis2hh12ADDR				0x1E				// 0x1C -> 0x1F selectable
#define lis2hh12WHOAMI_NUM			0x41

// ######################################## Enumerations ###########################################

enum {
	lis2hh12TEMP_L		= 0x0B,
	lis2hh12TEMP_H		= 0x0C,
	lis2hh12WHO_AM_I	= 0x0F,
	lis2hh12ACT_THS		= 0x1E,
	lis2hh12ACT_DUR,
	lis2hh12CTRL1,
	lis2hh12CTRL2,
	lis2hh12CTRL3,
	lis2hh12CTRL4,
	lis2hh12CTRL5,
	lis2hh12CTRL6,
	lis2hh12CTRL7,
	lis2hh12STATUS,
	lis2hh12OUT_X_L,
	lis2hh12OUT_X_H,
	lis2hh12OUT_Y_L,
	lis2hh12OUT_Y_H,
	lis2hh12OUT_Z_L,
	lis2hh12OUT_Z_H,
	lis2hh12FIFO_CTRL,
	lis2hh12FIFO_SRC,
	lis2hh12IG_CFG1,
	lis2hh12IG_SRC1,
	lis2hh12IG_THS_X1,
	lis2hh12IG_THS_Y1,
	lis2hh12IG_THS_Z1,
	lis2hh12IG_DUR1,
	lis2hh12IG_CFG2,
	lis2hh12IG_SRC2,
	lis2hh12IG_THS2,
	lis2hh12IG_DUR2,
	lis2hh12XL_REF,
	lis2hh12XH_REF,
	lis2hh12YL_REF,
	lis2hh12YH_REF,
	lis2hh12ZL_REF,
	lis2hh12ZH_REF,
};

// ######################################### Structures ############################################

typedef struct __attribute__((packed)) {
	uint8_t mode : 1;				// 0=Standby  1=Active
	uint8_t	reset : 1;
	uint8_t	gain : 3;
	uint8_t	res : 3;
} lis2hh12_control_t;

typedef struct __attribute__((packed)) {
	union {							// TEMP
		uint16_t u16TEMP;
		uint8_t u8TEMP[2];
	};
	uint8_t ACT_THS;
	uint8_t ACT_DUR;
	uint8_t CTRL1;
	uint8_t CTRL2;
	uint8_t CTRL3;
	uint8_t CTRL4;
	uint8_t CTRL5;
	uint8_t CTRL6;
	uint8_t CTRL7;
	uint8_t STATUS;
	union {							// OUT_X
		uint16_t u16OUT_X;
		uint8_t u8OUT_X[2];
	};
	union {							// OUT_Y
		uint16_t u16OUT_Y;
		uint8_t u8OUT_Y[2];
	};
	union {							// OUT_Z
		uint16_t u16OUT_Z;
		uint8_t u8OUT_Z[2];
	};
	uint8_t FIFO_CTRL;
	uint8_t FIFO_SRC;
	uint8_t IG_CFG1;
	uint8_t IG_SRC1;
	uint8_t IG_THS_X1;
	uint8_t IG_THS_Y1;
	uint8_t IG_THS_Z1;
	uint8_t IG_DUR1;
	uint8_t IG_CFG2;
	uint8_t IG_SRC2;
	uint8_t IG_THS2;
	uint8_t IG_DUR2;
	union {							// REF_X
		uint16_t u16REF_X;
		uint8_t u8REF_X[2];
	};
	union {							// REF_Y
		uint16_t u16REF_Y;
		uint8_t u8REF_Y[2];
	};
	union {							// REF_Z
		uint16_t u16REF_Z;
		uint8_t u8REF_Z[2];
	};
} lis2hh12_reg_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_reg_t) == 36);

typedef struct __attribute__((packed)) {				// SI70006/13/14/20/xx TMP & RH sensors
	i2c_di_t *		psI2C;			// 4 bytes
	SemaphoreHandle_t mux;
	TimerHandle_t	timer;
	union {
		lis2hh12_reg_t Reg;
		uint8_t u8Buf[sizeof(lis2hh12_reg_t)];
	};
} lis2hh12_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_t) == 48);

// ###################################### Public variables #########################################


// ###################################### Public functions #########################################

int	lis2hh12Identify(i2c_di_t * psI2C_DI);
int	lis2hh12Config(i2c_di_t * psI2C_DI);
int	lis2hh12ReConfig(i2c_di_t * psI2C_DI);
int	lis2hh12Diags(i2c_di_t * psI2C_DI);
void lis2hh12ReportAll(void) ;

struct rule_t ;
int	lis2hh12ConfigMode (struct rule_t *, int Xcur, int Xmax);

struct epw_t ;
int	lis2hh12ReadHdlr(epw_t * psEWP);

#ifdef __cplusplus
	}
#endif
