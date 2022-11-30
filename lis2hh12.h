/*
 * Copyright 2022 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#include <stdint.h>

#include "endpoints.h"
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

typedef struct __attribute__((packed)) {				// CTRL1
	u8_t xen : 1;
	u8_t yen : 1;
	u8_t zen : 1;
	u8_t bdu : 1;
	u8_t odr : 3;				// 0 = off, 10->800Hz
	u8_t hr : 1;					// 1 = high resolution enabled
} lis2hh12_ctrl1_t;

typedef struct __attribute__((packed)) {				// CTRL2
	u8_t hpis2 : 1;
	u8_t hpis1 : 1;
	u8_t fds : 1;
	u8_t hpm : 2;
	u8_t dfc1 : 2;
	u8_t res : 1;
} lis2hh12_ctrl2_t;

typedef struct __attribute__((packed)) {				// CTRL3
	u8_t int1_drdy : 1;
	u8_t int1_fth : 1;
	u8_t int1_ovr : 1;
	u8_t int1_ig1 : 1;
	u8_t int1_ig2 : 1;
	u8_t int1_inact : 1;
	u8_t stop_fth : 1;
	u8_t fifo_en : 1;
} lis2hh12_ctrl3_t;

typedef struct __attribute__((packed)) {				// CTRL4
	u8_t sim : 1;
	u8_t i2c_enable : 1;
	u8_t if_add_inc : 1;
	u8_t bw_scale_odr : 1;
	u8_t fs : 2;
	u8_t bw : 2;
} lis2hh12_ctrl4_t;

typedef struct __attribute__((packed)) {				// CTRL5
	u8_t pp_od : 1;
	u8_t h_lactive : 1;
	u8_t st : 2;
	u8_t dec : 2;
	u8_t soft_reset : 1;
	u8_t debug : 1;
} lis2hh12_ctrl5_t;

typedef struct __attribute__((packed)) {				// CTRL6
	u8_t int2_drdy : 1;
	u8_t int_fth : 1;
	u8_t int_empty : 1;
	u8_t int2_ig1 : 1;
	u8_t int2_ig2 : 1;
	u8_t int2_boot : 1;
	u8_t res : 1;
	u8_t boot : 1;
} lis2hh12_ctrl6_t;

typedef struct __attribute__((packed)) {				// CTRL7
	u8_t _4d_ig : 2;
	u8_t lir : 2;
	u8_t dcrm : 2;
	u8_t res : 2;
} lis2hh12_ctrl7_t;

typedef struct __attribute__((packed)) {				// STATUS
	u8_t xda : 1;
	u8_t yda : 1;
	u8_t zda : 1;
	u8_t zyxda : 1;
	u8_t xor : 1;
	u8_t yor : 1;
	u8_t zor : 1;
	u8_t zyxor: 1;
} lis2hh12_status_t;

typedef struct __attribute__((packed)) {				// FIFO_CTRL
	u8_t fth : 5;
	u8_t fmode: 3;
} lis2hh12_fifo_ctrl_t;

typedef struct __attribute__((packed)) {				// FIFO_SRC
	u8_t fss : 5;
	u8_t empty: 1;
	u8_t ovr: 1;
	u8_t fth: 1;
} lis2hh12_fifo_src_t;

typedef struct __attribute__((packed)) {				// REGS
	u8_t ACT_THS;
	u8_t ACT_DUR;
	union {							// CTRL1
		lis2hh12_ctrl1_t ctrl1;
		u8_t CTRL1;
	};
	union {							// CTRL2
		lis2hh12_ctrl2_t ctrl2;
		u8_t CTRL2;
	};
	union {							// CTRL3
		lis2hh12_ctrl3_t ctrl3;
		u8_t CTRL3;
	};
	union {							// CTRL4
		lis2hh12_ctrl4_t ctrl4;
		u8_t CTRL4;
	};
	union {							// CTRL5
		lis2hh12_ctrl5_t ctrl5;
		u8_t CTRL5;
	};
	union {							// CTRL6
		lis2hh12_ctrl6_t ctrl6;
		u8_t CTRL6;
	};
	union {							// CTRL7
		lis2hh12_ctrl7_t ctrl7;
		u8_t CTRL7;
	};
	union {							// STATUS
		lis2hh12_status_t status;
		u8_t STATUS;
	};
	union {							// OUT_X
		u16_t u16OUT_X;
		u8_t u8OUT_X[2];
	};
	union {							// OUT_Y
		u16_t u16OUT_Y;
		u8_t u8OUT_Y[2];
	};
	union {							// OUT_Z
		u16_t u16OUT_Z;
		u8_t u8OUT_Z[2];
	};
	union {							// FIFO_CTRL
		lis2hh12_fifo_ctrl_t fifo_ctrl;
		u8_t FIFO_CTRL;
	};
	union {							// FIFO_SRC
		lis2hh12_fifo_src_t fifo_src;
		u8_t FIFO_SRC;
	};
	u8_t IG_CFG1;
	u8_t IG_SRC1;
	u8_t IG_THS_X1;
	u8_t IG_THS_Y1;
	u8_t IG_THS_Z1;
	u8_t IG_DUR1;
	u8_t IG_CFG2;
	u8_t IG_SRC2;
	u8_t IG_THS2;
	u8_t IG_DUR2;
	union {							// REF_X
		u16_t u16REF_X;
		u8_t u8REF_X[2];
	};
	union {							// REF_Y
		u16_t u16REF_Y;
		u8_t u8REF_Y[2];
	};
	union {							// REF_Z
		u16_t u16REF_Z;
		u8_t u8REF_Z[2];
	};
} lis2hh12_reg_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_reg_t) == 34);

typedef struct {
	i2c_di_t * psI2C;
	SemaphoreHandle_t mux;
	lis2hh12_reg_t Reg;
} lis2hh12_t;
//DUMB_STATIC_ASSERT(sizeof(lis2hh12_t) == 42);

// ###################################### Public variables #########################################


// ###################################### Public functions #########################################

int lis2hh12EventHandler(void);

int	lis2hh12Identify(i2c_di_t * psI2C_DI);
int	lis2hh12Config(i2c_di_t * psI2C_DI);
int	lis2hh12ReConfig(i2c_di_t * psI2C_DI);
int	lis2hh12Diags(i2c_di_t * psI2C_DI);
void lis2hh12ReportAll(void) ;

struct rule_t ;
int	lis2hh12ConfigMode (struct rule_t *, int Xcur, int Xmax, int EI);

struct epw_t ;
int	lis2hh12ReadHdlrAccel(epw_t * psEWP);

#ifdef __cplusplus
	}
#endif
