/*
 * lis2hh12.h - Copyright (c) 2022-23 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

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

typedef union __attribute__((packed)) {				// CTRL1
	struct __attribute__((packed)) { u8_t en:3; u8_t bdu:1; u8_t odr:3; u8_t hr:1; };
	struct __attribute__((packed)) { u8_t xen:1; u8_t yen:1; u8_t zen:1; u8_t oth:5; };
} lis2hh12_ctrl1_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl1_t) == 1);

typedef struct __attribute__((packed)) {				// CTRL2
	u8_t hpis2:1;
	u8_t hpis1:1;
	u8_t fds:1;
	u8_t hpm:2;
	u8_t dfc1:2;
	u8_t res:1;
} lis2hh12_ctrl2_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl2_t) == 1);

typedef struct __attribute__((packed)) {				// CTRL3
	u8_t int1_drdy:1;
	u8_t int1_fth:1;
	u8_t int1_ovr:1;
	u8_t int1_ig1:1;
	u8_t int1_ig2:1;
	u8_t int1_inact:1;
	u8_t stop_fth:1;
	u8_t fifo_en:1;
} lis2hh12_ctrl3_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl3_t) == 1);

typedef struct __attribute__((packed)) {				// CTRL4
	u8_t sim:1;
	u8_t i2c_enable:1;
	u8_t if_add_inc:1;
	u8_t bw_scale_odr:1;
	u8_t fs:2;
	u8_t bw:2;
} lis2hh12_ctrl4_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl4_t) == 1);

typedef struct __attribute__((packed)) {				// CTRL5
	u8_t pp_od:1;
	u8_t h_lactive:1;
	u8_t st:2;
	u8_t dec:2;
	u8_t soft_reset:1;
	u8_t debug:1;
} lis2hh12_ctrl5_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl5_t) == 1);

typedef struct __attribute__((packed)) {				// CTRL6
	u8_t int2_drdy:1;
	u8_t int_fth:1;
	u8_t int_empty:1;
	u8_t int2_ig1:1;
	u8_t int2_ig2:1;
	u8_t int2_boot:1;
	u8_t res:1;
	u8_t boot:1;
} lis2hh12_ctrl6_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl6_t) == 1);

typedef struct __attribute__((packed)) {				// CTRL7
	u8_t _4d_ig:2;
	u8_t lir:2;
	u8_t dcrm:2;
	u8_t res:2;
} lis2hh12_ctrl7_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl7_t) == 1);

typedef struct __attribute__((packed)) {				// STATUS
	u8_t xda:1;
	u8_t yda:1;
	u8_t zda:1;
	u8_t zyxda:1;
	u8_t Xor:1;
	u8_t Yor:1;
	u8_t Zor:1;
	u8_t ZYXor:1;
} lis2hh12_status_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_status_t) == 1);

typedef struct __attribute__((packed)) {				// FIFO_CTRL
	u8_t fth:5;
	u8_t fmode: 3;
} lis2hh12_fifo_ctrl_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_fifo_ctrl_t) == 1);

typedef struct __attribute__((packed)) {				// FIFO_SRC
	u8_t fss:5;
	u8_t empty:1;
	u8_t ovr:1;
	u8_t fth:1;
} lis2hh12_fifo_src_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_fifo_src_t) == 1);

typedef union {						// REGS
	struct __attribute__((packed)) {
		union { u8_t TEMPX[2]; u16_t TEMP; };
		u8_t ACT_THS;
		u8_t ACT_DUR;
		union { lis2hh12_ctrl1_t ctrl1; u8_t CTRL1; };				// CTRL1
		union { lis2hh12_ctrl2_t ctrl2; u8_t CTRL2; };				// CTRL2
		union { lis2hh12_ctrl3_t ctrl3; u8_t CTRL3; };				// CTRL3
		union { lis2hh12_ctrl4_t ctrl4; u8_t CTRL4; };				// CTRL4
		union { lis2hh12_ctrl5_t ctrl5; u8_t CTRL5; };				// CTRL5
		union { lis2hh12_ctrl6_t ctrl6; u8_t CTRL6; };				// CTRL6
		union { lis2hh12_ctrl7_t ctrl7; u8_t CTRL7; };				// CTRL7
		union { lis2hh12_status_t status; u8_t STATUS; };			// STATUS
		union { u16_t u16OUT_X; u8_t u8OUT_X[2]; };					// OUT_X
		union { u16_t u16OUT_Y; u8_t u8OUT_Y[2]; };					// OUT_Y
		union { u16_t u16OUT_Z; u8_t u8OUT_Z[2]; };					// OUT_Z
		union { lis2hh12_fifo_ctrl_t fifo_ctrl; u8_t FIFO_CTRL; };	// FIFO_CTRL
		union { lis2hh12_fifo_src_t fifo_src; u8_t FIFO_SRC; };		// FIFO_SRC
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
		union { u16_t u16REF_X; u8_t u8REF_X[2]; };					// REF_X
		union { u16_t u16REF_Y; u8_t u8REF_Y[2]; };					// REF_Y
		union { u16_t u16REF_Z; u8_t u8REF_Z[2]; };					// REF_Z
	};
	u8_t Regs[36];
} lis2hh12_reg_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_reg_t) == 36);

struct i2c_di_t;
typedef struct {
	struct i2c_di_t * psI2C;
	SemaphoreHandle_t mux;
	lis2hh12_reg_t Reg;
} lis2hh12_t;
//DUMB_STATIC_ASSERT(sizeof(lis2hh12_t) == 42);

// ###################################### Public variables #########################################

extern const u16_t fs_scale[];
extern const u16_t odr_scale[];
extern lis2hh12_t sLIS2HH12;

// ###################################### Public functions #########################################

int lis2hh12ReadRegs(u8_t Reg, u8_t * pRxBuf, size_t RxSize);
int lis2hh12WriteReg(u8_t reg, u8_t val);
int lis2hh12UpdateReg(u8_t reg, u8_t * pRxBuf, u8_t _and, u8_t _or);

void lis2hh12ReadTrigger(void * Arg);
int lis2hh12EventHandler(void);
struct i2c_di_t;
int	lis2hh12Identify(struct i2c_di_t * psI2C);
int	lis2hh12Config(struct i2c_di_t * psI2C);
int	lis2hh12Diags(struct i2c_di_t * psI2C);

struct report_t;
int lis2hh12ReportIG_SRC(report_t * psR);
int lis2hh12ReportAll(struct report_t * psR);

#ifdef __cplusplus
}
#endif
