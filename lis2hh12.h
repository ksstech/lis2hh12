// lis2hh12.h - Copyright (c) 2022-24 Andre M. Maree/KSS Technologies (Pty) Ltd.

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

typedef enum { odr0, odr10, odr50, odr100, odr200, odr400, odr800 } e_odr_t;

typedef enum { axisNONE, axisX, axisY, axisXY, axisZ, axisXZ, axisYZ, axisXYZ } e_axis_t;

typedef enum { fs2G, fs4G = 2, fs8G } e_fs_t;

typedef enum { bw400, bw200, bw100, bm50 } e_bw_t;

typedef enum { fmBYPASS, fmFIFO, fmSTREAM, fmS2F, fmB2S, fmB2F = 7 } e_fm_t;

// ######################################### Structures ############################################

typedef union {											// CTRL1 ~ general config
	struct __attribute__((packed)) { u8_t en:3; u8_t bdu:1; u8_t odr:3; u8_t hr:1; };
	struct __attribute__((packed)) { u8_t xen:1; u8_t yen:1; u8_t zen:1; u8_t oth:5; };
} lis2hh12_ctrl1_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl1_t) == 1);

typedef struct __attribute__((packed)) {				// CTRL2 ~ HPF
	u8_t hpis2:1;					// HPF on INT2
	u8_t hpis1:1;					// HPF on INT1
	u8_t fds:1;						// 0=bypass, 1=enable HPF data select
	u8_t hpm:2;						// 00=normal, 01=reference HPF signal
	u8_t dfc:2;						// Data Filter Cutoff frequency (ODR related)
	u8_t res:1;
} lis2hh12_ctrl2_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl2_t) == 1);

typedef struct __attribute__((packed)) {				// CTRL3 ~ INT1
	u8_t int1_drdy:1;				// Data Ready
	u8_t int1_fth:1;				// FIFO Threshold
	u8_t int1_ovr:1;				// FIFO Overrun
	u8_t int1_ig1:1;				// IG1 set on INT1
	u8_t int1_ig2:1;				// IG2 set on INT1
	u8_t int1_inact:1;				// fed through to INT1 only
	u8_t stop_fth:1;				// FIFO Stop Threshold to INT1
	u8_t fifo_en:1;					// FIFO enable
} lis2hh12_ctrl3_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl3_t) == 1);

typedef struct __attribute__((packed)) {				// CTRL4 ~ general config
	u8_t sim:1;						// 0=3, 1=4 wire SPI
	u8_t i2c_enable:1;				// 0=enable, 1=disable I2C
	u8_t if_add_inc:1;				// 1=enable
	u8_t bw_scale_odr:1;			// 0=BW auto, 1=use bw[6:7]
	u8_t fs:2;
	u8_t bw:2;
} lis2hh12_ctrl4_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl4_t) == 1);

typedef struct __attribute__((packed)) {				// CTRL5 ~ general config
	u8_t pp_od:1;					// 0=push-pull, 1=open-drain INTx
	u8_t h_lactive:1;				// 0=high, 1=low active
	u8_t st:2;						// 0=normal, 1=pos, 2=neg sign test
	u8_t dec:2;						// 0=none, 1=2, 2=4, 3=8 samples
	u8_t soft_reset:1;
	u8_t debug:1;
} lis2hh12_ctrl5_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl5_t) == 1);

typedef struct __attribute__((packed)) {				// CTRL6 ~ INT2
	u8_t int2_drdy:1;				// Data Ready
	u8_t int2_fth:1;				// FIFO Threshold
	u8_t int2_empty:1;				// FIFO Empty
	u8_t int2_ig1:1;
	u8_t int2_ig2:1;
	u8_t int2_boot:1;				// fed through to INT2 only
	u8_t res:1;
	u8_t boot:1;					// fed through to INT2 only
} lis2hh12_ctrl6_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl6_t) == 1);

typedef struct __attribute__((packed)) {				// CTRL7 ~ INTx config
	u8_t _4d_ig1:1;					// Latched INT1, cleared by reading IG_SRC1
	u8_t _4d_ig2:1;					// Latched INT2, cleared by reading IG_SRC2
	u8_t lir1:1;					// 1=Latched INT1, cleared by reading IG_SRC1
	u8_t lir2:1;					// 1=Latched INT2, cleared by reading IG_SRC2
	u8_t dcrm1:1;					// Decrement Counter Reset Mode INT1
	u8_t dcrm2:1;					// Decrement Counter Reset Mode INT2
	u8_t res:2;
} lis2hh12_ctrl7_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ctrl7_t) == 1);

typedef struct __attribute__((packed)) {				// STATUS
	u8_t Xda:1;
	u8_t Yda:1;
	u8_t Zda:1;
	u8_t ZYXda:1;					// fed to both INT1/2 with CTRL3/6[bit0]
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
	u8_t empty:1;					// fed through to INT2 only
	u8_t ovr:1;						// fed through to INT1 only
	u8_t fth:1;						// fed through to both INT1/2
} lis2hh12_fifo_src_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_fifo_src_t) == 1);

typedef struct __attribute__((packed)) {				// IG_CFGx
	u8_t xl:1;
	u8_t xh:1;
	u8_t yl:1;
	u8_t yh:1;
	u8_t zl:1;
	u8_t zh:1;
	u8_t d6:1;
	u8_t aoi:1;
} lis2hh12_ig_cfg_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ig_cfg_t) == 1);

typedef struct __attribute__((packed)) {				// IG_SRCx
	u8_t xl:1;
	u8_t xh:1;
	u8_t yl:1;
	u8_t yh:1;
	u8_t zl:1;
	u8_t zh:1;
	u8_t ia:1;						// Both IG1/2 fed through to both INT1/2
	u8_t spare:1;
} lis2hh12_ig_src_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ig_src_t) == 1);

typedef struct __attribute__((packed)) {				// IG_DURx
	u8_t ths:7;
	u8_t wait:1;
} lis2hh12_ig_dur_t;
DUMB_STATIC_ASSERT(sizeof(lis2hh12_ig_dur_t) == 1);

typedef union __attribute__((packed)) {					// REGS
	struct __attribute__((packed)) {
		union { i16_t i16TEMP; u8_t TEMPX[2]; };					// x0B/C
		u8_t ACT_THS;												// x1E
		u8_t ACT_DUR;												// x1F
		union { lis2hh12_ctrl1_t ctrl1; u8_t CTRL1; };				// x20
		union { lis2hh12_ctrl2_t ctrl2; u8_t CTRL2; };				// x21
		union { lis2hh12_ctrl3_t ctrl3; u8_t CTRL3; };				// x22
		union { lis2hh12_ctrl4_t ctrl4; u8_t CTRL4; };				// x23
		union { lis2hh12_ctrl5_t ctrl5; u8_t CTRL5; };				// x24
		union { lis2hh12_ctrl6_t ctrl6; u8_t CTRL6; };				// x25
		union { lis2hh12_ctrl7_t ctrl7; u8_t CTRL7; };				// x26
		union { lis2hh12_status_t status; u8_t STATUS; };			// x27
		union { i16_t i16OUT_X; u8_t u8OUT_X[2]; };					// x28/9
		union { i16_t i16OUT_Y; u8_t u8OUT_Y[2]; };					// x2A/B
		union { i16_t i16OUT_Z; u8_t u8OUT_Z[2]; };					// x2C/D
		union { lis2hh12_fifo_ctrl_t fifo_ctrl; u8_t FIFO_CTRL; };	// x2E
		union { lis2hh12_fifo_src_t fifo_src; u8_t FIFO_SRC; };		// x2F
		union { lis2hh12_ig_cfg_t ig_cfg1; u8_t IG_CFG1; };			// x30
		union { lis2hh12_ig_src_t ig_src1; u8_t IG_SRC1; };			// x31
		u8_t IG_THS_X1;												// x32
		u8_t IG_THS_Y1;												// x33
		u8_t IG_THS_Z1;												// x34
		union { lis2hh12_ig_dur_t ig_dur1; u8_t IG_DUR1; };			// x35
		union { lis2hh12_ig_cfg_t ig_cfg2; u8_t IG_CFG2; };			// x36
		union { lis2hh12_ig_src_t ig_src2; u8_t IG_SRC2; };			// x37
		u8_t IG_THS2;												// x38
		union { lis2hh12_ig_dur_t ig_dur2; u8_t IG_DUR2; };			// x39
		union { u16_t u16REF_X; u8_t u8REF_X[2]; };					// x3A/B
		union { u16_t u16REF_Y; u8_t u8REF_Y[2]; };					// x3C/D
		union { u16_t u16REF_Z; u8_t u8REF_Z[2]; };					// x3E/F
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
DUMB_STATIC_ASSERT(sizeof(lis2hh12_t) == 44);

// ###################################### Public variables #########################################

extern const u16_t odr_scale[];
extern lis2hh12_t sLIS2HH12;

// ###################################### Public functions #########################################

int lis2hh12ReadRegs(u8_t Reg, u8_t * pU8, size_t RxSize);
int lis2hh12WriteReg(u8_t Reg, u8_t * pU8, u8_t val);

f32_t lis2hh12ConvCoord(i32_t Val);

struct i2c_di_t;
int	lis2hh12Identify(struct i2c_di_t * psI2C);
int	lis2hh12Config(struct i2c_di_t * psI2C);
int	lis2hh12Diags(struct i2c_di_t * psI2C);

struct report_t;
int lis2hh12ReportIG_SRC(struct report_t * psR);
int lis2hh12ReportAll(struct report_t * psR);

#ifdef __cplusplus
}
#endif
