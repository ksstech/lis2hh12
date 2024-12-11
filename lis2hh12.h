// lis2hh12.h - Copyright (c) 2022-24 Andre M. Maree/KSS Technologies (Pty) Ltd.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ########################################### Macros ##############################################

#define lis2hh12ADDR				0x1E				// 0x1C -> 0x1F selectable
#define lis2hh12WHOAMI_NUM			0x41

#define	makeCTRL1(HR,ODR,BDU,Zen,Yen,Xen)										\
	(((HR&1)<<7) | ((ODR&7)<<4) | ((BDU&1)<<3) | ((Zen&1)<<2) |	((Yen&1)<<1) | (Xen&1))

#define	makeCTRL2(DFC,HPM,FDS,HPIS1,HPIS2)										\
	(((DFC&3)<<5) | ((HPM&3)<<3) | ((FDS&1)<<2) | ((HPIS1&1)<<1) | (HPIS2&1))

#define	makeCTRL3(Fen,I1STOP,I1INACT,I1IG2,I1IG1,I1OVR,I1FTH,I1DRDY)			\
	(((Fen&1)<<7) | ((I1STOP&1)<<6) | ((I1INACT&1)<<5) | ((I1IG2&1)<<4) | ((I1IG1&1)<<3) | ((I1OVR&1)<<2) | ((I1FTH&1)<<1) | (I1DRDY&1))

#define	makeCTRL4(BW,FS,BWman,INCR,I2Cdis,SIM)									\
	(((BW&3)<<6) | ((FS&3)<<4) | ((BWman&1)<<3) | ((INCR&1)<<2) | ((I2Cdis&1)<<1) | (SIM&1))

#define	makeCTRL5(DBG,RST,DEC,TST,HLact,ODen)									\
	(((DBG&1)<<7) | ((RST&1)<<6) | ((DEC&3)<<4) | ((TST&3)<<2) | ((HLact&1)<<1) | (ODen&1))

#define	makeCTRL6(BOOT,I2BOOT,I2IG2,I2IG1,I2EMPTY,I2FTH,I2DRDY)					\
	(((BOOT&1)<<7) | ((I2BOOT&1)<<5) | ((I2IG2&1)<<4) | ((I2IG1&1)<<3) | ((I2EMPTY&1)<<2) | ((I2FTH&1)<<1) | (I2DRDY&1))

#define	makeCTRL7(I2DCRM,I1DCRM,I2LIR,I1LIR,I2_4D,I1_4D)						\
	(((I2DCRM&1)<<5) | ((I1DCRM&1)<<4) | ((I2LIR&1)<<3) | ((I1LIR&1)<<2) | ((I2_4D&1)<<1) | ((I1_4D&1)))

#define	makeFIFOC(FMode,FTH)													\
	(((FMode&7)<<4) | (FTH&31))

#define	makeIGxCFG(AOI,D6,ZH,ZL,YH,YL,XH,XL)									\
	(((AOI&1)<<7) | ((D6&1)<<6) | ((ZH&1)<<5) | ((ZL&1)<<4) | ((YH&1)<<3) | ((YL&1)<<2) | ((XH&1)<<1) | (XL&1))

#define	makeIGxDUR(WAITx,DURx)													\
	(((WAITx&1)<<7) | (DURx&0x7F))

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

typedef enum {
	lis2hh12_axisNONE, 
	lis2hh12_axisX, 
	lis2hh12_axisY, 
	lis2hh12_axisYX, 
	lis2hh12_axisZ,
	lis2hh12_axisZX, 
	lis2hh12_axisZY,
	lis2hh12_axisZYX,
} lis2hh12_axis_t;

typedef enum { lis2hh12_odr0, lis2hh12_odr10, lis2hh12_odr50, lis2hh12_odr100, lis2hh12_odr200, lis2hh12_odr400, lis2hh12_odr800 } lis2hh12_odr_t;

typedef enum { lis2hh12_fs2G, lis2hh12_fs4G = 2, lis2hh12_fs8G } lis2hh12_fs_t;

typedef enum { lis2hh12_deci0, lis2hh12_deci2, lis2hh12_deci4, lis2hh12_deci8 } lis2hh12_deci_t;

typedef enum { bw400, bw200, bw100, bm50 } e_bw_t;

typedef enum { fmBYPASS, fmFIFO, fmSTREAM, fmS2F, fmB2S, fmB2F = 7 } e_fm_t;

typedef enum { lis2hh12_intpathNONE, lis2hh12_intpathIG1, lis2hh12_intpathIG2, lis2hh12_intpathBOTH } lis2hh12_intpath_t;

typedef enum { lis2hh12_outpathBYPASS, lis2hh12_outpathLOPASS, lis2hh12_outpathHIPASS } lis2hh12_outpath_t;

typedef enum {
	lis2hh12_hp_odr_div50			= 0x00,
	lis2hh12_hp_odr_div100			= 0x20,
	lis2hh12_hp_odr_div9			= 0x40,
	lis2hh12_hp_odr_div400			= 0x60,
	lis2hh12_hp_odr_div50_REF_MD	= 0x01,
	lis2hh12_hp_odr_div100_REF_MD	= 0x21,
	lis2hh12_hp_odr_div9_REF_MD		= 0x41,
	lis2hh12_hp_odr_div400_REF_MD	= 0x61,
} lis2hh12_hp_bw_t;

typedef enum { lis2hh12_lp_odr_div50, lis2hh12_lp_odr_div100, lis2hh12_lp_odr_div9, lis2hh12_lp_odr_div400 } lis2hh12_lp_bw_t;

typedef enum {
  lis2hh12_aa_bwAUTO      = 0x00,
  lis2hh12_aa_bw408Hz     = 0x08,
  lis2hh12_aa_bw211Hz     = 0x48,
  lis2hh12_aa_bw105Hz     = 0x88,
  lis2hh12_aa_bw50Hz      = 0xC8,
} lis2hh12_aa_bw_t;

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
	u8_t i2c_disable:1;				// 0=enable, 1=disable I2C
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

typedef union __attribute__((packed)) lis2hh12_reg_t {	// REGS
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
		union {														// x28-2D
			i16_t i16OUT[3];
			struct {
				union { i16_t i16OUT_X; u8_t u8OUT_X[2]; };			// x28/9
				union { i16_t i16OUT_Y; u8_t u8OUT_Y[2]; };			// x2A/B
				union { i16_t i16OUT_Z; u8_t u8OUT_Z[2]; };			// x2C/D
			};
		};
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
