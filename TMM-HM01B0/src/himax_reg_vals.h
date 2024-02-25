
#if defined(use_hm01b0)
#define HIMAX_BOOT_RETRY            (10)
#define HIMAX_LINE_LEN_PCK_FULL     0x178
#define HIMAX_FRAME_LENGTH_FULL     0x109

#define HIMAX_LINE_LEN_PCK_QVGA     0x178
#define HIMAX_FRAME_LENGTH_QVGA     0x104

#define HIMAX_LINE_LEN_PCK_QQVGA    0x178
#define HIMAX_FRAME_LENGTH_QQVGA    0x084


typedef enum {
	LOAD_DEFAULT_REGS,
	LOAD_WALKING1S_REG,
	LOAD_SHM01B0INIT_REGS,
    LOAD_FULL_REGS,
    LOAD_QVGA_REGS,
    LOAD_QQVGA_REGS,

} camera_reg_settings_t;

const uint16_t default_regs[][2] = {
    {BLC_TGT,              0x08},          //  BLC target :8  at 8 bit mode
    {BLC2_TGT,             0x08},          //  BLI target :8  at 8 bit mode
    {0x3044,               0x0A},          //  Increase CDS time for settling
    {0x3045,               0x00},          //  Make symetric for cds_tg and rst_tg
    {0x3047,               0x0A},          //  Increase CDS time for settling
    {0x3050,               0xC0},          //  Make negative offset up to 4x
    {0x3051,               0x42},
    {0x3052,               0x50},
    {0x3053,               0x00},
    {0x3054,               0x03},          //  tuning sf sig clamping as lowest
    {0x3055,               0xF7},          //  tuning dsun
    {0x3056,               0xF8},          //  increase adc nonoverlap clk
    {0x3057,               0x29},          //  increase adc pwr for missing code
    {0x3058,               0x1F},          //  turn on dsun
    {0x3059,               0x1E},
    {0x3064,               0x00},
    {0x3065,               0x04},          //  pad pull 0
    {ANA_Register_17,      0x00},          //  Disable internal oscillator
   
    {BLC_CFG,              0x43},          //  BLC_on, IIR
   
    {0x1001,               0x43},          //  BLC dithering en
    {0x1002,               0x43},          //  blc_darkpixel_thd
    {0x0350,               0x7F},          //  Dgain Control
    {BLI_EN,               0x01},          //  BLI enable
    {0x1003,               0x00},          //  BLI Target [Def: 0x20]
   
    {DPC_CTRL,             0x01},          //  DPC option 0: DPC off   1 : mono   3 : bayer1   5 : bayer2
    {0x1009,               0xA0},          //  cluster hot pixel th
    {0x100A,               0x60},          //  cluster cold pixel th
    {SINGLE_THR_HOT,       0x90},          //  single hot pixel th
    {SINGLE_THR_COLD,      0x40},          //  single cold pixel th
    {0x1012,               0x00},          //  Sync. shift disable
    {STATISTIC_CTRL,       0x07},          //  AE stat en | MD LROI stat en | magic
    {0x2003,               0x00},
    {0x2004,               0x1C},
    {0x2007,               0x00},
    {0x2008,               0x58},
    {0x200B,               0x00},
    {0x200C,               0x7A},
    {0x200F,               0x00},
    {0x2010,               0xB8},
    {0x2013,               0x00},
    {0x2014,               0x58},
    {0x2017,               0x00},
    {0x2018,               0x9B},
   
    {AE_CTRL,              0x01},          //Automatic Exposure
    {AE_TARGET_MEAN,       0x64},          //AE target mean          [Def: 0x3C]
    {AE_MIN_MEAN,          0x0A},          //AE min target mean      [Def: 0x0A]
    {CONVERGE_IN_TH,       0x03},          //Converge in threshold   [Def: 0x03]
    {CONVERGE_OUT_TH,      0x05},          //Converge out threshold  [Def: 0x05]
    {MAX_INTG_H,           (HIMAX_FRAME_LENGTH_QVGA-2)>>8},          //Maximum INTG High Byte  [Def: 0x01]
    {MAX_INTG_L,           (HIMAX_FRAME_LENGTH_QVGA-2)&0xFF},        //Maximum INTG Low Byte   [Def: 0x54]
    {MAX_AGAIN_FULL,       0x04},          //Maximum Analog gain in full frame mode [Def: 0x03]
    {MAX_AGAIN_BIN2,       0x04},          //Maximum Analog gain in bin2 mode       [Def: 0x04]
    {MAX_DGAIN,            0xC0},
   
    {INTEGRATION_H,        0x01},          //Integration H           [Def: 0x01]
    {INTEGRATION_L,        0x08},          //Integration L           [Def: 0x08]
    {ANALOG_GAIN,          0x00},          //Analog Global Gain      [Def: 0x00]
    {DAMPING_FACTOR,       0x20},          //Damping Factor          [Def: 0x20]
    {DIGITAL_GAIN_H,       0x01},          //Digital Gain High       [Def: 0x01]
    {DIGITAL_GAIN_L,       0x00},          //Digital Gain Low        [Def: 0x00]
   
    {FS_CTRL,              0x00},          //Flicker Control
   
    {FS_60HZ_H,            0x00},
    {FS_60HZ_L,            0x85},
    {FS_50HZ_H,            0x00},
    {FS_50HZ_L,            0xa0},

    {MD_CTRL,              0x00},
    {FRAME_LEN_LINES_H,    HIMAX_FRAME_LENGTH_QVGA>>8},
    {FRAME_LEN_LINES_L,    HIMAX_FRAME_LENGTH_QVGA&0xFF},
    {LINE_LEN_PCK_H,       HIMAX_LINE_LEN_PCK_QVGA>>8},
    {LINE_LEN_PCK_L,       HIMAX_LINE_LEN_PCK_QVGA&0xFF},
    {QVGA_WIN_EN,          0x01},          // Enable QVGA window readout
    {0x0383,               0x01},
    {0x0387,               0x01},
    {0x0390,               0x00},
    {0x3011,               0x70},
    {0x3059,               0x02},
    {OSC_CLK_DIV,          0x0B},
    {IMG_ORIENTATION,      0x00},          // change the orientation
    {0x0104,               0x01},

    //============= End of regs marker ==================
    {0x0000,            0x00},
};

const uint16_t Walking1s_reg[][2] =
{
    {0x2100, 0x00},    //W 24 2100 00 2 1 ; AE
    {0x1000, 0x00},    //W 24 1000 00 2 1 ; BLC
    {0x1008, 0x00},    //W 24 1008 00 2 1 ; DPC
    {0x0205, 0x00},    //W 24 0205 00 2 1 ; AGain
    {0x020E, 0x01},    //W 24 020E 01 2 1 ; DGain
    {0x020F, 0x00},    //W 24 020F 00 2 1 ; DGain
    {0x0601, 0x11},    //W 24 0601 11 2 1 ; Test pattern
    {0x0104, 0x01},    //W 24 0104 01 2 1 ;
    //============= End of regs marker ==================
    {0x0000,            0x00},	
};


const uint16_t FULL_regs[][2] = {
    {0x0383,                0x01},
    {0x0387,                0x01},
    {0x0390,                0x00},
    {QVGA_WIN_EN,           0x00},// Disable QVGA window readout
    {MAX_INTG_H,            (HIMAX_FRAME_LENGTH_FULL-2)>>8},
    {MAX_INTG_L,            (HIMAX_FRAME_LENGTH_FULL-2)&0xFF},
    {FRAME_LEN_LINES_H,     (HIMAX_FRAME_LENGTH_FULL>>8)},
    {FRAME_LEN_LINES_L,     (HIMAX_FRAME_LENGTH_FULL&0xFF)},
    {LINE_LEN_PCK_H,        (HIMAX_LINE_LEN_PCK_FULL>>8)},
    {LINE_LEN_PCK_L,        (HIMAX_LINE_LEN_PCK_FULL&0xFF)},
    {GRP_PARAM_HOLD,        0x01},
    //============= End of regs marker ==================
    {0x0000,            0x00},

};

const uint16_t QVGA_regs[][2] = {
    {0x0383,                0x01},
    {0x0387,                0x01},
    {0x0390,                0x00},
    {QVGA_WIN_EN,           0x01},// Enable QVGA window readout
    {MAX_INTG_H,            (HIMAX_FRAME_LENGTH_QVGA-2)>>8},
    {MAX_INTG_L,            (HIMAX_FRAME_LENGTH_QVGA-2)&0xFF},
    {FRAME_LEN_LINES_H,     (HIMAX_FRAME_LENGTH_QVGA>>8)},
    {FRAME_LEN_LINES_L,     (HIMAX_FRAME_LENGTH_QVGA&0xFF)},
    {LINE_LEN_PCK_H,        (HIMAX_LINE_LEN_PCK_QVGA>>8)},
    {LINE_LEN_PCK_L,        (HIMAX_LINE_LEN_PCK_QVGA&0xFF)},
	{BIT_CONTROL,			0x02},
    {GRP_PARAM_HOLD,        0x01},
    //============= End of regs marker ==================
    {0x0000,            0x00},

};

const uint16_t QQVGA_regs[][2] = {
    {0x0383,                0x03},
    {0x0387,                0x03},
    {0x0390,                0x03},
    {QVGA_WIN_EN,           0x01},// Enable QVGA window readout
    {MAX_INTG_H,            (HIMAX_FRAME_LENGTH_QQVGA-2)>>8},
    {MAX_INTG_L,            (HIMAX_FRAME_LENGTH_QQVGA-2)&0xFF},
    {FRAME_LEN_LINES_H,     (HIMAX_FRAME_LENGTH_QQVGA>>8)},
    {FRAME_LEN_LINES_L,     (HIMAX_FRAME_LENGTH_QQVGA&0xFF)},
    {LINE_LEN_PCK_H,        (HIMAX_LINE_LEN_PCK_QQVGA>>8)},
    {LINE_LEN_PCK_L,        (HIMAX_LINE_LEN_PCK_QQVGA&0xFF)},
    {GRP_PARAM_HOLD,        0x01},
    //============= End of regs marker ==================
    {0x0000,            0x00},
};

const uint16_t sHM01B0Init_regs[][2] = 
{
// ;*************************************************************************
// ; Sensor: HM01B0
// ; I2C ID: 24
// ; Resolution: 324x244
// ; Lens:
// ; Flicker:
// ; Frequency:
// ; Description: AE control enable
// ; 8-bit mode, LSB first
// ;
// ;
// ; Note:
// ;
// ; $Revision: 1338 $
// ; $Date:: 2017-04-11 15:43:45 +0800#$
// ;*************************************************************************
// 
// // ---------------------------------------------------
// // HUB system initial
// // ---------------------------------------------------
// W 20 8A04 01 2 1
// W 20 8A00 22 2 1
// W 20 8A01 00 2 1
// W 20 8A02 01 2 1
// W 20 0035 93 2 1 ; [3]&[1] hub616 20bits in, [5:4]=1 mclk=48/2=24mhz
// W 20 0036 00 2 1
// W 20 0011 09 2 1
// W 20 0012 B6 2 1
// W 20 0014 08 2 1
// W 20 0015 98 2 1
// ;W 20 0130 16 2 1 ; 3m soc, signal buffer control
// ;W 20 0100 44 2 1 ; [6] hub616 20bits in
// W 20 0100 04 2 1 ; [6] hub616 20bits in
// W 20 0121 01 2 1 ; [0] Q1 Intf enable, [1]:4bit mode, [2] msb first, [3] serial mode
// W 20 0150 00 2 1 ;
// W 20 0150 04 2 1 ;
// 
// 
// //---------------------------------------------------
// // Initial
// //---------------------------------------------------
// W 24 0103 00 2 1 ; software reset-> was 0x22
    {0x0103, 0x00},
// W 24 0100 00 2 1; power up
    {0x0100, 0x00},
// 
// 
// 
// //---------------------------------------------------
// // Analog
// //---------------------------------------------------
// L HM01B0_analog_setting.txt
    {0x1003, 0x08},
    {0x1007, 0x08},
    {0x3044, 0x0A},
    {0x3045, 0x00},
    {0x3047, 0x0A},
    {0x3050, 0xC0},
    {0x3051, 0x42},
    {0x3052, 0x50},
    {0x3053, 0x00},
    {0x3054, 0x03},
    {0x3055, 0xF7},
    {0x3056, 0xF8},
    {0x3057, 0x29},
    {0x3058, 0x1F},
    {0x3059, 0x1E},
    {0x3064, 0x00},
    {0x3065, 0x04},
// 
// 
// //---------------------------------------------------
// // Digital function
// //---------------------------------------------------
// 
// // BLC
// W 24 1000 43 2 1 ; BLC_on, IIR
    {0x1000, 0x43},
// W 24 1001 40 2 1 ; [6] : BLC dithering en
    {0x1001, 0x40},
// W 24 1002 32 2 1 ; // blc_darkpixel_thd
    {0x1002, 0x32},
// 
// // Dgain
// W 24 0350 7F 2 1 ; Dgain Control
    {0x0350, 0x7F},
// 
// // BLI
// W 24 1006 01 2 1 ; [0] : bli enable
    {0x1006, 0x01},
// 
// // DPC
// W 24 1008 00 2 1 ; [2:0] : DPC option 0: DPC off 1 : mono 3 : bayer1 5 : bayer2
    {0x1008, 0x00},
// W 24 1009 A0 2 1 ; cluster hot pixel th
    {0x1009, 0xA0},
// W 24 100A 60 2 1 ; cluster cold pixel th
    {0x100A, 0x60},
// W 24 100B 90 2 1 ; single hot pixel th
    {0x100B, 0x90},
// W 24 100C 40 2 1 ; single cold pixel th
    {0x100C, 0x40},
// //
// advance VSYNC by 1 row
    {0x3022, 0x01},
// W 24 1012 00 2 1 ; Sync. enable VSYNC shift
    {0x1012, 0x01},

// 
// // ROI Statistic
// W 24 2000 07 2 1 ; [0] : AE stat en [1] : MD LROI stat en [2] : MD GROI stat en [3] : RGB stat ratio en [4] : IIR selection (1 -> 16, 0 -> 8)
    {0x2000, 0x07},
// W 24 2003 00 2 1 ; MD GROI 0 y start HB
    {0x2003, 0x00},
// W 24 2004 1C 2 1 ; MD GROI 0 y start LB
    {0x2004, 0x1C},
// W 24 2007 00 2 1 ; MD GROI 1 y start HB
    {0x2007, 0x00},
// W 24 2008 58 2 1 ; MD GROI 1 y start LB
    {0x2008, 0x58},
// W 24 200B 00 2 1 ; MD GROI 2 y start HB
    {0x200B, 0x00},
// W 24 200C 7A 2 1 ; MD GROI 2 y start LB
    {0x200C, 0x7A},
// W 24 200F 00 2 1 ; MD GROI 3 y start HB
    {0x200F, 0x00},
// W 24 2010 B8 2 1 ; MD GROI 3 y start LB
    {0x2010, 0xB8},
// 
// W 24 2013 00 2 1 ; MD LRIO y start HB
    {0x2013, 0x00},
// W 24 2014 58 2 1 ; MD LROI y start LB
    {0x2014, 0x58},
// W 24 2017 00 2 1 ; MD LROI y end HB
    {0x2017, 0x00},
// W 24 2018 9B 2 1 ; MD LROI y end LB
    {0x2018, 0x9B},
// 
// // AE
// W 24 2100 01 2 1 ; [0]: AE control enable
    {0x2100, 0x01},
// W 24 2104 07 2 1 ; converge out th
    {0x2104, 0x07},
// W 24 2105 0C 2 1 ; max INTG Hb
    {0x2105, 0x0C},
// W 24 2106 78 2 1 ; max INTG Lb
    {0x2106, 0x78},
// W 24 2108 03 2 1 ; max AGain in full
    {0x2108, 0x03},
// W 24 2109 03 2 1 ; max AGain in bin2
    {0x2109, 0x03},
// W 24 210B 80 2 1 ; max DGain
    {0x210B, 0x80},
// W 24 210F 00 2 1 ; FS 60Hz Hb
    {0x210F, 0x00},
// W 24 2110 85 2 1 ; FS 60Hz Lb
    {0x2110, 0x85},
// W 24 2111 00 2 1 ; Fs 50Hz Hb
    {0x2111, 0x00},
// W 24 2112 A0 2 1 ; FS 50Hz Lb
    {0x2112, 0xA0},
// 
// 
// // MD
// W 24 2150 03 2 1 ; [0] : MD LROI en [1] : MD GROI en
    {0x2150, 0x03},
// 
// 
// //---------------------------------------------------
// // frame rate : 5 FPS
// //---------------------------------------------------
// W 24 0340 0C 2 1 ; smia frame length Hb
    {0x0340, 0x0C},
// W 24 0341 7A 2 1 ; smia frame length Lb 3192
    {0x0341, 0x7A},
// 
// W 24 0342 01 2 1 ; smia line length Hb
    {0x0342, 0x01},
// W 24 0343 77 2 1 ; smia line length Lb 375
    {0x0343, 0x77},
// 
// //---------------------------------------------------
// // Resolution : QVGA 324x244
// //---------------------------------------------------
// W 24 3010 01 2 1 ; [0] : window mode 0 : full frame 324x324 1 : QVGA
    {0x3010, 0x01},
// 
// 
// W 24 0383 01 2 1 ;
    {0x0383, 0x01},
// W 24 0387 01 2 1 ;
    {0x0387, 0x01},
// W 24 0390 00 2 1 ;
    {0x0390, 0x00},
// 
// //---------------------------------------------------
// // bit width Selection
// //---------------------------------------------------
// W 24 3011 70 2 1 ; [0] : 6 bit mode enable
    {0x3011, 0x70},
// 
// 
// W 24 3059 02 2 1 ; [7]: Self OSC En, [6]: 4bit mode, [5]: serial mode, [4:0]: keep value as 0x02
    {0x3059, 0x02},
// W 24 3060 01 2 1 ; [5]: gated_clock, [4]: msb first,
    {0x3060, 0x20},
// ; [3:2]: vt_reg_div -> div by 4/8/1/2
// ; [1;0]: vt_sys_div -> div by 8/4/2/1
// 
// 
    {0x0101, 0x01},
// //---------------------------------------------------
// // CMU update
// //---------------------------------------------------
// 
// W 24 0104 01 2 1 ; was 0100
    {0x0104, 0x01},
// 
// 
// 
// //---------------------------------------------------
// // Turn on rolling shutter
// //---------------------------------------------------
// W 24 0100 01 2 1 ; was 0005 ; mode_select 00 : standby - wait fir I2C SW trigger 01 : streaming 03 : output "N" frame, then enter standby 04 : standby - wait for HW trigger (level), then continuous video out til HW TRIG goes off 06 : standby - wait for HW trigger (edge), then output "N" frames then enter standby
    {0x0100, 0x00},
// 
// ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    //============= End of regs marker ==================
    {0x0000,            0x00},
};

//Arducam configuration
const uint16_t Arducam_hm01b0_324x244[][2]  = {
    {0x0103, 0x0},
    {0x0100,0x00},  
    {0x1003,0x08},
    {0x1007,0x08}, 
    {0x3044,0x0A},     
    {0x3045,0x00},    
    {0x3047,0x0A},    
    {0x3050,0xC0},    
    {0x3051,0x42}, 
    {0x3052,0x50},
    {0x3053,0x00},
    {0x3054,0x03}, 
    {0x3055,0xF7},
    {0x3056,0xF8},
    {0x3057,0x29},
    {0x3058,0x1F},
    {0x3059,0x1E},
    {0x3064,0x00},
    {0x3065,0x04},
    {0x1000,0x43},
    {0x1001,0x40},
    {0x1002,0x32}, 
    {0x0350,0x7F},
    {0x1006,0x01},
    {0x1008,0x00},
    {0x1009,0xA0},
    {0x100A,0x60},
    {0x100B,0x90},
    {0x100C,0x40},
    {0x3022,0x01},
    {0x1012,0x01},
    {0x2000,0x07},
    {0x2003,0x00}, 
    {0x2004,0x1C},
    {0x2007,0x00}, 
    {0x2008,0x58}, 
    {0x200B,0x00}, 
    {0x200C,0x7A}, 
    {0x200F,0x00},
    {0x2010,0xB8},
    {0x2013,0x00},
    {0x2014,0x58},
    {0x2017,0x00},
    {0x2018,0x9B},
    {0x2100,0x01},
    {0x2101,0x5F},
    {0x2102,0x0A},
    {0x2103,0x03},
    {0x2104,0x05},
    {0x2105,0x02},
    {0x2106,0x14},
    {0x2107,0x02},
    {0x2108,0x03},
    {0x2109,0x03},
    {0x210A,0x00},
    {0x210B,0x80},
    {0x210C,0x40},
    {0x210D,0x20},
    {0x210E,0x03},
    {0x210F,0x00},
    {0x2110,0x85},
    {0x2111,0x00},
    {0x2112,0xA0},
    {0x2150,0x03},
    {0x0340,0x01},
    {0x0341,0x7A},
    {0x0342,0x01},
    {0x0343,0x77},
    {0x3010,0x00},  //bit[0] 1 enable QVGA
    {0x0383,0x01},
    {0x0387,0x01},
    {0x0390,0x00},
    {0x3011,0x70},
    {0x3059,0x22},
    {0x3060,0x30},
    {0x0101,0x01}, 
    {0x0104,0x01},
    //{0x0390,0x03},  //1/4 binning
    //{0x0383,0x03},
    //{0x0387,0x03},
    //{0x1012,0x03},
    {0x0100,0x01},
    //============= End of regs marker ==================
    {0x0000,            0x00},
};
#elif defined(use_hm0360)

#define HIMAX_BOOT_RETRY            (10)
#define HIMAX_LINE_LEN_PCK_FULL      0x300
#define HIMAX_FRAME_LENGTH_FULL      0x214

#define HIMAX_LINE_LEN_PCK_VGA      0x300
#define HIMAX_FRAME_LENGTH_VGA      0x214

#define HIMAX_LINE_LEN_PCK_QVGA     0x178
#define HIMAX_FRAME_LENGTH_QVGA     0x109

#define HIMAX_LINE_LEN_PCK_QQVGA    0x178
#define HIMAX_FRAME_LENGTH_QQVGA    0x084
/* looks like ROI is for motion detection */
#define HIMAX_MD_ROI_VGA_W          40
#define HIMAX_MD_ROI_VGA_H          30

#define HIMAX_MD_ROI_QVGA_W         20
#define HIMAX_MD_ROI_QVGA_H         15

#define HIMAX_MD_ROI_QQVGA_W        10
#define HIMAX_MD_ROI_QQVGA_H        8

typedef enum {
	LOAD_DEFAULT_REGS,

} camera_reg_settings_t;


const uint16_t himax_default_regs[][2] = {
    {SW_RESET,          0x00},
    {MONO_MODE,         0x00},
    {MONO_MODE_ISP,     0x01},
    {MONO_MODE_SEL,     0x01},

    // BLC control
    {0x1000,            0x01},
    {0x1003,            0x04},
    {BLC_TGT,           0x04},
    {0x1007,            0x01},
    {0x1008,            0x04},
    {BLC2_TGT,          0x04},
    {MONO_CTRL,         0x01},

    // Output format control
    {OPFM_CTRL,         0x0C},

    // Reserved regs
    {0x101D,            0x00},
    {0x101E,            0x01},
    {0x101F,            0x00},
    {0x1020,            0x01},
    {0x1021,            0x00},

    {CMPRS_CTRL,        0x00},
    {CMPRS_01,          0x09},
    {CMPRS_02,          0x12},
    {CMPRS_03,          0x23},
    {CMPRS_04,          0x31},
    {CMPRS_05,          0x3E},
    {CMPRS_06,          0x4B},
    {CMPRS_07,          0x56},
    {CMPRS_08,          0x5E},
    {CMPRS_09,          0x65},
    {CMPRS_10,          0x72},
    {CMPRS_11,          0x7F},
    {CMPRS_12,          0x8C},
    {CMPRS_13,          0x98},
    {CMPRS_14,          0xB2},
    {CMPRS_15,          0xCC},
    {CMPRS_16,          0xE6},

    {0x3112,            0x00},  // PCLKO_polarity falling

    {OSC_CONFIG,       0x08},  // Core = 24MHz PCLKO = 24MHz I2C = 12MHz
    {PLL2_CONFIG,       0x0A},  // MIPI pre-dev (default)
    {PLL3_CONFIG,       0x77},  // PMU/MIPI pre-dev (default)

    {PMU_CFG_3,         0x08},  // Disable context switching
    {PAD_REGISTER_07,   0x00},  // PCLKO_polarity falling

    {AE_CTRL,           0x5F},  // Automatic Exposure (NOTE: Auto framerate enabled)
    {AE_CTRL1,          0x00},
    {T_DAMPING,         0x20},  // AE T damping factor
    {N_DAMPING,         0x00},  // AE N damping factor
    {AE_TARGET_MEAN,    0x64},  // AE target
    {AE_MIN_MEAN,       0x0A},  // AE min target mean
    {AE_TARGET_ZONE,    0x23},  // AE target zone
    {CONVERGE_IN_TH,    0x03},  // AE converge in threshold
    {CONVERGE_OUT_TH,   0x05},  // AE converge out threshold
    {MAX_INTG_H,        (HIMAX_FRAME_LENGTH_QVGA-4)>>8},
    {MAX_INTG_L,        (HIMAX_FRAME_LENGTH_QVGA-4)&0xFF},

    {MAX_AGAIN,         0x04},  // Maximum analog gain
    {MAX_DGAIN_H,       0x03},
    {MAX_DGAIN_L,       0x3F},
    {INTEGRATION_H,     0x01},
    {INTEGRATION_L,     0x08},

    {MD_CTRL,           0x6A},
    {MD_TH_MIN,         0x01},
    {MD_BLOCK_NUM_TH,   0x01},
    {MD_CTRL1,          0x06},
    {PULSE_MODE,        0x00},  // Interrupt in level mode.
    {ROI_START_END_V,   0xF0},
    {ROI_START_END_H,   0xF0},

    {FRAME_LEN_LINES_H, HIMAX_FRAME_LENGTH_QVGA>>8},
    {FRAME_LEN_LINES_L, HIMAX_FRAME_LENGTH_QVGA&0xFF},
    {LINE_LEN_PCK_H,    HIMAX_LINE_LEN_PCK_QVGA>>8},
    {LINE_LEN_PCK_L,    HIMAX_LINE_LEN_PCK_QVGA&0xFF},
    {H_SUBSAMPLE,       0x01},
    {V_SUBSAMPLE,       0x01},
    {BINNING_MODE,      0x00},
    {WIN_MODE,          0x00},
    {IMG_ORIENTATION,   0x00},
    {COMMAND_UPDATE,    0x01},

    /// SYNC function config.
    {0x3010,            0x00},
    {0x3013,            0x01},
    {0x3019,            0x00},
    {0x301A,            0x00},
    {0x301B,            0x20},
    {0x301C,            0xFF},

    // PREMETER config.
    {0x3026,            0x03},
    {0x3027,            0x81},
    {0x3028,            0x01},
    {0x3029,            0x00},
    {0x302A,            0x30},
    {0x302E,            0x00},
    {0x302F,            0x00},

    // Magic regs 🪄.
    {0x302B,            0x2A},
    {0x302C,            0x00},
    {0x302D,            0x03},
    {0x3031,            0x01},
    {0x3051,            0x00},
    {0x305C,            0x03},
    {0x3060,            0x00},
    {0x3061,            0xFA},
    {0x3062,            0xFF},
    {0x3063,            0xFF},
    {0x3064,            0xFF},
    {0x3065,            0xFF},
    {0x3066,            0xFF},
    {0x3067,            0xFF},
    {0x3068,            0xFF},
    {0x3069,            0xFF},
    {0x306A,            0xFF},
    {0x306B,            0xFF},
    {0x306C,            0xFF},
    {0x306D,            0xFF},
    {0x306E,            0xFF},
    {0x306F,            0xFF},
    {0x3070,            0xFF},
    {0x3071,            0xFF},
    {0x3072,            0xFF},
    {0x3073,            0xFF},
    {0x3074,            0xFF},
    {0x3075,            0xFF},
    {0x3076,            0xFF},
    {0x3077,            0xFF},
    {0x3078,            0xFF},
    {0x3079,            0xFF},
    {0x307A,            0xFF},
    {0x307B,            0xFF},
    {0x307C,            0xFF},
    {0x307D,            0xFF},
    {0x307E,            0xFF},
    {0x307F,            0xFF},
    {0x3080,            0x01},
    {0x3081,            0x01},
    {0x3082,            0x03},
    {0x3083,            0x20},
    {0x3084,            0x00},
    {0x3085,            0x20},
    {0x3086,            0x00},
    {0x3087,            0x20},
    {0x3088,            0x00},
    {0x3089,            0x04},
    {0x3094,            0x02},
    {0x3095,            0x02},
    {0x3096,            0x00},
    {0x3097,            0x02},
    {0x3098,            0x00},
    {0x3099,            0x02},
    {0x309E,            0x05},
    {0x309F,            0x02},
    {0x30A0,            0x02},
    {0x30A1,            0x00},
    {0x30A2,            0x08},
    {0x30A3,            0x00},
    {0x30A4,            0x20},
    {0x30A5,            0x04},
    {0x30A6,            0x02},
    {0x30A7,            0x02},
    {0x30A8,            0x01},
    {0x30A9,            0x00},
    {0x30AA,            0x02},
    {0x30AB,            0x34},
    {0x30B0,            0x03},
    {0x30C4,            0x10},
    {0x30C5,            0x01},
    {0x30C6,            0xBF},
    {0x30C7,            0x00},
    {0x30C8,            0x00},
    {0x30CB,            0xFF},
    {0x30CC,            0xFF},
    {0x30CD,            0x7F},
    {0x30CE,            0x7F},
    {0x30D3,            0x01},
    {0x30D4,            0xFF},
    {0x30D5,            0x00},
    {0x30D6,            0x40},
    {0x30D7,            0x00},
    {0x30D8,            0xA7},
    {0x30D9,            0x05},
    {0x30DA,            0x01},
    {0x30DB,            0x40},
    {0x30DC,            0x00},
    {0x30DD,            0x27},
    {0x30DE,            0x05},
    {0x30DF,            0x07},
    {0x30E0,            0x40},
    {0x30E1,            0x00},
    {0x30E2,            0x27},
    {0x30E3,            0x05},
    {0x30E4,            0x47},
    {0x30E5,            0x30},
    {0x30E6,            0x00},
    {0x30E7,            0x27},
    {0x30E8,            0x05},
    {0x30E9,            0x87},
    {0x30EA,            0x30},
    {0x30EB,            0x00},
    {0x30EC,            0x27},
    {0x30ED,            0x05},
    {0x30EE,            0x00},
    {0x30EF,            0x40},
    {0x30F0,            0x00},
    {0x30F1,            0xA7},
    {0x30F2,            0x05},
    {0x30F3,            0x01},
    {0x30F4,            0x40},
    {0x30F5,            0x00},
    {0x30F6,            0x27},
    {0x30F7,            0x05},
    {0x30F8,            0x07},
    {0x30F9,            0x40},
    {0x30FA,            0x00},
    {0x30FB,            0x27},
    {0x30FC,            0x05},
    {0x30FD,            0x47},
    {0x30FE,            0x30},
    {0x30FF,            0x00},
    {0x3100,            0x27},
    {0x3101,            0x05},
    {0x3102,            0x87},
    {0x3103,            0x30},
    {0x3104,            0x00},
    {0x3105,            0x27},
    {0x3106,            0x05},
    {0x310B,            0x10},
    {0x3113,            0xA0},
    {0x3114,            0x67},
    {0x3115,            0x42},
    {0x3116,            0x10},
    {0x3117,            0x0A},
    {0x3118,            0x3F},
    {0x311C,            0x10},
    {0x311D,            0x06},
    {0x311E,            0x0F},
    {0x311F,            0x0E},
    {0x3120,            0x0D},
    {0x3121,            0x0F},
    {0x3122,            0x00},
    {0x3123,            0x1D},
    {0x3126,            0x03},
    {0x3128,            0x57},
    {0x312A,            0x11},
    {0x312B,            0x41},
    {0x312E,            0x00},
    {0x312F,            0x00},
    {0x3130,            0x0C},
    {0x3141,            0x2A},
    {0x3142,            0x9F},
    {0x3147,            0x18},
    {0x3149,            0x18},
    {0x314B,            0x01},
    {0x3150,            0x50},
    {0x3152,            0x00},
    {0x3156,            0x2C},
    {0x315A,            0x0A},
    {0x315B,            0x2F},
    {0x315C,            0xE0},
    {0x315F,            0x02},
    {0x3160,            0x1F},
    {0x3163,            0x1F},
    {0x3164,            0x7F},
    {0x3165,            0x7F},
    {0x317B,            0x94},
    {0x317C,            0x00},
    {0x317D,            0x02},
    {0x318C,            0x00},

    {0x310F,            0x00},  //puts it in 8bit mode
    {0x3112,0x04},	//was 0x0c

    {COMMAND_UPDATE,    0x01},
    {0x0000,            0x00},
};

const uint16_t himax_vga_regs[][2] = {
    {OSC_CONFIG,           0x08},  // Core = 24MHz PCLKO = 24MHz I2C = 12MHz
    {H_SUBSAMPLE,           0x00},
    {V_SUBSAMPLE,           0x00},
    {BINNING_MODE,          0x00},
    {WIN_MODE,              0x01},
    {MAX_INTG_H,            (HIMAX_FRAME_LENGTH_VGA-4)>>8},
    {MAX_INTG_L,            (HIMAX_FRAME_LENGTH_VGA-4)&0xFF},
    {FRAME_LEN_LINES_H,     (HIMAX_FRAME_LENGTH_VGA>>8)},
    {FRAME_LEN_LINES_L,     (HIMAX_FRAME_LENGTH_VGA&0xFF)},
    {LINE_LEN_PCK_H,        (HIMAX_LINE_LEN_PCK_VGA>>8)},
    {LINE_LEN_PCK_L,        (HIMAX_LINE_LEN_PCK_VGA&0xFF)},
    {ROI_START_END_H,       0xF0},
    {ROI_START_END_V,       0xE0},
    {0x310F,                0x00},  //puts it in 8bit mode
    {0x3112,0x04},	//was 0x0c
    {COMMAND_UPDATE,        0x01},
    {0x0000,                0x00},
};

const uint16_t himax_qvga_regs[][2] = {
    {OSC_CONFIG,           0x09},  // Core = 12MHz PCLKO = 24MHz I2C = 12MHz
    {H_SUBSAMPLE,           0x01},
    {V_SUBSAMPLE,           0x01},
    {BINNING_MODE,          0x00},
    {WIN_MODE,              0x01},
    {MAX_INTG_H,            (HIMAX_FRAME_LENGTH_QVGA-4)>>8},
    {MAX_INTG_L,            (HIMAX_FRAME_LENGTH_QVGA-4)&0xFF},
    {FRAME_LEN_LINES_H,     (HIMAX_FRAME_LENGTH_QVGA>>8)},
    {FRAME_LEN_LINES_L,     (HIMAX_FRAME_LENGTH_QVGA&0xFF)},
    {LINE_LEN_PCK_H,        (HIMAX_LINE_LEN_PCK_QVGA>>8)},
    {LINE_LEN_PCK_L,        (HIMAX_LINE_LEN_PCK_QVGA&0xFF)},
    {ROI_START_END_H,       0xF0},
    {ROI_START_END_V,       0xE0},
    {0x310F,                0x00},  //added to get a good pic
    {0x3112,0x04},	//was 0x0c
    {COMMAND_UPDATE,        0x01},
    {0x0000,                0x00},
};
/*  note to self:  1 bit enable
https://github.com/ArduCAM/Pico4ML_AdapterBoard/blob/main/HM0360/lib/arducam/hm0360_init.h
*/
const uint16_t himax_qvga4bit_regs[][2] = {
    {OSC_CONFIG,           0x09},  // Core = 12MHz PCLKO = 24MHz I2C = 12MHz
    {H_SUBSAMPLE,           0x01},
    {V_SUBSAMPLE,           0x01},
    {BINNING_MODE,          0x00},
    {WIN_MODE,              0x01},
    {MAX_INTG_H,            (HIMAX_FRAME_LENGTH_QVGA-4)>>8},
    {MAX_INTG_L,            (HIMAX_FRAME_LENGTH_QVGA-4)&0xFF},
    {FRAME_LEN_LINES_H,     (HIMAX_FRAME_LENGTH_QVGA>>8)},
    {FRAME_LEN_LINES_L,     (HIMAX_FRAME_LENGTH_QVGA&0xFF)},
    {LINE_LEN_PCK_H,        (HIMAX_LINE_LEN_PCK_QVGA>>8)},
    {LINE_LEN_PCK_L,        (HIMAX_LINE_LEN_PCK_QVGA&0xFF)},
    {ROI_START_END_H,       0xF0},
    {ROI_START_END_V,       0xE0},
    {0x310F,                0x40},
    //{0x1014,0x01},  
    {0x3112,0x04},	//was 0x0c
    {COMMAND_UPDATE,        0x01},
    {0x0000,                0x00},
};

const uint16_t himax_qqvga_regs[][2] = {
    {OSC_CONFIG,           0x09},  // Core = 12MHz PCLKO = 24MHz I2C = 12MHz
    {H_SUBSAMPLE,           0x02},
    {V_SUBSAMPLE,           0x02},
    {BINNING_MODE,          0x00},
    {WIN_MODE,              0x01},
    {MAX_INTG_H,            (HIMAX_FRAME_LENGTH_QQVGA-4)>>8},
    {MAX_INTG_L,            (HIMAX_FRAME_LENGTH_QQVGA-4)&0xFF},
    {FRAME_LEN_LINES_H,     (HIMAX_FRAME_LENGTH_QQVGA>>8)},
    {FRAME_LEN_LINES_L,     (HIMAX_FRAME_LENGTH_QQVGA&0xFF)},
    {LINE_LEN_PCK_H,        (HIMAX_LINE_LEN_PCK_QQVGA>>8)},
    {LINE_LEN_PCK_L,        (HIMAX_LINE_LEN_PCK_QQVGA&0xFF)},
    {ROI_START_END_H,       0xF0},
    {ROI_START_END_V,       0xD0},
    {COMMAND_UPDATE,        0x01},
    {0x0000,                0x00},
};

#endif
