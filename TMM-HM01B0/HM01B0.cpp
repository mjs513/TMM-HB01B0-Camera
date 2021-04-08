/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2021 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2021 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * HM01B0 driver.
 */

#include "HM01B0.h"
#include "HM01B0_regs.h"

#include <Wire.h>

#define HIMAX_BOOT_RETRY            (10)
#define HIMAX_LINE_LEN_PCK_FULL     0x178
#define HIMAX_FRAME_LENGTH_FULL     0x109

#define HIMAX_LINE_LEN_PCK_QVGA     0x178
#define HIMAX_FRAME_LENGTH_QVGA     0x104

#define HIMAX_LINE_LEN_PCK_QQVGA    0x178
#define HIMAX_FRAME_LENGTH_QQVGA    0x084

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
    {FS_60HZ_L,            0x3C},
    {FS_50HZ_H,            0x00},
    {FS_50HZ_L,            0x32},

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

// Constructor
HM01B0::HM01B0()
{
}

int HM01B0::reset()
{
    // Reset sensor.
    uint8_t reg=0xff;
    for (int retry=HIMAX_BOOT_RETRY; reg != HIMAX_MODE_STANDBY; retry--) {
        if (retry == 0) {
            return -1;
        }
        if (cameraWriteRegister(SW_RESET, HIMAX_RESET) != 0) { //cambus_writeb2(&sensor->bus, sensor->slv_addr, SW_RESET, HIMAX_RESET
            return -1;
        }
        // Delay for 1ms.
        delay(1);
        if (cameraWriteRegister(MODE_SELECT, reg) != 0) { //cambus_readb2(&sensor->bus, sensor->slv_addr, MODE_SELECT, &reg
            return -1;
        }
    }

    // Write default regsiters
    int ret = 0;
    for (int i=0; default_regs[i][0] && ret == 0; i++) {
        ret |= cameraWriteRegister(default_regs[i][0], default_regs[i][1]);
    }

    // Set PCLK polarity.
    ret |= cameraWriteRegister(PCLK_POLARITY, (0x20 | PCLK_FALLING_EDGE));
    
    // Set mode to streaming
    ret |= cameraWriteRegister( MODE_SELECT, HIMAX_MODE_STREAMING);

    return ret;
}



// Read a single uint8_t from address and return it as a uint8_t
uint8_t HM01B0::cameraReadRegister(uint16_t reg) {
  Wire.beginTransmission(0x24);
  Wire.write(reg >> 8);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("error reading HM01B0, address");
    return 0;
  }
  if (Wire.requestFrom(0x24, 1) < 1) {
    Serial.println("error reading HM01B0, data");
    return 0;
  }
  return Wire.read();
}


uint8_t HM01B0::cameraWriteRegister(uint16_t reg, uint8_t data) {
  Wire.beginTransmission(0x24);
  Wire.write(reg >> 8);
  Wire.write(reg);
  Wire.write(data);
  if (Wire.endTransmission() != 0) {
    Serial.println("error writing to HM01B0");
  }
  return 0;
}

int HM01B0::set_pixformat( pixformat_t pixformat)
{
    int ret = 0;
    switch (pixformat) {
        case PIXFORMAT_BAYER:
			pixformat = PIXFORMAT_BAYER;
			break;
        case PIXFORMAT_GRAYSCALE:
			pixformat = PIXFORMAT_GRAYSCALE;
            break;
        default:
			pixformat = PIXFORMAT_INVALID;
            return -1;
    }

    return ret;
}

uint8_t HM01B0::set_framesize(framesize_t new_framesize)
{
    int ret=0;
    //uint16_t w = resolution[framesize][0];
    //uint16_t h = resolution[framesize][1];
	framesize = new_framesize;

    switch (framesize) {
        case FRAMESIZE_320X320:
			w = 320; h = 320;  //324, 244
            for (int i=0; FULL_regs[i][0] && ret == 0; i++) {
                ret |=  cameraWriteRegister(FULL_regs[i][0], FULL_regs[i][1]);  //cambus_writeb2(&sensor->bus, sensor->slv_addr, FULL_regs[i][0], FULL_regs[i][1]
            }
            break;
        case FRAMESIZE_QVGA:
			w = 320; h = 240;
            for (int i=0; QVGA_regs[i][0] && ret == 0; i++) {
                ret |= cameraWriteRegister( QVGA_regs[i][0], QVGA_regs[i][1]);
            }
            break;
        case FRAMESIZE_QQVGA:
			w = 160; h = 120;
            for (int i=0; QQVGA_regs[i][0] && ret == 0; i++) {
                ret |= cameraWriteRegister( QQVGA_regs[i][0], QQVGA_regs[i][1]);
            }
            break;
		case FRAMESIZE_INVALID:
            for (int i=0; default_regs[i][0] && ret == 0; i++) {
                ret |= cameraWriteRegister( default_regs[i][0], default_regs[i][1]);
			}
			break;
		default:
            ret = -1;
            
    }

    return ret;
}

int HM01B0::set_framerate(int framerate)
{
    uint8_t osc_div = 0;
    bool    highres = false;
	//framesize_t framesize;

    if (framesize == FRAMESIZE_INVALID
            || framesize == FRAMESIZE_QVGA
            || framesize == FRAMESIZE_320X320) {
        highres = true;
    }

    switch (framerate) {
        case 15:
            osc_div = (highres == true) ? 0x01 : 0x00;
            break;
        case 30:
            osc_div = (highres == true) ? 0x02 : 0x01;
            break;
        case 60:
            osc_div = (highres == true) ? 0x03 : 0x02;
            break;
        case 120:
            // Set to the max possible FPS at this resolution.
            osc_div = 0x03;
            break;
        default:
            return -1;
    }
	Serial.printf("OSC_CLK_DIV: 0x%X\n", 0x08 | osc_div);
    return cameraWriteRegister(OSC_CLK_DIV, 0x08 | osc_div);
}

int HM01B0::set_brightness(int level)
{
    uint8_t ae_mean;
    // Simulate brightness levels by setting AE loop target mean.
    switch (level) {
        case 0:
            ae_mean = 60;
            break;
        case 1:
            ae_mean = 80;
            break;
        case 2:
            ae_mean = 100;
            break;
        case 3:
            ae_mean = 127;
            break;
        default:
            ae_mean = 60;
    }
    return cameraWriteRegister(AE_TARGET_MEAN, ae_mean);
}

int HM01B0::set_gainceiling(gainceiling_t gainceiling)
{
    int ret = 0;
    int gain = 0x0;
    switch (gainceiling) {
        case GAINCEILING_2X:
            gain = 0x01;
            break;
        case GAINCEILING_4X:
            gain = 0x02;
            break;
        case GAINCEILING_8X:
            gain = 0x03;
            break;
        case GAINCEILING_16X:
            gain = 0x04;
            break;
        default:
            return -1;
    }
    ret |= cameraWriteRegister(MAX_AGAIN_FULL, gain);
    ret |= cameraWriteRegister(MAX_AGAIN_BIN2, gain);
    return ret;
}

int HM01B0::set_colorbar(int enable)
{
    return cameraWriteRegister(TEST_PATTERN_MODE, enable & 0x1);
}

int HM01B0::set_auto_gain(int enable, float gain_db, float gain_db_ceiling)
{
    int ret = 0;
    if ((enable == 0) && (!isnanf(gain_db)) && (!isinff(gain_db))) {
        gain_db = max(min(gain_db, 24.0f), 0.0f);
        int gain = fast_ceilf(fast_log2(fast_expf((gain_db / 20.0f) * fast_log(10.0f))));
        ret |= cameraWriteRegister(AE_CTRL, 0); // Must disable AE
        ret |= cameraWriteRegister( ANALOG_GAIN, ((gain&0x7)<<4));
        ret |= cameraWriteRegister( GRP_PARAM_HOLD, 0x01);
    } else if ((enable != 0) && (!isnanf(gain_db_ceiling)) && (!isinff(gain_db_ceiling))) {
        gain_db_ceiling = max(min(gain_db_ceiling, 24.0f), 0.0f);
        int gain = fast_ceilf(fast_log2(fast_expf((gain_db_ceiling / 20.0f) * fast_log(10.0f))));
        ret |= cameraWriteRegister( MAX_AGAIN_FULL, (gain&0x7));
        ret |= cameraWriteRegister( MAX_AGAIN_BIN2, (gain&0x7));
        ret |= cameraWriteRegister( AE_CTRL, 1);
    }
    return ret;
}


int HM01B0::get_vt_pix_clk(uint32_t *vt_pix_clk)
{
    uint8_t reg;
    reg = cameraReadRegister(OSC_CLK_DIV);

    // 00 -> MCLK / 8
    // 01 -> MCLK / 4
    // 10 -> MCLK / 2
    // 11 -> MCLK / 1
    uint32_t vt_sys_div = 8 / (1 << (reg & 0x03));

    // vt_pix_clk = MCLK / vt_sys_div
    *vt_pix_clk = OMV_XCLK_FREQUENCY / vt_sys_div;
    return 0;
}


int HM01B0::get_gain_db(float *gain_db)
{
    uint8_t gain;
	gain = cameraReadRegister(ANALOG_GAIN);
    if (gain != 0) {
        return -1;
    }
    *gain_db = fast_floorf(fast_log(1 << (gain>>4)) / fast_log(10.0f) * 20.0f);
    return 0;
}

int HM01B0::getCameraClock(uint32_t *vt_pix_clk)
{
    uint8_t reg;
	reg = cameraReadRegister(OSC_CLK_DIV);

    // 00 -> MCLK / 8
    // 01 -> MCLK / 4
    // 10 -> MCLK / 2
    // 11 -> MCLK / 1
    uint32_t vt_sys_div = 8 / (1 << (reg & 0x03));

    // vt_pix_clk = MCLK / vt_sys_div
    *vt_pix_clk = OMV_XCLK_FREQUENCY / vt_sys_div;
    return 0;
}

int HM01B0::set_auto_exposure(int enable, int exposure_us)
{
    int ret=0;

    if (enable) {
        ret |= cameraWriteRegister( AE_CTRL, 1);
    } else {
        uint32_t line_len;
        uint32_t frame_len;
        uint32_t coarse_int;
        uint32_t vt_pix_clk = 0;

        switch (framesize) {
            case FRAMESIZE_320X320:
                line_len = HIMAX_LINE_LEN_PCK_FULL;
                frame_len = HIMAX_FRAME_LENGTH_FULL;
                break;
            case FRAMESIZE_QVGA:
                line_len = HIMAX_LINE_LEN_PCK_QVGA;
                frame_len = HIMAX_FRAME_LENGTH_QVGA;
                break;
            case FRAMESIZE_QQVGA:
                line_len = HIMAX_LINE_LEN_PCK_QQVGA;
                frame_len = HIMAX_FRAME_LENGTH_QQVGA;
                break;
            default:
                return -1;
        }

        ret |= get_vt_pix_clk(&vt_pix_clk);
        coarse_int = fast_roundf(exposure_us * (vt_pix_clk / 1000000.0f) / line_len);

        if (coarse_int < 2) {
            coarse_int = 2;
        } else if (coarse_int > (frame_len-2)) {
            coarse_int = frame_len-2;
        }

        ret |= cameraWriteRegister( AE_CTRL, 0);
        ret |= cameraWriteRegister( INTEGRATION_H, coarse_int>>8);
        ret |= cameraWriteRegister( INTEGRATION_L, coarse_int&0xff);
        ret |= cameraWriteRegister( GRP_PARAM_HOLD, 0x01);
    }

    return ret;
}

int HM01B0::get_exposure_us(int *exposure_us)
{
    int ret = 0;
    uint32_t line_len;
    uint32_t coarse_int = 0;
    uint32_t vt_pix_clk = 0;
    if (framesize == FRAMESIZE_QVGA) {
        line_len = HIMAX_LINE_LEN_PCK_QVGA;
    } else {
        line_len = HIMAX_LINE_LEN_PCK_QQVGA;
    }
    ret |= get_vt_pix_clk(&vt_pix_clk);
	((uint8_t*)&coarse_int)[1] = cameraReadRegister( INTEGRATION_H);
    ((uint8_t*)&coarse_int)[0] = cameraReadRegister( INTEGRATION_L);
	
    ret |= ((uint8_t*)&coarse_int)[1];
    ret |= ((uint8_t*)&coarse_int)[0] ;
	
    *exposure_us = fast_roundf(coarse_int * line_len / (vt_pix_clk / 1000000.0f));
    return ret;
}

int HM01B0::set_hmirror(int enable)
{
    uint8_t reg;
	reg = cameraReadRegister( IMG_ORIENTATION);
    int ret = reg;
    ret |= cameraWriteRegister( IMG_ORIENTATION, HIMAX_SET_HMIRROR(reg, enable)) ;
    ret |= cameraWriteRegister(  GRP_PARAM_HOLD, 0x01);
    return ret;
}

int HM01B0::set_vflip( int enable)
{
    uint8_t reg;
    reg = cameraReadRegister( IMG_ORIENTATION);
    int ret = reg;
    ret |= cameraWriteRegister(  IMG_ORIENTATION, HIMAX_SET_VMIRROR(reg, enable)) ;
    ret |= cameraWriteRegister(  GRP_PARAM_HOLD, 0x01);
    return ret;
}

uint8_t HM01B0::set_mode(uint8_t Mode, uint8_t FrameCnt)
{
	uint8_t Err = 0;
	
    if (Mode == HIMAX_MODE_STREAMING_NFRAMES)
    {
        Err = cameraWriteRegister(PMU_AUTOSLEEP_FRAMECNT, FrameCnt);
    } else {
        Err = cameraWriteRegister(MODE_SELECT, Mode);
	}
	
    if(Err != 0)
    {
		Serial.println("Mode Could not be set");
    }

    return Err;
}

uint8_t HM01B0::cmdUpdate()
{
	uint8_t status = 0;
    status = cameraWriteRegister(GRP_PARAM_HOLD, 0x01);
	return status;
}


uint8_t HM01B0::loadSettings(camera_reg_settings_t settings)
{
	uint8_t ret = 0;
	switch(settings) {
		case LOAD_DEFAULT_REGS:
			for (int i=0; default_regs[i][0] && ret == 0; i++) {
				ret |=  cameraWriteRegister(default_regs[i][0], default_regs[i][1]);
			}
			break;
		case LOAD_WALKING1S_REG:
			for (int i=0; Walking1s_reg[i][0] && ret == 0; i++) {
				ret |=  cameraWriteRegister(Walking1s_reg[i][0], Walking1s_reg[i][1]);  
			}
			break;
		case LOAD_SHM01B0INIT_REGS:
			w = 320; h = 240;
			framesize = FRAMESIZE_QVGA;
			for (int i=0; sHM01B0Init_regs[i][0] && ret == 0; i++) {
				ret |=  cameraWriteRegister(sHM01B0Init_regs[i][0], sHM01B0Init_regs[i][1]);  
			}
			break;
			
			
		default:
			ret = -1;
	}
	return ret;
}

void HM01B0::readFrame(void* buffer)
{
//uint32_t ulPin = 33; // P1.xx set of GPIO is in 'pin' 32 and above
//NRF_GPIO_Type * port;

  //port = nrf_gpio_pin_port_decode(&ulPin);

  uint8_t* b = (uint8_t*)buffer;
  int bytesPerRow = w ;
    bool _grayscale = (pixformat == PIXFORMAT_GRAYSCALE);

  // Falling edge indicates start of frame
  //pinMode(_pclkPin, INPUT); // make sure back to input pin...
  // lets add our own glitch filter.  Say it must be hig for at least 100us
  elapsedMicros emHigh;
  do {
    while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
    while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
  } while (emHigh < 2);

  for (int i = 0; i < h; i++) {
    // rising edge indicates start of line
    while ((*_hrefPort & _hrefMask) == 0); // wait for HIGH
    while ((*_pclkPort & _pclkMask) != 0); // wait for LOW
    noInterrupts();

    for (int j = 0; j < bytesPerRow; j++) {
      // rising edges clock each data byte
      while ((*_pclkPort & _pclkMask) == 0); // wait for HIGH

      //uint32_t in = ((_frame_buffer_pointer)? GPIO1_DR : GPIO6_DR) >> 18; // read all bits in parallel
      uint32_t in =  (GPIO7_PSR >> 4); // read all bits in parallel
	  //uint32_t in = mmBus;
	  

      if (!(j & 1) || !_grayscale) {
        *b++ = in;
      }
      while (((*_pclkPort & _pclkMask) != 0) && ((*_hrefPort & _hrefMask) != 0)) ; // wait for LOW bail if _href is lost
    }

    while ((*_hrefPort & _hrefMask) != 0); // wait for LOW
    interrupts();
  }
  
   set_mode(HIMAX_MODE_STREAMING, 0);


}



//*****************************************************************************
//
//! @brief Get HM01B0 Model ID
//!
//! @param psCfg                - Pointer to HM01B0 configuration structure.
//! @param pui16MID             - Pointer to buffer for the read back model ID.
//!
//! This function reads back HM01B0 model ID.
//!
//! @return Error code.
//
//*****************************************************************************
uint16_t HM01B0::get_modelid()
{
    uint8_t Data;
    uint16_t MID = 0x0000;

    Data = cameraReadRegister(MODEL_ID_H);
       MID = (Data << 8);

    Data = cameraReadRegister(MODEL_ID_L);
        MID |= Data;

    return MID;
}



//*****************************************************************************
//
//! @brief AE calibration.
//!
//! @param psCfg            - Pointer to HM01B0 configuration structure.
//! @param ui8CalFrames     - Frame counts for calibratoin.
//! @param pui8Buffer       - Pointer to the frame buffer.
//! @param ui32BufferLen    - Framebuffer size.
//! @param pAECfg           - Pointer to AECfg structure to fill with calibration results
//!
//! This function lets HM01B0 AE settled as much as possible within a given frame counts.
//!
//! @return Error code.
//
//*****************************************************************************
uint8_t HM01B0::cal_ae( uint8_t CalFrames, uint8_t* Buffer, uint32_t ui32BufferLen,  ae_cfg_t* pAECfg)
{
    uint8_t        ui32Err     = HM01B0_ERR_OK;
    if(pAECfg == NULL){
        return HM01B0_ERR_PARAMS;
    }

    for (uint8_t i = 0; i < CalFrames; i++)
    {
		set_mode(HIMAX_MODE_STREAMING_NFRAMES, 1);
        readFrame(Buffer);

        ui32Err = get_ae(pAECfg);

        // // todo: could report out intermediate results here (without using printing - perhaps a callback function)
        // SERIAL_PORT.printf("AE Calibration(0x%02X) TargetMean 0x%02X, ConvergeInTh 0x%02X, AEMean 0x%02X\n",
        //                                 ui32Err, sAECfg.ui8AETargetMean, sAECfg.ui8ConvergeInTh, sAECfg.ui8AEMean);

        // if AE calibration is done in ui8CalFrames, just exit to save some time.
        if (ui32Err == HM01B0_ERR_OK)
            break;
    }

    set_mode(HIMAX_MODE_STREAMING, 0);

    return ui32Err;
}


//*****************************************************************************
//
//! @brief Get HM01B0 AE convergance
//!
//! @param psCfg            - Pointer to HM01B0 configuration structure.
//! @param psAECfg          - Pointer to the structure hm01b0_ae_cfg_t.
//!
//! This function checks if AE is converged or not and returns ui32Err accordingly.
//! If caller needs detailed AE settings, psAECfg has to be non NULL.
//!
//! @return Error code.
//
//*****************************************************************************
uint8_t HM01B0::get_ae( ae_cfg_t *psAECfg)
{
    uint32_t    ui32Err = HM01B0_ERR_OK;
    uint8_t     ui8AETargetMean;
    uint8_t     ui8AEMinMean;
    uint8_t     ui8AEMean;
    uint8_t     ui8ConvergeInTh;
    uint8_t     ui8ConvergeOutTh;

	//Read target mean
    ui8AETargetMean = cameraReadRegister( AE_TARGET_MEAN);
    ui8AEMinMean = cameraReadRegister(AE_MIN_MEAN);
    ui8ConvergeInTh = cameraReadRegister( CONVERGE_IN_TH);
    ui8ConvergeOutTh = cameraReadRegister( CONVERGE_OUT_TH);
    if (ui32Err != HM01B0_ERR_OK) return ui32Err;

    ui8AEMean = cameraReadRegister( 0x2020);

    if ((ui8AEMean < (ui8AETargetMean - ui8ConvergeInTh)) || (ui8AEMean > (ui8AETargetMean + ui8ConvergeInTh)))
        ui32Err = HM01B0_ERR_AE_NOT_CONVERGED;

     //Serial.printf("AE Calibration(0x%02X) TargetMean 0x%02X, ConvergeInTh 0x%02X, AEMean 0x%02X\n",	\
     //                                 ui8AETargetMean, ui8ConvergeInTh, ui8AEMean);
    if (psAECfg)
    {
        psAECfg->ui8AETargetMean    = ui8AETargetMean;
        psAECfg->ui8AEMinMean       = ui8AEMinMean;
        psAECfg->ui8ConvergeInTh    = ui8ConvergeInTh;
        psAECfg->ui8ConvergeOutTh   = ui8ConvergeOutTh;
        psAECfg->ui8AEMean          = ui8AEMean;
    }

    return ui32Err;
}


int HM01B0::init()
{
	pinMode(VSYNC_PIN, INPUT_PULLDOWN); // VSYNC Pin
	pinMode(PCLK_PIN, INPUT_PULLDOWN);  //PCLK
	pinMode(HSYNC_PIN, INPUT_PULLDOWN);  //HSYNC
	
	/*Thanks to @luni for how to read 8bit port	\
	 * https://forum.pjrc.com/threads/66771-MicroMod-Beta-Testing?p=275567&viewfull=1#post275567
	 * This interesting too: https://forum.pjrc.com/threads/57698-Parallel-IO-is-it-possible?p=216501&viewfull=1#post216501
	*/
	for (uint8_t pin : {G0, G1, G2, G3, G4, G5, G6, G7})
	{
		pinMode(pin, INPUT_PULLUP);
	}
	//mmBus.pinMode(INPUT);
	
	_vsyncMask = digitalPinToBitMask(VSYNC_PIN);
    _hrefMask = digitalPinToBitMask(HSYNC_PIN);
    _pclkMask = digitalPinToBitMask(PCLK_PIN);

    _vsyncPort = portInputRegister(digitalPinToPort(VSYNC_PIN));
    _hrefPort = portInputRegister(digitalPinToPort(HSYNC_PIN));
    _pclkPort = portInputRegister(digitalPinToPort(PCLK_PIN));
	
	// turn on power to camera (1.8V - might be an issue?)
	pinMode(EN_PIN, OUTPUT);
	digitalWrite(EN_PIN, HIGH);
	delay(10);

	// turn on MCLK
	analogWriteFrequency(MCLK_PIN, OMV_XCLK_FREQUENCY);
	analogWrite(MCLK_PIN, 128);
	delay(5);
	//set_mode(HIMAX_MODE_STREAMING,0);
	
    return 0;
}
