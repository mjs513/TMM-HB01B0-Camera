/*
HM01B0 pin      pin#    NXP     Usage
----------      ----    ---     -----
FVLD/VSYNC      33      EMC_07  GPIO
LVLD/HSYNC      32      B0_12   FlexIO2:12
MCLK            7       B1_01   PWM
PCLK            8       B1_00   FlexIO2:16
D0              40      B0_04   FlexIO2:4
D1              41      B0_05   FlexIO2:5
D2              42      B0_06   FlexIO2:6
D3              43      B0_07   FlexIO2:7
D4              44      B0_08   FlexIO2:8  - probably not needed, use 4 bit mode
D5              45      B0_09   FlexIO2:9  - probably not needed, use 4 bit mode
D6              6       B0_10   FlexIO2:10 - probably not needed, use 4 bit mode
D7              9       B0_11   FlexIO2:11 - probably not needed, use 4 bit mode
TRIG            5       EMC_08  ???
INT             29      EMC_31  ???
SCL             19      AD_B1_0 I2C
SDA             18      AD_B1_1 I2C
*/


#include <stdint.h>
#include <Wire.h>
#include "HM01B0.h"
#include "HM01B0_regs.h"
#include <SD.h>
#include <SPI.h>

HM01B0 hm01b0;
#define USE_SPARKFUN 1


#define TFT_DC  1
#define TFT_CS  5
#define TFT_RST  0

#define TFT_ST7789 1
//#define TFT_ILI9341 1

#ifdef TFT_ST7789
//ST7735 Adafruit 320x240 display
#include <ST7789_t3.h>
ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_RST);
#define TFT_BLACK ST77XX_BLACK
#define TFT_YELLOW ST77XX_YELLOW
#define TFT_RED   ST77XX_RED
#define TFT_GREEN ST77XX_GREEN
#define TFT_BLUE  ST77XX_BLUE

#else
#include "ILI9341_t3.h"
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST);
#define TFT_BLACK ILI9341_BLACK
#define TFT_YELLOW ILI9341_YELLOW
#define TFT_RED   ILI9341_RED
#define TFT_GREEN ILI9341_GREEN
#define TFT_BLUE  ILI9341_BLUE
#endif


# pragma pack (2)
typedef struct  tBMPHDR565 {
  uint16_t  bfType = 0x4d42;   //'bm';
  uint32_t  bfSize = 614466;// 614400 pixels + 66 header
  uint16_t  bfReserved1 = 0;
  uint16_t  bfReserved2 = 0;
  uint32_t  bfOffBits =  66; // 14 bytes to here
  uint32_t  biSize = 40;
  int32_t   biWidth = 640;
  int32_t   biHeight = -480;  // windows wants negative for top-down image
  int16_t   biPlanes = 1;
  uint16_t  biBitCount = 16 ;
  uint32_t  biCompression = 3;  // bitfields used
  uint32_t  biSizeImage = 614400;  // 640 * 480 * 2
  int32_t   biXPelsPerMeter = 0;
  int32_t   biYPelsPerMeter = 0;
  uint32_t  biClrUsed  = 0;
  uint32_t  biClrImportant = 0;// 54 bytes
  uint32_t  rmask = 0x0000F800;
  uint32_t  gmask = 0x000007E0;
  uint32_t  bmask = 0x0000001F;  //66 bytes
} BMPHDR565;

tBMPHDR565 fileheader;
File bmpfile;

uint16_t FRAME_WIDTH, FRAME_HEIGHT;
uint8_t frameBuffer[(324) * 244];
uint8_t sendImageBuf[324 * 244 * 2];
uint16_t imageBuffer[(324) * 244];

bool g_continuous_mode = false;
bool g_continuous_pc_mode = false;

ae_cfg_t aecfg;

void setup()
{
#ifdef TFT_ILI9341
  tft.begin();
#else
  tft.init(240, 320);           // Init ST7789 320x240
#endif
  tft.setRotation(1);
  tft.fillScreen(TFT_RED);
  delay(500);
  tft.fillScreen(TFT_GREEN);
  delay(500);
  tft.fillScreen(TFT_BLUE);
  delay(500);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW);
  tft.setTextSize(2);
  tft.println("Waiting for Arduino Serial Monitor...");

  Wire.begin();
  Serial.begin(9600);

  
  while (!Serial) ;
  Serial.println("HM01B0 Camera Test");
  Serial.println("------------------");

  hm01b0.init();
  delay(500);

  tft.fillScreen(TFT_BLACK);


  uint16_t ModelID;
  ModelID = hm01b0.get_modelid();
  if(ModelID == 0x01B0){
    Serial.printf("SENSOR DETECTED :-) MODEL HM0%X\n", ModelID);
  } else {
    Serial.println("SENSOR NOT DETECTED! :-(");
    while(1){}
  }

  /*
   * FRAMESIZE_INVALID = 0,
   * VGA Resolutions
   * FRAMESIZE_QQVGA,    // 160x120
   * FRAMESIZE_QVGA,     // 320x240
   * FRAMESIZE_320X320,  // 320x320
   */
  uint8_t status;
  #if defined(USE_SPARKFUN)
    status = hm01b0.loadSettings(LOAD_SHM01B0INIT_REGS);  //hangs the TMM.
  #else  
    status = hm01b0.loadSettings(LOAD_DEFAULT_REGS);
    status = hm01b0.set_framesize(FRAMESIZE_QVGA);
  #endif
  
  if(status != 0) {
    Serial.println("Settings failed to load");
    while(1){}
  }
  //hm01b0.cmdUpdate();  //only need after changing auto exposure settings
  hm01b0.set_framerate(30);  //15, 30, 60, 120
  hm01b0.set_pixformat(PIXFORMAT_GRAYSCALE);
  hm01b0.set_brightness(1);

  hm01b0.set_mode(HIMAX_MODE_STREAMING,0); // turn on, continuous streaming mode

  uint32_t pixClk;
  hm01b0.get_vt_pix_clk(&pixClk);
  Serial.printf("Pixel Clock: %u\n", pixClk);
  uint32_t camClk;
  hm01b0.getCameraClock(&camClk);
  Serial.printf("Camera Clock: %u\n", camClk);

  FRAME_HEIGHT = hm01b0.h;
  FRAME_WIDTH  = hm01b0.w;
  Serial.printf("ImageSize (w,h): %d, %d\n", hm01b0.w, hm01b0.h);

  // Calibrate Autoexposure
  Serial.println("Calibrating Auto Exposure..."); 
  memset((uint8_t*)frameBuffer, 0, sizeof(frameBuffer));
  if(hm01b0.cal_ae(10, frameBuffer, FRAME_WIDTH*FRAME_HEIGHT, &aecfg) != HM01B0_ERR_OK){
    Serial.println("\tnot converged"); 
  }else{
    Serial.println("\tconverged!");
    hm01b0.cmdUpdate();
  }
  
  Serial.println("Send the 'c' character to read a frame ...");
  Serial.println("Send the 's' character to start/stop continuous display mode");
  Serial.println("Send the 'p' character to snapshot to PC on USB1");
  Serial.println("Send the 'b' character to save snapshot (BMP) to SD Card");
  Serial.println();

  //hm01b0.loadSettings(LOAD_WALKING1S_REG);
  //hm01b0.set_colorbar(true);
  
}



void loop()
{
  if (Serial.available()) {
    int ch = Serial.read();
    while (Serial.read() != -1); // get rid of the rest...
    switch (ch) {
      case 'c':
       {
          tft.fillScreen(TFT_BLACK); delay(4);
          Serial.println("Reading frame");
          memset((uint8_t*)frameBuffer, 0, sizeof(frameBuffer));
          hm01b0.set_mode(HIMAX_MODE_STREAMING_NFRAMES, 1);
          hm01b0.readFrame(frameBuffer);
          Serial.println("Finished reading frame"); Serial.flush();
          //convert grayscale to rgb
          for(int i = 0; i < FRAME_HEIGHT*FRAME_WIDTH; i++) {
            imageBuffer[i] = color565(frameBuffer[i], frameBuffer[i], frameBuffer[i]);
          }
          tft.writeRect(0, 0, tft.width(), tft.height(), imageBuffer);
          ch = ' ';
          g_continuous_mode = false;
          break;
        }
      case 's':
      {
        if (g_continuous_mode) {
          g_continuous_mode = false;
          Serial.println("*** Continuous mode turned off");
        } else {
          g_continuous_mode = true;
          Serial.println("*** Continuous mode turned on");
        }
        break;
      }
      case 'p':
      {
          memset((uint8_t*)frameBuffer, 0, sizeof(frameBuffer));
          hm01b0.set_mode(HIMAX_MODE_STREAMING_NFRAMES, 1);
          hm01b0.readFrame(frameBuffer);
          uint32_t idx = 0;
          for(int i = 0; i < FRAME_HEIGHT*FRAME_WIDTH; i++) {
            idx = i*2;
            imageBuffer[i] = color565(frameBuffer[i], frameBuffer[i], frameBuffer[i]);
            sendImageBuf[idx+1] = (imageBuffer[i] >> 0) & 0xFF;
            sendImageBuf[idx] = (imageBuffer[i] >> 8) & 0xFF;
          }
          send_raw();
          Serial.println("Image Sent!");
          ch = ' ';
          g_continuous_mode = false;
          break;
      }
       case 'b':
       {
          memset((uint8_t*)frameBuffer, 0, sizeof(frameBuffer));
          hm01b0.set_mode(HIMAX_MODE_STREAMING_NFRAMES, 1);
          hm01b0.readFrame(frameBuffer);
          uint32_t idx1 = 0;
          for(int i = 0; i < FRAME_HEIGHT*FRAME_WIDTH; i++) {
            idx1 = i*2;
            imageBuffer[i] = frameBuffer[i] << 16 | frameBuffer[i] << 8 | frameBuffer[i];
            sendImageBuf[idx1+1] = (imageBuffer[i] >> 0) & 0xFF;
            sendImageBuf[idx1] = (imageBuffer[i] >> 8) & 0xFF;
          }
          save_image_SD();
          break;
       }
     }
  }


  if(g_continuous_mode){ 
        memset((uint8_t*)frameBuffer, 0, sizeof(frameBuffer));
        hm01b0.set_mode(HIMAX_MODE_STREAMING_NFRAMES, 1);
        hm01b0.readFrame(frameBuffer);
        for(int i = 0; i < FRAME_HEIGHT*FRAME_WIDTH; i++) {
          imageBuffer[i] = color565(frameBuffer[i], frameBuffer[i], frameBuffer[i]);
        }
        tft.writeRect(0, 0, tft.width(), tft.height(), imageBuffer);
  }

}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void send_raw() {
  uint32_t imagesize;
  imagesize = (FRAME_WIDTH * FRAME_HEIGHT * 2);
  SerialUSB1.write(sendImageBuf, imagesize);
}

void save_image_SD() {
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    //while (1){
    //    LEDON; delay(100);
    //    LEDOFF; delay(100);
    //  }
  }
  Serial.println("initialization done.");

  delay(100);

  uint32_t imagesize;
  imagesize = (FRAME_WIDTH * FRAME_HEIGHT * 2);

  // lets update fileheader with our actual image size...
  fileheader.biSizeImage = imagesize;
  fileheader.biWidth = FRAME_WIDTH;
  fileheader.biHeight = -FRAME_HEIGHT;
  fileheader.bfSize = imagesize + fileheader.bfOffBits;

  Serial.printf("fb ready to save %lu bytes.", imagesize);
  Serial.println("Saving HM01B0.bmp ");
  if (SD.exists("HM01B0.bmp")) {
    // delete the file:
    Serial.println("Removing HM01B0.bmp...");
    SD.remove("HM01B0.bmp");
  }
  bmpfile = SD.open("HM01B0.bmp", FILE_WRITE);
  bmpfile.write((const uint8_t *)&fileheader, sizeof(fileheader));
  bmpfile.write(sendImageBuf, imagesize);
  bmpfile.close();
  Serial.println("File saved to SD card.");
}
