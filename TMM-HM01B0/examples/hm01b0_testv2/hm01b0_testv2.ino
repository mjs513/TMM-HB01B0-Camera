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
//#define USE_SPARKFUN 1
//#define USE_SDCARD 1
File file;

#define TFT_DC  1   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS  4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST  0  // "RX1" on left side of Sparkfun ML Carrier

//#define TFT_ST7789 1
#define TFT_ILI9341 1

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
#include "ILI9341_t3n.h" // https://github.com/KurtE/ILI9341_t3n
ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST);
#define TFT_BLACK ILI9341_BLACK
#define TFT_YELLOW ILI9341_YELLOW
#define TFT_RED   ILI9341_RED
#define TFT_GREEN ILI9341_GREEN
#define TFT_BLUE  ILI9341_BLUE
#endif

uint16_t FRAME_WIDTH, FRAME_HEIGHT;
uint8_t frameBuffer[(324) * 244];
uint16_t imageBuffer[(324) * 244]; DMAMEM
uint8_t sendImageBuf[(324) * 244 * 2];
uint8_t frameBuffer2[(324) * 244] DMAMEM;

bool g_continuous_mode = false;
bool g_continuous_pc_mode = false;
bool g_dma_mode = false;
bool g_vsync = false;

ae_cfg_t aecfg;
elapsedMicros vsync_usec;

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
  delay(500);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW);
  tft.setTextSize(2);
  tft.println("Waiting for Arduino Serial Monitor...");

  Wire.begin();
  Serial.begin(9600);

#if defined(USE_SDCARD)
    Serial.println("Using SDCARD - Initializing");
    if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    //while (1){
    //    LEDON; delay(100);
    //    LEDOFF; delay(100);
    //  }
  }
  Serial.println("initialization done.");
  delay(100);
#endif

  
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
  hm01b0.set_framerate(30);  //15, 30, 60, 120

  /* Gain Ceilling
   * GAINCEILING_1X
   * GAINCEILING_4X
   * GAINCEILING_8X
   * GAINCEILING_16X
   */
  hm01b0.set_gainceiling(GAINCEILING_4X);
  /* Brightness
   *  Can be 1, 2, or 3
   */
  //hm01b0.set_brightness(1);
  hm01b0.set_auto_exposure(true, 250);
  hm01b0.cmdUpdate();  //only need after changing auto exposure settings

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

  Serial.println("Send the 's' character to read a frame ...");
  Serial.println("Send the 'c' character to start/stop continuous display mode");
  Serial.println("Send the 'p' character to snapshot to PC on USB1");
  Serial.println("Send the 'd' character to read a frame using DMA ...");
  Serial.println("Send the 'b' character to save snapshot (BMP) to SD Card");
  Serial.println("send the 'r' Show Camera register settings");
  Serial.println("send the 'v' Show VSYNCH Timing");
  Serial.println("Send the '0' character to blank the display");
  Serial.println();

  //hm01b0.loadSettings(LOAD_WALKING1S_REG);
  //hm01b0.set_colorbar(true);

 vsync_usec = 0;

  
}

uint16_t *last_dma_frame_buffer = nullptr;

void hm01b0_dma_callback(void *pfb) {
  //Serial.printf("Callback: %x\n", (uint32_t)pfb);
  uint8_t *pframeBuffer = (uint8_t*)pfb;
  for(int i = 0; i < FRAME_HEIGHT*FRAME_WIDTH; i++) {
    uint8_t b = *pframeBuffer++;
    imageBuffer[i] = color565(b, b, b);
  }
  tft.writeRect(0, 0, tft.width(), tft.height(), imageBuffer);

  tft.updateScreenAsync();

  last_dma_frame_buffer = (uint16_t*)pfb;
}


void loop()
{
  if(g_vsync) {
      static int prior_vsync = LOW;
    
      int vsync = digitalRead(33);
    
      if (prior_vsync == LOW && vsync == HIGH) {
        uint32_t us = vsync_usec;
        vsync_usec = 0;
        Serial.print("VSYNC event: ");
        Serial.print(1000000.0f / (float)us);
        Serial.println(" Hz");
      }
      prior_vsync = vsync;
  }
  
  if (Serial.available()) {
    int ch = Serial.read();
    while (Serial.read() != -1); // get rid of the rest...
    switch (ch) {
      case 's':
       {
          tft.fillScreen(TFT_BLACK);
          calAE();
          Serial.println("Reading frame");
          memset((uint8_t*)frameBuffer, 0, sizeof(frameBuffer));
          hm01b0.set_mode(HIMAX_MODE_STREAMING_NFRAMES, 1);
          hm01b0.readFrame(frameBuffer);
          Serial.println("Finished reading frame"); Serial.flush();
          //convert grayscale to rgb
          for(int i = 0; i < FRAME_HEIGHT*FRAME_WIDTH; i++) {
            imageBuffer[i] = color565(frameBuffer[i], frameBuffer[i], frameBuffer[i]);
          }
          tft.writeSubImageRect(0, 0, tft.width(), tft.height(),  (FRAME_WIDTH - tft.width()) / 2, (FRAME_HEIGHT - tft.height()),
                                  FRAME_WIDTH, FRAME_HEIGHT, imageBuffer);
          ch = ' ';
          g_continuous_mode = false;
          break;
        }
      case 'c':
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
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
          calAE();
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
#else 
          Serial.println("*** Only works in USB Dual or Tripple Serial Mode ***");
#endif          
          break;
      }
      case 'b':
       {
        #if defined(USE_SDCARD)
          calAE();
          memset((uint8_t*)frameBuffer, 0, sizeof(frameBuffer));
          hm01b0.set_mode(HIMAX_MODE_STREAMING_NFRAMES, 1);
          hm01b0.readFrame(frameBuffer);
          save_image_SD();
        #endif
          break;
       }
      case 'd':
        {
          if (g_dma_mode) {
            Serial.println("*** stopReadFrameDMA ***");
            hm01b0.stopReadFrameDMA();
            Serial.println("*** return from stopReadFrameDMA ***");
            tft.endUpdateAsync();
            tft.useFrameBuffer(false);
            g_dma_mode = false;
          } else {
            hm01b0.startReadFrameDMA(&hm01b0_dma_callback, frameBuffer, frameBuffer2);
            Serial.println("*** Return from startReadFrameDMA ***");
            tft.useFrameBuffer(true);
            //        tft.updateScreenAsync(true);
            //        Serial.println("*** Return from updateScreenAsync ***");
            g_dma_mode = true;
          }
          break;
        }
      case 'r':
        hm01b0.showRegisters();
        break;

      case '0':
       {
          tft.fillScreen(TFT_BLACK);
          break;
       }
      case 'v':
      {
        if (g_vsync) {
          g_vsync = false;
        } else {
          g_vsync = true;
        }
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
        //tft.writeRect(0, 0, tft.width(), tft.height(), imageBuffer);
        tft.writeSubImageRect(0, 0, tft.width(), tft.height(),  (FRAME_WIDTH - tft.width()) / 2, (FRAME_HEIGHT - tft.height()),
                              FRAME_WIDTH, FRAME_HEIGHT, imageBuffer);
  }

}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


void calAE(){
  // Calibrate Autoexposure
  Serial.println("Calibrating Auto Exposure..."); 
  memset((uint8_t*)frameBuffer, 0, sizeof(frameBuffer));
  if(hm01b0.cal_ae(10, frameBuffer, FRAME_WIDTH*FRAME_HEIGHT, &aecfg) != HM01B0_ERR_OK){
    Serial.println("\tnot converged"); 
  }else{
    Serial.println("\tconverged!");
    hm01b0.cmdUpdate();
  }
}

#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
void send_raw() {
  uint32_t imagesize;
  imagesize = (FRAME_WIDTH * FRAME_HEIGHT * 2);
  SerialUSB1.write(sendImageBuf, imagesize); // set Tools > USB Type to "Dual Serial"
}
#endif

char name[] = "9px_0000.bmp";       // filename convention (will auto-increment)
void save_image_SD(){
  uint8_t r, g, b;
  uint32_t x, y;

  Serial.println("Writing BMP to SD CARD");
  Serial.println(name);
  
  // if name exists, create new filename, SD.exists(filename)
    for (int i=0; i<10000; i++) {
      name[4] = (i/1000)%10 + '0';    // thousands place
      name[5] = (i/100)%10 + '0';     // hundreds
      name[6] = (i/10)%10 + '0';      // tens
      name[7] = i%10 + '0';           // ones
      if(!SD.exists(name)){
        Serial.println(name);
        file = SD.open(name, FILE_WRITE);
        break;
      }
    }

  uint16_t w = FRAME_WIDTH;
  uint16_t h = FRAME_HEIGHT;

  unsigned char *img = NULL;

  // set fileSize (used in bmp header)
  int rowSize = 4 * ((3*w + 3)/4);      // how many bytes in the row (used to create padding)
  int fileSize = 54 + h*rowSize;        // headers (54 bytes) + pixel data

  img = (unsigned char *)malloc(3*w*h);
  
  for(int i=0; i<w; i++)
  {
    for(int j=0; j<h; j++)
    {
      //r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
      x=i; y=(h-1)-j;
            
      r = frameBuffer[(x+y*w)]    ;
      g = frameBuffer[(x+y*w)]    ;
      b = frameBuffer[(x+y*w)]    ;

      img[(x+y*w)*3+2] = (unsigned char)(r);
      img[(x+y*w)*3+1] = (unsigned char)(g);
      img[(x+y*w)*3+0] = (unsigned char)(b);
    }
  }

  // create padding (based on the number of pixels in a row
  unsigned char bmpPad[rowSize - 3*w];
  for (int i=0; i< (int)(sizeof(bmpPad)); i++) {         // fill with 0s
    bmpPad[i] = 0;
  }

  unsigned char bmpFileHeader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
  unsigned char bmpInfoHeader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};

  bmpFileHeader[ 2] = (unsigned char)(fileSize      );
  bmpFileHeader[ 3] = (unsigned char)(fileSize >>  8);
  bmpFileHeader[ 4] = (unsigned char)(fileSize >> 16);
  bmpFileHeader[ 5] = (unsigned char)(fileSize >> 24);

  bmpInfoHeader[ 4] = (unsigned char)(       w    );
  bmpInfoHeader[ 5] = (unsigned char)(       w>> 8);
  bmpInfoHeader[ 6] = (unsigned char)(       w>>16);
  bmpInfoHeader[ 7] = (unsigned char)(       w>>24);
  bmpInfoHeader[ 8] = (unsigned char)(       h    );
  bmpInfoHeader[ 9] = (unsigned char)(       h>> 8);
  bmpInfoHeader[10] = (unsigned char)(       h>>16);
  bmpInfoHeader[11] = (unsigned char)(       h>>24);

  // write the file (thanks forum!)
  file.write(bmpFileHeader, sizeof(bmpFileHeader));    // write file header
  file.write(bmpInfoHeader, sizeof(bmpInfoHeader));    // " info header

  for (int i=0; i<h; i++) {                            // iterate image array
    file.write(img+(FRAME_WIDTH*(FRAME_HEIGHT-i-1)*3), 3*FRAME_WIDTH);                // write px data
    file.write(bmpPad, (4-(FRAME_WIDTH*3)%4)%4);                 // and padding as needed
  }
  free(img);
  file.close();                                        // close file when done writing
  Serial.println("Done Writing BMP");
}
