#include <stdint.h>
#include <SD.h>
#include <SPI.h>

#include <Wire.h>

#include "HM01B0.h"
#include "HM01B0_regs.h"

// Color to Monochrome Mapping since the HM01B0 is a monochrome camera
#define MCP(m) (uint16_t)(((m & 0xF8) << 8) | ((m & 0xFC) << 3) | (m >> 3))

static const uint16_t mono_palette[256] PROGMEM = {
  MCP(0x00), MCP(0x01), MCP(0x02), MCP(0x03), MCP(0x04), MCP(0x05), MCP(0x06), MCP(0x07), MCP(0x08), MCP(0x09), MCP(0x0a), MCP(0x0b), MCP(0x0c), MCP(0x0d), MCP(0x0e), MCP(0x0f),
  MCP(0x10), MCP(0x11), MCP(0x12), MCP(0x13), MCP(0x14), MCP(0x15), MCP(0x16), MCP(0x17), MCP(0x18), MCP(0x19), MCP(0x1a), MCP(0x1b), MCP(0x1c), MCP(0x1d), MCP(0x1e), MCP(0x1f),
  MCP(0x20), MCP(0x21), MCP(0x22), MCP(0x23), MCP(0x24), MCP(0x25), MCP(0x26), MCP(0x27), MCP(0x28), MCP(0x29), MCP(0x2a), MCP(0x2b), MCP(0x2c), MCP(0x2d), MCP(0x2e), MCP(0x2f),
  MCP(0x30), MCP(0x31), MCP(0x32), MCP(0x33), MCP(0x34), MCP(0x35), MCP(0x36), MCP(0x37), MCP(0x38), MCP(0x39), MCP(0x3a), MCP(0x3b), MCP(0x3c), MCP(0x3d), MCP(0x3e), MCP(0x3f),
  MCP(0x40), MCP(0x41), MCP(0x42), MCP(0x43), MCP(0x44), MCP(0x45), MCP(0x46), MCP(0x47), MCP(0x48), MCP(0x49), MCP(0x4a), MCP(0x4b), MCP(0x4c), MCP(0x4d), MCP(0x4e), MCP(0x4f),
  MCP(0x50), MCP(0x51), MCP(0x52), MCP(0x53), MCP(0x54), MCP(0x55), MCP(0x56), MCP(0x57), MCP(0x58), MCP(0x59), MCP(0x5a), MCP(0x5b), MCP(0x5c), MCP(0x5d), MCP(0x5e), MCP(0x5f),
  MCP(0x60), MCP(0x61), MCP(0x62), MCP(0x63), MCP(0x64), MCP(0x65), MCP(0x66), MCP(0x67), MCP(0x68), MCP(0x69), MCP(0x6a), MCP(0x6b), MCP(0x6c), MCP(0x6d), MCP(0x6e), MCP(0x6f),
  MCP(0x70), MCP(0x71), MCP(0x72), MCP(0x73), MCP(0x74), MCP(0x75), MCP(0x76), MCP(0x77), MCP(0x78), MCP(0x79), MCP(0x7a), MCP(0x7b), MCP(0x7c), MCP(0x7d), MCP(0x7e), MCP(0x7f),
  MCP(0x80), MCP(0x81), MCP(0x82), MCP(0x83), MCP(0x84), MCP(0x85), MCP(0x86), MCP(0x87), MCP(0x88), MCP(0x89), MCP(0x8a), MCP(0x8b), MCP(0x8c), MCP(0x8d), MCP(0x8e), MCP(0x8f),
  MCP(0x90), MCP(0x91), MCP(0x92), MCP(0x93), MCP(0x94), MCP(0x95), MCP(0x96), MCP(0x97), MCP(0x98), MCP(0x99), MCP(0x9a), MCP(0x9b), MCP(0x9c), MCP(0x9d), MCP(0x9e), MCP(0x9f),
  MCP(0xa0), MCP(0xa1), MCP(0xa2), MCP(0xa3), MCP(0xa4), MCP(0xa5), MCP(0xa6), MCP(0xa7), MCP(0xa8), MCP(0xa9), MCP(0xaa), MCP(0xab), MCP(0xac), MCP(0xad), MCP(0xae), MCP(0xaf),
  MCP(0xb0), MCP(0xb1), MCP(0xb2), MCP(0xb3), MCP(0xb4), MCP(0xb5), MCP(0xb6), MCP(0xb7), MCP(0xb8), MCP(0xb9), MCP(0xba), MCP(0xbb), MCP(0xbc), MCP(0xbd), MCP(0xbe), MCP(0xbf),
  MCP(0xc0), MCP(0xc1), MCP(0xc2), MCP(0xc3), MCP(0xc4), MCP(0xc5), MCP(0xc6), MCP(0xc7), MCP(0xc8), MCP(0xc9), MCP(0xca), MCP(0xcb), MCP(0xcc), MCP(0xcd), MCP(0xce), MCP(0xcf),
  MCP(0xd0), MCP(0xd1), MCP(0xd2), MCP(0xd3), MCP(0xd4), MCP(0xd5), MCP(0xd6), MCP(0xd7), MCP(0xd8), MCP(0xd9), MCP(0xda), MCP(0xdb), MCP(0xdc), MCP(0xdd), MCP(0xde), MCP(0xdf),
  MCP(0xe0), MCP(0xe1), MCP(0xe2), MCP(0xe3), MCP(0xe4), MCP(0xe5), MCP(0xe6), MCP(0xe7), MCP(0xe8), MCP(0xe9), MCP(0xea), MCP(0xeb), MCP(0xec), MCP(0xed), MCP(0xee), MCP(0xef),
  MCP(0xf0), MCP(0xf1), MCP(0xf2), MCP(0xf3), MCP(0xf4), MCP(0xf5), MCP(0xf6), MCP(0xf7), MCP(0xf8), MCP(0xf9), MCP(0xfa), MCP(0xfb), MCP(0xfc), MCP(0xfd), MCP(0xfe), MCP(0xff)
};


// BMP config for saving to SD Card or PC display
#define BMPIMAGEOFFSET 66
const char bmp_header[BMPIMAGEOFFSET] PROGMEM =
{
  0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
  0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
  0x00, 0x00
};

// Library supports 8-bit or 4-bit camera interfacing to the Teensy Micromod Processor via pin selection
// or supports the default camera configuration.
PROGMEM const char hmConfig[][48] = {
 "HM01B0_SPARKFUN_ML_CARRIER",    
 "HM01B0_FLEXIO_CUSTOM_LIKE_8_BIT",
 "HM01B0_FLEXIO_CUSTOM_LIKE_4_BIT"
};

#define _hmConfig 1 // select mode 

#if _hmConfig == 0
HM01B0 hm01b0(HM01B0_SPARKFUN_ML_CARRIER);

#elif _hmConfig == 1
/* We are doing manual settings: 
 * this one should duplicate the 8 bit ML Carrier:
 *    HM01B0(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, en_pin,
 *    uint8_t g0, uint8_t g1,uint8_t g2, uint8_t g3,
 *    uint8_t g4=0xff, uint8_t g5=0xff,uint8_t g6=0xff,uint8_t g7=0xff, TwoWire &wire=Wire); 
 */
HM01B0 hm01b0(7, 8, 33, 32, 2, 40, 41, 42, 43, 44, 45, 6, 9);

#elif _hmConfig == 2
/* We are doing manual settings: 
 * this sets the camera up using 4 bits only.
 *    HM01B0(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, en_pin,
 *    uint8_t g0, uint8_t g1,uint8_t g2, uint8_t g3,
 *    uint8_t g4=0xff, uint8_t g5=0xff,uint8_t g6=0xff,uint8_t g7=0xff, TwoWire &wire=Wire);
 */
HM01B0 hm01b0(7, 8, 33, 32, 2, 40, 41, 42, 43);
#endif

/*  Camera Configuration
 *  Two base configurations are used to initalize the camera.  The first is the Sparakfun configuration
 *  used in their library, the second is the configurations used by the OpenMV camera (modified) for the HM01B0
 *  If the define for USE_SPARKFUN is uncommented the sketch will use the Sparkfun configuration otherwise it will
 *  the OpenMV version which we modified slightly
 *  NOTE:
 *    1. If using 4 bit mode use set_framerate(60) with OpenMV config
 *    2. If using 8 bit/Sparkfun ML with the Sparkfun config use frameRate of 30. You do get flicker    
 *       using this combination.
 *    3. If using 8 bit/SparkfunML mode use set_framerate(60) with OpenMV config
 */
//#define USE_SPARKFUN 1

// If you want to use the SDCard to store images uncomment the following line
#define USE_SDCARD 1
File file;

/* The sketch supports 2 displays: ST7789 and ILI9431 displays.  The ST7789 library is installed automatically
 * when you install Teensyduino.  For the ILI931 to work properly you will need to download and install the 
 * ILI9341_t3n library - https://github.com/KurtE/ILI9341_t3n
 * Select display by uncommenting the one you want and commenting out the other one.
 */
//#define TFT_ST7789 1
#define TFT_ILI9341 1

#define TFT_DC  1   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS  4   // "D0" on right side of Sparkfun ML Carrier
#define TFT_RST 0   // "RX1" on left side of Sparkfun ML Carrier
#define TFT_BL  5   // "D1" on right side of Sparkfun ML Carrier

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

// Configs frameBuffers for use with the camera
uint16_t FRAME_WIDTH, FRAME_HEIGHT;
uint8_t frameBuffer[(324) * 244];
uint8_t sendImageBuf[(324) * 244 * 2];
uint8_t frameBuffer2[(324) * 244] DMAMEM;

//Sketch defines
bool g_continuous_flex_mode = false;
void * volatile g_new_flexio_data = nullptr;
uint32_t g_flexio_capture_count = 0;
uint32_t g_flexio_redraw_count = 0;
elapsedMillis g_flexio_runtime;
bool g_dma_mode = false;

ae_cfg_t aecfg;

void setup()
{
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  delay(100);
  
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

  Serial.begin(921600);

#if defined(USE_SDCARD)
  Serial.println("Using SDCARD - Initializing");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
  }
  Serial.println("initialization done.");
  delay(100);
#endif


  while (!Serial) ;
  Serial.println("HM01B0 Camera Test");
  Serial.println( hmConfig[_hmConfig] );
  Serial.println("------------------");

  tft.fillScreen(TFT_BLACK);

  uint16_t ModelID;
  ModelID = hm01b0.get_modelid();
  if (ModelID == 0x01B0) {
    Serial.printf("SENSOR DETECTED :-) MODEL HM0%X\n", ModelID);
  } else {
    Serial.println("SENSOR NOT DETECTED! :-(");
    while (1) {}
  }


  uint8_t status;
#if defined(USE_SPARKFUN)
  status = hm01b0.loadSettings(LOAD_SHM01B0INIT_REGS);  //hangs the TMM.
#else
  status = hm01b0.loadSettings(LOAD_DEFAULT_REGS);
#endif

  if(_hmConfig == 2){
    status = hm01b0.set_framesize(FRAMESIZE_QVGA4BIT);
  } else {
    status = hm01b0.set_framesize(FRAMESIZE_QVGA);
  }
  if (status != 0) {
    Serial.println("Settings failed to load");
    while (1) {}
  }
  hm01b0.set_framerate(60);  //15, 30, 60

  /* Gain Ceilling
   * GAINCEILING_1X
   * GAINCEILING_4X
   * GAINCEILING_8X
   * GAINCEILING_16X
   */
  hm01b0.set_gainceiling(GAINCEILING_2X);
  
  /* Brightness
   *  Can be 1, 2, or 3
   */
  hm01b0.set_brightness(3);
  hm01b0.set_autoExposure(true, 1500);  //higher the setting the less saturaturation of whiteness
  hm01b0.cmdUpdate();  //only need after changing auto exposure settings

  hm01b0.set_mode(HIMAX_MODE_STREAMING, 0); // turn on, continuous streaming mode

  FRAME_HEIGHT = hm01b0.height();
  FRAME_WIDTH  = hm01b0.width();
  Serial.printf("ImageSize (w,h): %d, %d\n", FRAME_WIDTH, FRAME_HEIGHT);

  showCommandList();
}

// Callbacks for frame buffers
bool hm01b0_flexio_callback(void *pfb)
{
  //Serial.println("Flexio callback");
  g_new_flexio_data = pfb;
  return true;
}
// Quick and Dirty
#define UPDATE_ON_CAMERA_FRAMES

uint8_t *pfb_last_frame_returned = nullptr;

bool hm01b0_flexio_callback_video(void *pfb)
{
  pfb_last_frame_returned = (uint8_t*)pfb;
#ifdef UPDATE_ON_CAMERA_FRAMES
  tft.setOrigin(-2, -2);
  if ((uint32_t)pfb_last_frame_returned >= 0x20200000u)
    arm_dcache_delete(pfb_last_frame_returned, FRAME_WIDTH*FRAME_HEIGHT);

  tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, (uint8_t*)pfb_last_frame_returned, mono_palette);
  pfb_last_frame_returned = nullptr;
  tft.setOrigin(0, 0);
  uint16_t *pframebuf = tft.getFrameBuffer();
  if ((uint32_t)pframebuf >= 0x20200000u) arm_dcache_flush(pframebuf, FRAME_WIDTH*FRAME_HEIGHT);
#endif  
  //Serial.print("#");
  return true;
}

void frame_complete_cb() {
  //Serial.print("@");
#ifndef UPDATE_ON_CAMERA_FRAMES
  if (!pfb_last_frame_returned) return;
  tft.setOrigin(-2, -2);
  if ((uint32_t)pfb_last_frame_returned >= 0x20200000u)
    arm_dcache_delete(pfb_last_frame_returned, FRAME_WIDTH*FRAME_HEIGHT);

  tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, (uint8_t*)pfb_last_frame_returned, mono_palette);
  pfb_last_frame_returned = nullptr;
  tft.setOrigin(0, 0);
  uint16_t *pfb = tft.getFrameBuffer();
  if ((uint32_t)pfb >= 0x20200000u) arm_dcache_flush(pfb, FRAME_WIDTH*FRAME_HEIGHT);
#endif
}


void loop()
{
  char ch;

  #if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
  while (SerialUSB1.available()) {
    ch = SerialUSB1.read();
    if ( 0x30 == ch ) {
      Serial.print(F("ACK CMD CAM start single shoot ... "));
      send_image( &SerialUSB1 );
      Serial.println(F("READY. END"));
    }
  }
  #endif
  if (Serial.available()) {
    ch = Serial.read();
    switch (ch) {
      case 'p':
      {
  #if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
        uint16_t pixel;
        memset((uint8_t*)frameBuffer, 0, sizeof(frameBuffer));
        hm01b0.readFrame(frameBuffer);
        uint32_t idx = 0;
        for (int i = 0; i < FRAME_HEIGHT * FRAME_WIDTH; i++) {
          idx = i * 2;
          pixel = color565(frameBuffer[i], frameBuffer[i], frameBuffer[i]);
          sendImageBuf[idx + 1] = (pixel >> 0) & 0xFF;
          sendImageBuf[idx] = (pixel >> 8) & 0xFF;
        }
        send_raw();
        Serial.println("Image Sent!");
        ch = ' ';
  #else
        Serial.println("*** Only works in USB Dual or Triple Serial Mode ***");
  #endif
        break;
      }
      case 'z':
      {
  #if defined(USE_SDCARD)
        save_image_SD();
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
        ch = ' '; 
  #endif
        break;
      }

      case 'f':
      {
        tft.useFrameBuffer(false);
        tft.fillScreen(TFT_BLACK);
        //calAE();
        Serial.println("Reading frame");
        memset((uint8_t*)frameBuffer, 0, sizeof(frameBuffer));
        hm01b0.readFrame(frameBuffer);
        Serial.println("Finished reading frame"); Serial.flush();
        tft.setOrigin(-2, -2);
        tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, frameBuffer, mono_palette);
        tft.setOrigin(0, 0);
        ch = ' ';
        g_continuous_flex_mode = false;
        break;
      }
      case 'F':
      {
        if (!g_continuous_flex_mode) {
          if (hm01b0.readContinuous(&hm01b0_flexio_callback, frameBuffer, frameBuffer2)) {
            Serial.println("* continuous mode started");
            g_flexio_capture_count = 0;
            g_flexio_redraw_count = 0;
            g_continuous_flex_mode = true;
          } else {
            Serial.println("* error, could not start continuous mode");
          }
        } else {
          hm01b0.stopReadContinuous();
          g_continuous_flex_mode = false;
          Serial.println("* continuous mode stopped");
        }
        break;
      }
      case 'V':
      {
        if (!g_continuous_flex_mode) {
          if (hm01b0.readContinuous(&hm01b0_flexio_callback_video, frameBuffer, frameBuffer2)) {
            Serial.println("Before Set frame complete CB");
            if (!tft.useFrameBuffer(true)) Serial.println("Failed call to useFrameBuffer");
            tft.setFrameCompleteCB(&frame_complete_cb, false);
            Serial.println("Before UPdateScreen Async");
            tft.updateScreenAsync(true);
            Serial.println("* continuous mode (Video) started");
            g_flexio_capture_count = 0;
            g_flexio_redraw_count = 0;
            g_continuous_flex_mode = 2;
          } else {
            Serial.println("* error, could not start continuous mode");
          }
        } else {
          hm01b0.stopReadContinuous();
          tft.endUpdateAsync();
          g_continuous_flex_mode = 0;
          Serial.println("* continuous mode stopped");
        }
        ch = ' ';
        break;
      }
      case '1':
      {
        tft.fillScreen(TFT_BLACK);
        break;
      }
      case 0x30:
      {
          Serial.println(F("ACK CMD CAM start single shoot. END"));
          send_image( &Serial );
          Serial.println(F("READY. END"));
          break;
      }
      case '?':
      {
        showCommandList();
        ch = ' ';
        break;
      }
      default:
        break;
    }
   while (Serial.read() != -1); // lets strip the rest out
   }


  if ( g_continuous_flex_mode ) {
    if (g_new_flexio_data) {
      //Serial.println("new FlexIO data");
      if (1) {
      tft.setOrigin(-2, -2);
      tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, (uint8_t *)g_new_flexio_data, mono_palette);
      tft.setOrigin(0, 0);
      tft.updateScreenAsync();
    }
      g_new_flexio_data = nullptr;
      g_flexio_redraw_count++;
      if (g_flexio_runtime > 10000) {
        // print some stats on actual speed, but not too much
        // printing too quickly to be considered "spew"
        //float redraw_rate = (float)g_flexio_redraw_count / (float)g_flexio_runtime * 1000.0f;
        g_flexio_runtime = 0;
        g_flexio_redraw_count = 0;
        //Serial.printf("redraw rate = %.2f Hz\t secs = %lu\t", redraw_rate, millis()/1000);
        //Serial.printf( "\tdeg  C=%u\n" , (uint32_t)tempmonGetTemp() );
      }
    }
  }
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


void send_image( Stream *imgSerial) {
  uint32_t imagesize;
  imagesize = (320 * 240 * 2);
  hm01b0.set_vflip(true);
  memset(frameBuffer, 0, sizeof(frameBuffer));
  hm01b0.set_mode(HIMAX_MODE_STREAMING_NFRAMES, 1);
  hm01b0.readFrame(frameBuffer);
  
  uint32_t image_idx = 0;
  uint32_t frame_idx = 0;

  for (uint32_t row = 0; row < 240; row++) {
    for (uint32_t col = 0; col < 320; col++) {
      frame_idx = (324 * (row + 2)) + col + 2;
      uint16_t framePixel = color565(frameBuffer[frame_idx], frameBuffer[frame_idx], frameBuffer[frame_idx]);
      sendImageBuf[image_idx++] = (framePixel) & 0xFF;
      sendImageBuf[image_idx++] = (framePixel >> 8) & 0xFF;
    }
  }
  
  imgSerial->write(0xFF);
  imgSerial->write(0xAA);

  imgSerial->write((const uint8_t *)&bmp_header, sizeof(bmp_header));

  imgSerial->write(sendImageBuf, imagesize);

  imgSerial->write(0xBB);
  imgSerial->write(0xCC);

  imgSerial->println(F("ACK CMD CAM Capture Done. END"));delay(50);

}

#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
void send_raw() {
  uint32_t imagesize;
  imagesize = (FRAME_WIDTH * FRAME_HEIGHT * 2);
  SerialUSB1.write(sendImageBuf, imagesize); // set Tools > USB Type to "Dual Serial"
}
#endif

char name[] = "9px_0000.bmp";       // filename convention (will auto-increment)
  DMAMEM unsigned char img[3 * 320*240];
void save_image_SD() {
  uint8_t r, g, b;
  uint32_t x, y;

  Serial.print("Writing BMP to SD CARD File: ");

  // if name exists, create new filename, SD.exists(filename)
  for (int i = 0; i < 10000; i++) {
    name[4] = (i / 1000) % 10 + '0'; // thousands place
    name[5] = (i / 100) % 10 + '0'; // hundreds
    name[6] = (i / 10) % 10 + '0';  // tens
    name[7] = i % 10 + '0';         // ones
    if (!SD.exists(name)) {
      Serial.println(name);
      file = SD.open(name, FILE_WRITE);
      break;
    }
  }

  uint16_t w = FRAME_WIDTH;
  uint16_t h = FRAME_HEIGHT;

  //unsigned char *img = NULL;
  // set fileSize (used in bmp header)
  int rowSize = 4 * ((3 * w + 3) / 4);  // how many bytes in the row (used to create padding)
  int fileSize = 54 + h * rowSize;      // headers (54 bytes) + pixel data

//  img = (unsigned char *)malloc(3 * w * h);

  for (int i = 0; i < w; i++)
  {
    for (int j = 0; j < h; j++)
    {
      //r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
      x = i; y = (h - 1) - j;

      r = frameBuffer[(x + y * w)]    ;
      g = frameBuffer[(x + y * w)]    ;
      b = frameBuffer[(x + y * w)]    ;

      img[(x + y * w) * 3 + 2] = (unsigned char)(r);
      img[(x + y * w) * 3 + 1] = (unsigned char)(g);
      img[(x + y * w) * 3 + 0] = (unsigned char)(b);
    }
  }

  // create padding (based on the number of pixels in a row
  unsigned char bmpPad[rowSize - 3 * w];
  for (int i = 0; i < (int)(sizeof(bmpPad)); i++) {      // fill with 0s
    bmpPad[i] = 0;
  }

  unsigned char bmpFileHeader[14] = {'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0};
  unsigned char bmpInfoHeader[40] = {40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24, 0};

  bmpFileHeader[ 2] = (unsigned char)(fileSize      );
  bmpFileHeader[ 3] = (unsigned char)(fileSize >>  8);
  bmpFileHeader[ 4] = (unsigned char)(fileSize >> 16);
  bmpFileHeader[ 5] = (unsigned char)(fileSize >> 24);

  bmpInfoHeader[ 4] = (unsigned char)(       w    );
  bmpInfoHeader[ 5] = (unsigned char)(       w >> 8);
  bmpInfoHeader[ 6] = (unsigned char)(       w >> 16);
  bmpInfoHeader[ 7] = (unsigned char)(       w >> 24);
  bmpInfoHeader[ 8] = (unsigned char)(       h    );
  bmpInfoHeader[ 9] = (unsigned char)(       h >> 8);
  bmpInfoHeader[10] = (unsigned char)(       h >> 16);
  bmpInfoHeader[11] = (unsigned char)(       h >> 24);

  // write the file (thanks forum!)
  file.write(bmpFileHeader, sizeof(bmpFileHeader));    // write file header
  file.write(bmpInfoHeader, sizeof(bmpInfoHeader));    // " info header

  for (int i = 0; i < h; i++) {                        // iterate image array
    file.write(img + (FRAME_WIDTH * (FRAME_HEIGHT - i - 1) * 3), 3 * FRAME_WIDTH);    // write px data
    file.write(bmpPad, (4 - (FRAME_WIDTH * 3) % 4) % 4);         // and padding as needed
  }
  //free(img);
  file.close();                                        // close file when done writing
  Serial.println("Done Writing BMP");
}

void showCommandList() {
  Serial.println("Send the 'f' character to read a frame using FlexIO (changes hardware setup!)");
  Serial.println("Send the 'F' to start/stop continuous using FlexIO (changes hardware setup!)");
  Serial.println("Send the 'V' character DMA to TFT async continuous  ...");
  Serial.println("Send the 'p' character to snapshot to PC on USB1");
  Serial.println("Send the 'b' character to save snapshot (BMP) to SD Card");
  Serial.println("Send the '1' character to blank the display");
  Serial.println("Send the 'z' character to send current screen BMP to SD");
  Serial.println();
}


void calAE() {
  // Calibrate Autoexposure
  Serial.println("Calibrating Auto Exposure...");
  memset((uint8_t*)frameBuffer, 0, sizeof(frameBuffer));
  if (hm01b0.cal_ae(10, frameBuffer, FRAME_WIDTH * FRAME_HEIGHT, &aecfg) != HM01B0_ERR_OK) {
    Serial.println("\tnot converged");
  } else {
    Serial.println("\tconverged!");
    hm01b0.cmdUpdate();
  }
}
