#include <stdint.h>
#include <stdio.h>
#include <stdint.h>
#include <stdio.h>
#include <SPI.h>
#include <Wire.h>

#include "HM01B0.h"
#include "HM01B0_regs.h"

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

#define BMPIMAGEOFFSET 66
const char bmp_header[BMPIMAGEOFFSET] PROGMEM =
{
  0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
  0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
  0x00, 0x00
};

HM01B0 hm01b0;
//#define USE_SPARKFUN 1
ae_cfg_t aecfg;

#define TFT_DC  1   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS  4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST  0  // "RX1" on left side of Sparkfun ML Carrieruint16_t colorWrite;
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
#include "ILI9341_t3n.h" // https://github.com/KurtE/ILI9341_t3n
ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST);
#define TFT_BLACK ILI9341_BLACK
#define TFT_YELLOW ILI9341_YELLOW
#define TFT_RED   ILI9341_RED
#define TFT_GREEN ILI9341_GREEN
#define TFT_BLUE  ILI9341_BLUE
#endif

uint16_t colorWriteArray[240][320] DMAMEM;
uint8_t  resized_buffer[32*32*3] DMAMEM;
#define CNN_IMG_SIZE 32

#include "arm_math.h"
#include "arm_nnexamples_cifar10_parameter.h"
#include "arm_nnexamples_cifar10_weights.h"
#include "arm_nnfunctions.h"

const char* cifar10_label[] = {"Plane", "Car", "Bird", "Cat", "Deer", "Dog", "Frog", "Horse", "Ship", "Truck"};
q7_t output_data[10]; //10-classes

// include the input and weights
static q7_t conv1_wt[CONV1_IM_CH * CONV1_KER_DIM * CONV1_KER_DIM * CONV1_OUT_CH] = CONV1_WT;
static q7_t conv1_bias[CONV1_OUT_CH] = CONV1_BIAS;

static q7_t conv2_wt[CONV2_IM_CH * CONV2_KER_DIM * CONV2_KER_DIM * CONV2_OUT_CH] = CONV2_WT;
static q7_t conv2_bias[CONV2_OUT_CH] = CONV2_BIAS;

static q7_t conv3_wt[CONV3_IM_CH * CONV3_KER_DIM * CONV3_KER_DIM * CONV3_OUT_CH] = CONV3_WT;
static q7_t conv3_bias[CONV3_OUT_CH] = CONV3_BIAS;

static q7_t ip1_wt[IP1_DIM * IP1_OUT] = IP1_WT;
static q7_t ip1_bias[IP1_OUT] = IP1_BIAS;

/* Here the image_data should be the raw uint8 type RGB image in [RGB, RGB, RGB ... RGB] format */
//uint8_t   image_data[CONV1_IM_CH * CONV1_IM_DIM * CONV1_IM_DIM] = IMG_DATA;
//q7_t      output_data[IP1_OUT];

//vector buffer: max(im2col buffer,average pool buffer, fully connected buffer)
q7_t      col_buffer[2 * 5 * 5 * 32 * 2];

q7_t      scratch_buffer[32 * 32 * 10 * 4];

//---------------------------------------------------

uint16_t FRAME_WIDTH, FRAME_HEIGHT;
uint8_t frameBuffer[(324) * 244];
uint8_t sendImageBuf[(324) * 244 * 2];
void * volatile g_new_flexio_data = nullptr;


void setup() {
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
  Serial.begin(921600);
  
  while (!Serial) ;
  Serial.println("HM01B0 Camera Test");
  Serial.println("------------------");

  hm01b0.init();
  delay(500);

  tft.fillScreen(TFT_BLACK);


  uint16_t ModelID;
  ModelID = hm01b0.get_modelid();
  if (ModelID == 0x01B0) {
    Serial.printf("SENSOR DETECTED :-) MODEL HM0%X\n", ModelID);
  } else {
    Serial.println("SENSOR NOT DETECTED! :-(");
    while (1) {}
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
#endif

  status = hm01b0.set_framesize(FRAMESIZE_QVGA);

  if (status != 0) {
    Serial.println("Settings failed to load");
    while (1) {}
  }
  hm01b0.set_framerate(30);  //15, 30, 60, 120

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
  hm01b0.set_auto_exposure(true, 1500);  //higher the setting the less saturaturation of whiteness
  hm01b0.cmdUpdate();  //only need after changing auto exposure settings

  hm01b0.set_mode(HIMAX_MODE_STREAMING, 0); // turn on, continuous streaming mode

  FRAME_HEIGHT = hm01b0.h;
  FRAME_WIDTH  = hm01b0.w;
  Serial.printf("ImageSize (w,h): %d, %d\n", hm01b0.w, hm01b0.h);

  showCommandList();
}


bool hm01b0_flexio_callback(void *pfb)
{
  //Serial.println("Flexio callback");
  g_new_flexio_data = pfb;
  return true;
}


  
  
void loop() {
  char ch;
  if (Serial.available()) {
    ch = Serial.read();
    switch (ch) {
    case 'f':
    {
      tft.useFrameBuffer(false);
      tft.fillScreen(TFT_BLACK);
      //calAE();
      Serial.println("Reading frame using FlexIO");
      memset((uint8_t*)frameBuffer, 0, sizeof(frameBuffer));
      hm01b0.set_mode(HIMAX_MODE_STREAMING_NFRAMES, 1);
      hm01b0.readFrameFlexIO(frameBuffer);
      Serial.println("Finished reading frame"); Serial.flush();
      tft.setOrigin(-2, -2);
      tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, frameBuffer, mono_palette);
      tft.setOrigin(0, 0);
      ch = ' ';
      break;
    }
    case '1':
    {
      tft.fillScreen(TFT_BLACK);
      break;
    }
    case 'N':   //Neural network
    {
      tft.useFrameBuffer(false);
      tft.fillScreen(TFT_BLACK);
      //calAE();
      Serial.println("Reading frame using FlexIO");
      memset((uint8_t*)frameBuffer, 0, sizeof(frameBuffer));
      hm01b0.set_mode(HIMAX_MODE_STREAMING_NFRAMES, 1);
      hm01b0.readFrameFlexIO(frameBuffer);
      uint32_t image_idx = 0;
      uint32_t frame_idx = 0;
    
      for (uint32_t row = 0; row < 240; row++) {
        for (uint32_t col = 0; col < 320; col++) {
          frame_idx = (324 * (row + 2)) + col + 2;
          uint16_t framePixel = color565(frameBuffer[frame_idx], frameBuffer[frame_idx], frameBuffer[frame_idx]);
          //sendImageBuf[image_idx++] = (framePixel) & 0xFF;
          //sendImageBuf[image_idx++] = (framePixel >> 8) & 0xFF;
          colorWriteArray[row][col] = framePixel;
        }
      }
      tft.fillScreen(TFT_BLACK);
      resize();
      nn();
      break;
    }
    case 0x30:
    {
        Serial.println(F("ACK CMD CAM start single shoot. END"));
        send_image();
        Serial.println(F("READY. END"));
        break;
    }
      default:
        showCommandList();
        break;
    }
   while (Serial.read() != -1); // lets strip the rest out
   }



  //bmpCapture();
  //delay(100);
  //tft.fillScreen(ILI9341_BLACK);
  //resize();
  //nn();
 }
 
 
 void resize()
{
  Serial.println("Resizing 320x240 Image");
  int factorW = 7;
  int factorH = 10;
  int W = 240, H = 320;
  int newW = (W/factorW);
  int newH = (H/factorH);
  uint16_t array_swap[32][32];

  tft.setOrigin(0,0);
  tft.fillScreen(TFT_BLUE);
  for(int i = 0; i < (newW-2); i++){
    for(int j = 0; j < newH; j++){
      uint16_t p = colorWriteArray[i*factorW][j*factorH];
      array_swap[i][j] = p;
      tft.drawPixel(j, i, array_swap[i][j]);
    }
  }

  int buffIdx = 0;
  for(int i = 0; i < CNN_IMG_SIZE; i++){
    for(int j = 0; j < CNN_IMG_SIZE; j++){
      resized_buffer[buffIdx] = (array_swap[i][j]>>8)&0x00F8;
      resized_buffer[buffIdx++] = (array_swap[i][j]>>3)&0x00FC;
      resized_buffer[buffIdx++] = (array_swap[i][j]>>3)&0x00F8;
      buffIdx++;
    }
  }
  tft.setOrigin(130,130);
  int h = 32, w = 32, row, col, buffidx=0;
  uint8_t rgb[3];
  for (row=0; row<32; row++) { // For each scanline...
    for (col=0; col<32; col++) { // For each pixel...
      //To read from Flash Memory, pgm_read_XXX is required.
      //Since image is stored as uint16_t, pgm_read_word is used as it uses 16bit address
      for(uint8_t i=0; i < 3; i++){
         rgb[i] = resized_buffer[buffidx+i];
         buffidx++;
      }
      //convert to color
      tft.drawPixel(col, row, CL(rgb[0], rgb[1], rgb[2]));
      //buffidx++;
    } // end pixel
  }

  tft.setOrigin(0,0);
}

int get_top_prediction(q7_t* predictions)
{
  int max_ind = 0;
  int max_val = -128;
  for(int i=0;i<10;i++) {
    if(max_val < predictions[i]) {
      max_val = predictions[i];
      max_ind = i;
    }
  }
  return max_ind;
}

void nn()
{
    // run neural network 
  q7_t     *img_buffer1 = scratch_buffer;
  q7_t     *img_buffer2 = img_buffer1 + 32 * 32 * 32;

  /* input pre-processing */
  int mean_data[3] = INPUT_MEAN_SHIFT;
  unsigned int scale_data[3] = INPUT_RIGHT_SHIFT;
  for (int i=0;i<32*32*3; i+=3) {
    img_buffer2[i] =   (q7_t)__SSAT( ((((int)resized_buffer[i]   - mean_data[0])<<7) + (0x1<<(scale_data[0]-1)))
                             >> scale_data[0], 8);
    img_buffer2[i+1] = (q7_t)__SSAT( ((((int)resized_buffer[i+1] - mean_data[1])<<7) + (0x1<<(scale_data[1]-1)))
                             >> scale_data[1], 8);
    img_buffer2[i+2] = (q7_t)__SSAT( ((((int)resized_buffer[i+2] - mean_data[2])<<7) + (0x1<<(scale_data[2]-1)))
                             >> scale_data[2], 8);
  }
  
  // conv1 img_buffer2 -> img_buffer1
  arm_convolve_HWC_q7_RGB(img_buffer2, CONV1_IM_DIM, CONV1_IM_CH, conv1_wt, CONV1_OUT_CH, CONV1_KER_DIM, CONV1_PADDING,
                          CONV1_STRIDE, conv1_bias, CONV1_BIAS_LSHIFT, CONV1_OUT_RSHIFT, img_buffer1, CONV1_OUT_DIM,
                          (q15_t *) col_buffer, NULL);

  arm_relu_q7(img_buffer1, CONV1_OUT_DIM * CONV1_OUT_DIM * CONV1_OUT_CH);

  // pool1 img_buffer1 -> img_buffer2
  arm_maxpool_q7_HWC(img_buffer1, CONV1_OUT_DIM, CONV1_OUT_CH, POOL1_KER_DIM,
                     POOL1_PADDING, POOL1_STRIDE, POOL1_OUT_DIM, NULL, img_buffer2);

  // conv2 img_buffer2 -> img_buffer1
  arm_convolve_HWC_q7_fast(img_buffer2, CONV2_IM_DIM, CONV2_IM_CH, conv2_wt, CONV2_OUT_CH, CONV2_KER_DIM,
                           CONV2_PADDING, CONV2_STRIDE, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, img_buffer1,
                           CONV2_OUT_DIM, (q15_t *) col_buffer, NULL);

  arm_relu_q7(img_buffer1, CONV2_OUT_DIM * CONV2_OUT_DIM * CONV2_OUT_CH);

  // pool2 img_buffer1 -> img_buffer2
  arm_maxpool_q7_HWC(img_buffer1, CONV2_OUT_DIM, CONV2_OUT_CH, POOL2_KER_DIM,
                     POOL2_PADDING, POOL2_STRIDE, POOL2_OUT_DIM, col_buffer, img_buffer2);

// conv3 img_buffer2 -> img_buffer1
  arm_convolve_HWC_q7_fast(img_buffer2, CONV3_IM_DIM, CONV3_IM_CH, conv3_wt, CONV3_OUT_CH, CONV3_KER_DIM,
                           CONV3_PADDING, CONV3_STRIDE, conv3_bias, CONV3_BIAS_LSHIFT, CONV3_OUT_RSHIFT, img_buffer1,
                           CONV3_OUT_DIM, (q15_t *) col_buffer, NULL);

  arm_relu_q7(img_buffer1, CONV3_OUT_DIM * CONV3_OUT_DIM * CONV3_OUT_CH);

  // pool3 img_buffer-> img_buffer2
  arm_maxpool_q7_HWC(img_buffer1, CONV3_OUT_DIM, CONV3_OUT_CH, POOL3_KER_DIM,
                     POOL3_PADDING, POOL3_STRIDE, POOL3_OUT_DIM, col_buffer, img_buffer2);

  arm_fully_connected_q7_opt(img_buffer2, ip1_wt, IP1_DIM, IP1_OUT, IP1_BIAS_LSHIFT, IP1_OUT_RSHIFT, ip1_bias,
                             output_data, (q15_t *) img_buffer1);

  arm_softmax_q7(output_data, 10, output_data);
  
  
    int top_ind = get_top_prediction(output_data);

    tft.print("Prediction: "); tft.println(cifar10_label[top_ind]);
    tft.print("  Confidence (%):"); 
    //Serial.println((output_data[top_ind]/127.0)*100.0);

  tft.setOrigin(0,0);
  for (int i = 0; i < 10; i++)
  {
    Serial.printf("%d: %d\n", i, output_data[i]);
    tft.print(output_data[i]); tft.print("     "); tft.println(i);
  } 
  
  //delay(1000);
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


DMAMEM unsigned char image[324*244];
void send_image() {
  uint32_t imagesize;
  imagesize = (320 * 240 * 2);
  hm01b0.set_vflip(true);
  memset(frameBuffer, 0, sizeof(frameBuffer));
  hm01b0.set_mode(HIMAX_MODE_STREAMING_NFRAMES, 1);
  hm01b0.readFrameFlexIO(frameBuffer);
  
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
  
  Serial.write(0xFF);
  Serial.write(0xAA);

  Serial.write((const uint8_t *)&bmp_header, sizeof(bmp_header));

  Serial.write(sendImageBuf, imagesize);

  Serial.write(0xBB);
  Serial.write(0xCC);

  Serial.println(F("ACK CMD CAM Capture Done. END"));delay(50);

}

void showCommandList() {
  Serial.println("Send the 'f' character to read a frame using FlexIO (changes hardware setup!)");
  Serial.println("Send the '1' character to blank the display");
  Serial.println("Send the 'N' character to run CMSISNN CIFAR10 Example");
  Serial.println();
}
