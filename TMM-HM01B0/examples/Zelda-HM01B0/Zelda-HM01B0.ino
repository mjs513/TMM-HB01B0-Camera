/* Play Zelda music, Overland Theme from The Wind Walker

   Reference for this music:
   https://www.youtube.com/watch?v=RekBzP114do&t=1091s

   Requires Audio Shield: https://www.pjrc.com/store/teensy3_audio.html
*/

#include <Bounce.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <stdint.h>
#include "HM01B0.h"
#include "HM01B0_regs.h"
#include "TeensyThreads.h"

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


#define _hmConfig 1 // select mode string below

PROGMEM const char hmConfig[][48] = {
 "HM01B0_SPARKFUN_ML_CARRIER",
 "HM01B0_PJRC_CARRIER",
 "HM01B0_FLEXIO_CUSTOM_LIKE_8_BIT",
 "HM01B0_FLEXIO_CUSTOM_LIKE_4_BIT"
};
#if _hmConfig ==0
HM01B0 hm01b0(HM01B0_SPARKFUN_ML_CARRIER);
#elif _hmConfig == 1
HM01B0 hm01b0(HM01B0_PJRC_CARRIER);
#elif _hmConfig == 2
// We are doing manual settings: 
// this one should duplicate the 8 bit ML Carrier:
//    HM01B0(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, en_pin,
//    uint8_t g0, uint8_t g1,uint8_t g2, uint8_t g3,
//    uint8_t g4=0xff, uint8_t g5=0xff,uint8_t g6=0xff,uint8_t g7=0xff, TwoWire &wire=Wire);
HM01B0 hm01b0(7, 8, 33, 32, 2, 40, 41, 42, 43, 44, 45, 6, 9);

#elif _hmConfig == 3
// We are doing manual settings: 
// this one should duplicate the 8 bit ML Carrier:
//    HM01B0(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, en_pin,
//    uint8_t g0, uint8_t g1,uint8_t g2, uint8_t g3,
//    uint8_t g4=0xff, uint8_t g5=0xff,uint8_t g6=0xff,uint8_t g7=0xff, TwoWire &wire=Wire);
HM01B0 hm01b0(7, 8, 33, 32, 2, 40, 41, 42, 43);
#endif


//#define USE_SPARKFUN 1
//#define USE_SDCARD 1
File file;

#define MMOD_ML 0
#if MMOD_ML==1
#define TFT_DC  1   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS  4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST 0  // "RX1" on left side of Sparkfun ML Carrier
#else // PJRC_BREAKOUT
#define TFT_DC  9
#define TFT_CS  10
#define TFT_RST 255  // none
#endif

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
uint8_t sendImageBuf[(324) * 244 * 2];
uint8_t frameBuffer2[(324) * 244] DMAMEM;

bool g_continuous_mode = false;
uint8_t g_continuous_flex_mode = 0; // 0 no, 1=F command, 2= V command
void * volatile g_new_flexio_data = nullptr;
uint32_t g_flexio_capture_count = 0;
uint32_t g_flexio_redraw_count = 0;
elapsedMillis g_flexio_runtime;
bool g_dma_mode = false;

ae_cfg_t aecfg;

#include "Pizzicato_samples.h"
#include "Viola_samples.h"
#include "WT_Trumpet_samples.h"
#include "MutedTrumpet_samples.h"
#include "PlaySynthMusic.h"

//#define DEBUG_ALLOC

unsigned char *sp = score;

const int TOTAL_VOICES = 64;
const int TOTAL_MIXERS = 21;
const int SECONDARY_MIXERS = 4;

AudioControlSGTL5000 sgtl5000_1;
AudioSynthWavetable wavetable[TOTAL_VOICES];
AudioMixer4 mixer[TOTAL_MIXERS];
AudioOutputI2S i2s1;
AudioConnection patchCord[] = {
	{wavetable[ 0], 0, mixer[ 0], 0}, {wavetable[ 1], 0, mixer[ 0], 1}, {wavetable[ 2], 0, mixer[0],  2}, {wavetable[ 3], 0, mixer[0],  3}, {mixer[ 0], 0, mixer[TOTAL_MIXERS-2], 0},
	{wavetable[ 4], 0, mixer[ 1], 0}, {wavetable[ 5], 0, mixer[ 1], 1}, {wavetable[ 6], 0, mixer[1],  2}, {wavetable[ 7], 0, mixer[1],  3}, {mixer[ 1], 0, mixer[TOTAL_MIXERS-2], 1},
	{wavetable[ 8], 0, mixer[ 2], 0}, {wavetable[ 9], 0, mixer[ 2], 1}, {wavetable[10], 0, mixer[2],  2}, {wavetable[11], 0, mixer[2],  3}, {mixer[ 2], 0, mixer[TOTAL_MIXERS-2], 2},
	{wavetable[12], 0, mixer[ 3], 0}, {wavetable[13], 0, mixer[ 3], 1}, {wavetable[14], 0, mixer[3],  2}, {wavetable[15], 0, mixer[3],  3}, {mixer[ 3], 0, mixer[TOTAL_MIXERS-2], 3},
	{wavetable[16], 0, mixer[ 4], 0}, {wavetable[17], 0, mixer[ 4], 1}, {wavetable[18], 0, mixer[4],  2}, {wavetable[19], 0, mixer[4],  3}, {mixer[ 4], 0, mixer[TOTAL_MIXERS-3], 0},
	{wavetable[20], 0, mixer[ 5], 0}, {wavetable[21], 0, mixer[ 5], 1}, {wavetable[22], 0, mixer[5],  2}, {wavetable[23], 0, mixer[5],  3}, {mixer[ 5], 0, mixer[TOTAL_MIXERS-3], 1},
	{wavetable[24], 0, mixer[ 6], 0}, {wavetable[25], 0, mixer[ 6], 1}, {wavetable[26], 0, mixer[6],  2}, {wavetable[27], 0, mixer[6],  3}, {mixer[ 6], 0, mixer[TOTAL_MIXERS-3], 2},
	{wavetable[28], 0, mixer[ 7], 0}, {wavetable[29], 0, mixer[ 7], 1}, {wavetable[30], 0, mixer[7],  2}, {wavetable[31], 0, mixer[7],  3}, {mixer[ 7], 0, mixer[TOTAL_MIXERS-3], 3},
	{wavetable[32], 0, mixer[ 8], 0}, {wavetable[33], 0, mixer[ 8], 1}, {wavetable[34], 0, mixer[8],  2}, {wavetable[35], 0, mixer[8],  3}, {mixer[ 8], 0, mixer[TOTAL_MIXERS-4], 0},
	{wavetable[36], 0, mixer[ 9], 0}, {wavetable[37], 0, mixer[ 9], 1}, {wavetable[38], 0, mixer[9],  2}, {wavetable[39], 0, mixer[9],  3}, {mixer[ 9], 0, mixer[TOTAL_MIXERS-4], 1},
	{wavetable[40], 0, mixer[10], 0}, {wavetable[41], 0, mixer[10], 1}, {wavetable[42], 0, mixer[10], 2}, {wavetable[43], 0, mixer[10], 3}, {mixer[10], 0, mixer[TOTAL_MIXERS-4], 2},
	{wavetable[44], 0, mixer[11], 0}, {wavetable[45], 0, mixer[11], 1}, {wavetable[46], 0, mixer[11], 2}, {wavetable[47], 0, mixer[11], 3}, {mixer[11], 0, mixer[TOTAL_MIXERS-4], 3},
	{wavetable[48], 0, mixer[12], 0}, {wavetable[49], 0, mixer[12], 1}, {wavetable[50], 0, mixer[12], 2}, {wavetable[51], 0, mixer[12], 3}, {mixer[12], 0, mixer[TOTAL_MIXERS-5], 0},
	{wavetable[52], 0, mixer[13], 0}, {wavetable[53], 0, mixer[13], 1}, {wavetable[54], 0, mixer[13], 2}, {wavetable[55], 0, mixer[13], 3}, {mixer[13], 0, mixer[TOTAL_MIXERS-5], 1},
	{wavetable[56], 0, mixer[14], 0}, {wavetable[57], 0, mixer[14], 1}, {wavetable[58], 0, mixer[14], 2}, {wavetable[59], 0, mixer[14], 3}, {mixer[14], 0, mixer[TOTAL_MIXERS-5], 2},
	{wavetable[60], 0, mixer[15], 0}, {wavetable[61], 0, mixer[15], 1}, {wavetable[62], 0, mixer[15], 2}, {wavetable[63], 0, mixer[15], 3}, {mixer[15], 0, mixer[TOTAL_MIXERS-5], 3},
	{mixer[TOTAL_MIXERS-2], 0, mixer[TOTAL_MIXERS-1], 0},
	{mixer[TOTAL_MIXERS-3], 0, mixer[TOTAL_MIXERS-1], 1},
	{mixer[TOTAL_MIXERS-4], 0, mixer[TOTAL_MIXERS-1], 2},
	{mixer[TOTAL_MIXERS-5], 0, mixer[TOTAL_MIXERS-1], 3},
	{mixer[TOTAL_MIXERS-1], 0, i2s1, 0},
	{mixer[TOTAL_MIXERS-1], 0, i2s1, 1},
};
Bounce buttons[] = { {0, 15}, {1, 15}, {2, 15}, };
const int TOTAL_BUTTONS = sizeof(buttons) / sizeof(Bounce);

void guitarHeroMode();
void printVoices();
void setVolume() {
	sgtl5000_1.volume(0.8*(analogRead(PIN_A2)-1)/1022.0);
}

struct voice_t {
	int wavetable_id;
	byte channel;
	byte note;
};
voice_t voices[TOTAL_VOICES];

IntervalTimer midiMapTimer;
IntervalTimer guitarHeroTimer;
IntervalTimer volumeTimer;

void setup() {
#ifdef TFT_ILI9341
  tft.begin();
#else
  tft.init(240, 320);           // Init ST7789 320x240
#endif
  tft.setRotation(3);
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
  #if MMOD_ML==1
    if (!SD.begin(10)) {
  #else
    if (!SD.begin(BUILTIN_SDCARD)) {
  #endif
    }
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
  Serial.println( hmConfig[_hmConfig] );
  Serial.println("------------------");

  delay(500);

  tft.fillScreen(TFT_BLACK);


  uint16_t ModelID;
  ModelID = hm01b0.get_modelid();
  if (ModelID == 0x01B0) {
    Serial.printf("SENSOR DETECTED :-) MODEL HM0%X\n", ModelID);
  } else {
    Serial.printf("SENSOR NOT DETECTED! :-(  MODEL HM0%X\n", ModelID);
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

  if(_hmConfig == 1 || _hmConfig == 3){
    status = hm01b0.set_framesize(FRAMESIZE_QVGA4BIT);
  } else {
    status = hm01b0.set_framesize(FRAMESIZE_QVGA);
  }
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
  hm01b0.set_autoExposure(true, 1500);  //higher the setting the less saturaturation of whiteness
  hm01b0.cmdUpdate();  //only need after changing auto exposure settings

  hm01b0.set_mode(HIMAX_MODE_STREAMING, 0); // turn on, continuous streaming mode

  FRAME_HEIGHT = hm01b0.height();
  FRAME_WIDTH  = hm01b0.width();
  Serial.printf("ImageSize (w,h): %d, %d\n", FRAME_WIDTH, FRAME_HEIGHT);

  showCommandList();


	pinMode(0, INPUT_PULLUP);
	pinMode(1, INPUT_PULLUP);
	pinMode(2, INPUT_PULLUP);

	AudioMemory(120);

	sgtl5000_1.enable();
	sgtl5000_1.volume(0.8);

	for (int i = 0; i < TOTAL_VOICES; ++i) {
		wavetable[i].setInstrument(Pizzicato);
		wavetable[i].amplitude(1);
		voices[i].wavetable_id = i;
		voices[i].channel = voices[i].note = 0xFF;
	}

	for (int i = 0; i < TOTAL_MIXERS-1; ++i)
		for (int j = 0; j < 4; ++j)
			mixer[i].gain(j, 0.25);
	for (int i = 0; i < 4; ++i)
		mixer[TOTAL_MIXERS - 1].gain(i, 0.5);
	
	//usbMIDI.setHandleNoteOn(OnNoteOn);
	//usbMIDI.setHandleNoteOff(OnNoteOff);
	//volumeTimer.begin(setVolume, 100000);
	//guitarHeroTimer.begin(guitarHeroMode, 1000000/120);
	//midiMapTimer.begin(printVoices, 5000);

  //int cam = threads.addThread(camLoop);
  //threads.setTimeSlice(0, 1);
  //threads.setTimeSlice(0,5);

  delay(2000);
}

void loop() {
  unsigned char c,opcode,chan;
  unsigned long d_time;
  
  // read the next note from the table
  c = *sp++;
  opcode = c & 0xF0;
  chan = c & 0x0F;

  if(c < 0x80) {
    // Delay
    d_time = (c << 8) | *sp++;
    delay(d_time);
    return;
  }
  if(*sp == CMD_STOP) {
    for (int i=0; i<TOTAL_VOICES; i++) {
      wavetable[chan].stop();
    }
    while(1);
  }

  // It is a command

  //Change the instrument for generator
  if(opcode == CMD_CHANGEINST) {
    unsigned char inst = *sp++;
    wavetable[chan].stop();
    switch(inst) {
      case 48:
        wavetable[chan].setInstrument(Viola);
        break;
      case 57:
        wavetable[chan].setInstrument(WT_Trumpet);
        break;
      default:
        wavetable[chan].setInstrument(MutedTrumpet);
        break;
    }
    return;
  }
  
  // Stop the note on 'chan'
  if(opcode == CMD_STOPNOTE) {
    wavetable[chan].stop();
    return;
  }
  
  // Play the note on 'chan'
  if(opcode == CMD_PLAYNOTE) {
    unsigned char note = *sp++;
    unsigned char velocity = *sp++;
    wavetable[chan].playNote((byte)note);
    //OnNoteOn(chan, (byte)note, (byte)velocity);
    camLoop();
    return;
  }

  // replay the tune
  if(opcode == CMD_RESTART) {
    sp = score;
    return;
  }

	//usbMIDI.read();
	//for (int i = 0; i < TOTAL_BUTTONS; ++i) buttons[i].update();
	//if (buttons[0].fallingEdge()) AudioSynthWavetable::print_performance();
	//if (buttons[1].risingEdge()) {
	//	midiMapTimer.end();
	//	Serial.print('\n');
	//}
	//if (buttons[1].fallingEdge()) midiMapTimer.begin(printVoices, 5000);
	//if (buttons[2].risingEdge()) guitarHeroTimer.end();
	//if (buttons[2].fallingEdge())
	//	guitarHeroTimer.begin(guitarHeroMode, 1000000/60);

}

int allocateVoice(byte channel, byte note);
int findVoice(byte channel, byte note);
void freeVoices();

int used_voices = 0;
int stopped_voices = 0;
int evict_voice = 0;
int notes_played = 0;

void OnNoteOn(byte channel, byte note, byte velocity) {
	notes_played++;
#ifdef DEBUG_ALLOC
	//Serial.printf("**** NoteOn: channel==%hhu,note==%hhu ****\n", channel, note);
	printVoices();
#endif //DEBUG_ALLOC
	freeVoices();
	int wavetable_id = allocateVoice(channel, note);
  switch (channel) {
    case 1:
      wavetable[wavetable_id].setInstrument(Pizzicato);
      break;
    case 2:
      wavetable[wavetable_id].setInstrument(Pizzicato);
      break;
    case 3:
      wavetable[wavetable_id].setInstrument(Pizzicato);
      break;
    case 4:
      wavetable[wavetable_id].setInstrument(Pizzicato);
      break;
    case 5:
      wavetable[wavetable_id].setInstrument(Pizzicato);
      break;
    default:
      wavetable[wavetable_id].setInstrument(Pizzicato);
      break;
  }
	wavetable[wavetable_id].playNote(note);
#ifdef DEBUG_ALLOC
	printVoices();
#endif //DEBUG_ALLOC
}

void OnNoteOff(byte channel, byte note, byte velocity) {
#ifdef DEBUG_ALLOC
	//Serial.printf("\n**** NoteOff: channel==%hhu,note==%hhu ****", channel, note);
	printVoices();
#endif //DEBUG_ALLOC
	int wavetable_id = findVoice(channel, note);
	if (wavetable_id != TOTAL_VOICES)
		wavetable[wavetable_id].stop();
#ifdef DEBUG_ALLOC
	printVoices();
#endif //DEBUG_ALLOC
}

int allocateVoice(byte channel, byte note) {
	int i;
	int nonfree_voices = stopped_voices + used_voices;
	if (nonfree_voices < TOTAL_VOICES) {
		for (i = nonfree_voices; i < TOTAL_VOICES && voices[i].channel != channel; ++i);
		if (i < TOTAL_VOICES) {
			voice_t temp = voices[i];
			voices[i] = voices[nonfree_voices];
			voices[nonfree_voices] = temp;
		}
		i = nonfree_voices;
		used_voices++;
	} else {
		if (stopped_voices) {
			i = evict_voice % stopped_voices;
			voice_t temp = voices[i];
			stopped_voices--;
			voices[i] = voices[stopped_voices];
			voices[stopped_voices] = temp;
			used_voices++;
			i = stopped_voices;
		}
		else
			i = evict_voice;
	}

	voices[i].channel = channel;
	voices[i].note = note;

	evict_voice++;
	evict_voice %= TOTAL_VOICES;

	return voices[i].wavetable_id;
}

int findVoice(byte channel, byte note) {
	int i;
	//find match
	int nonfree_voices = stopped_voices + used_voices;
	for (i = stopped_voices; i < nonfree_voices && !(voices[i].channel == channel && voices[i].note == note); ++i);
	//return TOTAL_VOICES if no match
	if (i == (nonfree_voices)) return TOTAL_VOICES;

	voice_t temp = voices[i];
	voices[i] = voices[stopped_voices];
	voices[stopped_voices] = temp;
	--used_voices;

	return voices[stopped_voices++].wavetable_id;
}

void freeVoices() {
	for (int i = 0; i < stopped_voices; i++)
		if (wavetable[voices[i].wavetable_id].isPlaying() == false) {
			voice_t temp = voices[i];
			--stopped_voices;
			voices[i] = voices[stopped_voices];
			int nonfree_voices = stopped_voices + used_voices;
			voices[stopped_voices] = voices[nonfree_voices];
			voices[nonfree_voices] = temp;
		}
}

void guitarHeroMode() { // now unicorn friendly
	const int RESET = 4;
	const int MIDI_NOTES = 128;
	static char line[MIDI_NOTES+1] = { 0 };
	static int accumulated = 0;
	if (!accumulated) {
		for (int i = 0; i < MIDI_NOTES; ++i) line[i] = '-';
		++accumulated;
	}
	for (int i = stopped_voices; i < used_voices+stopped_voices; ++i) line[voices[i].note] = '*';
	if (accumulated == RESET) {
		Serial.println(line);
		accumulated = 0;
	} else {
		++accumulated;
	}
}

const char* note_map[] = {
	"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
};

void printVoices() {
	static int last_notes_played = notes_played;
	if (last_notes_played == notes_played)
		return;
	last_notes_played = notes_played;
	int usage = AudioProcessorUsage();
	Serial.printf("\nCPU:%03i voices:%02i CPU/Voice:%02i evict:%02i", usage, used_voices, usage/used_voices, evict_voice);
	for (int i = 0; i < used_voices; ++i)
		Serial.printf(" %02hhu %-2s", voices[i].channel, note_map[voices[i].note%12]);

}
