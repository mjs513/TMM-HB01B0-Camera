uint8_t *last_dma_frame_buffer = nullptr;
uint8_t *image_buffer_display = sendImageBuf; // BUGBUG using from somewhere else for now...

bool hm01b0_dma_callback(void *pfb) {
  //Serial.printf("Callback: %x\n", (uint32_t)pfb);
  if (tft.asyncUpdateActive()) return false; // don't use if we are already busy
  tft.setOrigin(-2, -2);
  tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, (uint8_t*)pfb, mono_palette);
  tft.setOrigin(0, 0);
  tft.updateScreenAsync();

  last_dma_frame_buffer = (uint8_t*)pfb;
  return true;
}

bool hm01b0_flexio_callback(void *pfb)
{
  //Serial.println("Flexio callback");
  static uint8_t callback_count = 0;
    Serial.print("#");
    callback_count++;
    if (!(callback_count & 0x3f)) Serial.println();
  g_new_flexio_data = pfb;
  return true;
}

bool hm01b0_flexio_callback_video(void *pfb)
{
  //Serial.println("Flexio callback");
  static uint8_t callback_count = 0;
    Serial.print("#");
    callback_count++;
    if (!(callback_count & 0x3f)) Serial.println();
  tft.setOrigin(-2, -2);
  tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, (uint8_t*)pfb, mono_palette);
  tft.setOrigin(0, 0);
  return true;
}

bool hm01b0_dma_callback_video(void *pfb) {
  // just remember the last frame given to us. 
  //Serial.printf("Callback: %x\n", (uint32_t)pfb);
  if (tft.asyncUpdateActive()) return false; // don't use if we are already busy
  tft.setOrigin(-2, -2);
  tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, (uint8_t*)pfb, mono_palette);
  tft.setOrigin(0, 0);
  //tft.updateScreenAsync();
  last_dma_frame_buffer = (uint8_t*)pfb;
  return true;
}

void tft_frame_cb() {
  tft.setOrigin(-2, -2);
  if (tft.subFrameCount()) {
    // so finished drawing the top half of the display
    tft.setClipRect(0, 0, tft.width(), tft.height() / 2);
    tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, (uint8_t*)image_buffer_display, mono_palette);
    // Lets play with buffers here. 
  } else {
    tft.setClipRect(0, tft.height() / 2, tft.width(), tft.height() / 2);      
    tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, (uint8_t*)image_buffer_display, mono_palette);
    if (last_dma_frame_buffer) {
        hm01b0.changeFrameBuffer(last_dma_frame_buffer, image_buffer_display);
        image_buffer_display = last_dma_frame_buffer;
        last_dma_frame_buffer = nullptr;
    }
  }
  tft.setOrigin(0, 0);
  tft.setClipRect();
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

DMAMEM unsigned char image[324*244];
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
  Serial.println("Send the 'p' character to snapshot to PC on USB1");
  Serial.println("Send the 'b' character to save snapshot (BMP) to SD Card");
  Serial.println("Send the '1' character to blank the display");
  Serial.println("Send the 'z' character to send current screen BMP to SD");
  Serial.println("Send the 'V' character DMA to TFT async continueous  ...");

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

void camLoop() {
  //while(1) {
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
        g_continuous_mode = false;
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
        if (g_dma_mode) {
          Serial.println("Must stop Video first!");
          break;
        }
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
        g_continuous_flex_mode = 0;
        break;
      }
      case 'F':
      {
        if (!g_continuous_flex_mode) {
          if (hm01b0.readContinuous(&hm01b0_flexio_callback, frameBuffer, frameBuffer2)) {
            Serial.println("* continuous mode started");
            g_flexio_capture_count = 0;
            g_flexio_redraw_count = 0;
            g_continuous_flex_mode = 1;
          } else {
            Serial.println("* error, could not start continuous mode");
          }
        } else {
          hm01b0.stopReadContinuous();
          tft.endUpdateAsync();
          g_continuous_flex_mode = 0;
          Serial.println("* continuous mode stopped");
        }
        break;
      }
      case 'V':
      {
        if (!g_continuous_flex_mode) {
          if (hm01b0.readContinuous(&hm01b0_flexio_callback_video, frameBuffer, frameBuffer2)) {
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


  if ( g_continuous_flex_mode == 1 ) {
    if (g_new_flexio_data) {
      //Serial.println("new FlexIO data");
      if (tft.asyncUpdateActive()) {
        Serial.print("!");
      } else {
        Serial.print("%");
        tft.setOrigin(-2, -2);
        tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, (uint8_t *)g_new_flexio_data, mono_palette);
        tft.setOrigin(0, 0);
        tft.updateScreenAsync();
        g_new_flexio_data = nullptr;
        g_flexio_redraw_count++;
        if (g_flexio_runtime > 10000) {
          // print some stats on actual speed, but not too much
          // printing too quickly to be considered "spew"
          float redraw_rate = (float)g_flexio_redraw_count / (float)g_flexio_runtime * 1000.0f;
          g_flexio_runtime = 0;
          g_flexio_redraw_count = 0;
          Serial.printf("redraw rate = %.2f Hz\n", redraw_rate);
        }
      }
    }
  }

    //threads.yield();
  //}
}
