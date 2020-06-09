
/* 
 Extension to detect movement in sequential images using centre of mass shift.
 This technique reduces spurious motion changes:
 - camera noise, particularly in low light
 - micro movements, eg leaves rustling, rain
 - transient movements, eg bird flying past
 - changes in illumination levels, eg passing cloud

 The amount of change between images will depend on the frame rate.
 A faster frame rate will need a higher sensitivity

 When frame size is changed the OV2640 outputs a few glitched frames whilst it 
 makes the transition. These could be interpreted as spurious motion.
 
 s60sc 2020
 
*/
#include "motion.h"
#include "esp_camera.h"
#include "esp_jpg_decode.h"
#include <SD_MMC.h>

#include "math.h"


// user configuration parameters for calibrating motion detection
#define MOTION_SEQUENCE 5 // min sequence of changed frames to confirm motion 
#define NIGHT_SEQUENCE 10 // frames of sequential darkness to avoid spurious day / night switching
#define WANT_BMP false // for debugging, true to add BMP header, false for none

// constants
#define RGB888_BYTES 3 // number of bytes per pixel
#define BMP_HEADER 54 // size of BMP header bytes

// auto newline printf
#define SerialPrintf(format, ...) {\
    Serial.printf(format, ##__VA_ARGS__);\
    Serial.println("");\
    } 
#define showInfo(format, ...) SerialPrintf(format, ##__VA_ARGS__)
#define showError(format, ...) SerialPrintf("ERROR: " format, ##__VA_ARGS__)
#define showDebug(format, ...) if (debug) SerialPrintf("DEBUG: " format, ##__VA_ARGS__)

/********* the following must be declared and initialised elsewhere **********/
extern bool debug;
extern uint8_t fsizePtr;
extern uint8_t lightLevel; // Current ambient light level 
extern uint8_t motionVal; // motion sensitivity setting

// 1600>>2 = 400 ; 1200 >> 2 = 300
// TODO : be able to allocate more  : 400 x 300
static uint8_t PROGMEM prevGrayIm[43000] = {0};
static MotionEstContext* me_ctx = new MotionEstContext();
static bool first = true;

struct frameStruct {
  const char* frameSizeStr;
  const uint16_t frameWidth;
  const uint16_t frameHeight;
  const uint16_t defaultFPS;
  const uint8_t scaleFactor;
  const uint8_t sampleRate;
};
extern const frameStruct frameData[];

/**********************************************************************************/

static bool jpg2rgb(const uint8_t *src, size_t src_len, uint8_t ** out, size_t * out_len, uint8_t scale, uint8_t bmpOffset);

static void bmp2gray(uint8_t *gray, uint8_t *bmpBuf, size_t numCols, size_t numRows,
        int pixelSpan, int bitmapWidth) {
    for (int r=0; r<numRows; r++) {
      for (int c=0; c<numCols; c++) {
        *(gray++) = (19595 * *(bmpBuf)            // R 
                   + 38469 * *(bmpBuf + 1)         // G
                    + 7472 * *(bmpBuf + 2)) >> 16;  // B
        bmpBuf += pixelSpan;
      }  
      // same as pixelPos = (c*pixelSpan+r*bitmapWidth*pixelSpan)
      bmpBuf += bitmapWidth * pixelSpan;  // r*bitmapWidth + c
    }
}

bool checkMotion(camera_fb_t* fb, bool motionStatus) {
  // get current frame and calculate centre of mass, then compare with previous
  uint32_t mTime = millis();
  // convert jpeg to downsized bitmap
  uint8_t* bmpBuf = NULL;
  size_t buf_len = 0;
  uint8_t bmpOffset = WANT_BMP ? BMP_HEADER : 0; // size of BMP header if wanted
  bool converted = jpg2rgb((uint8_t*)fb->buf, fb->len, &bmpBuf, &buf_len, frameData[fsizePtr].scaleFactor, bmpOffset);
  if (debug && bmpOffset) { 
    // for debug purposes, store on SD as BMP to check that bitmap is properly formed
    File bmpFile = SD_MMC.open("/test.bmp", FILE_WRITE);
    bmpFile.write(bmpBuf, buf_len); 
    bmpFile.close(); 
    showDebug("wrote BMP to SD");
  }
  if (converted) {       
    showDebug("Jpeg to bitmap conversion time %ums", (uint32_t)millis()-mTime); 
    uint32_t cTime = millis(); 
    static uint16_t motionCnt = 0;
    if(!bmpBuf) {
      showError("failed jpg2rgb!")
      return false;
    }
    // calculate parameters for sample size
    uint8_t log2_downsize = frameData[fsizePtr].scaleFactor + log2(frameData[fsizePtr].sampleRate);
    uint8_t numCols = frameData[fsizePtr].frameWidth >> log2_downsize; 
    uint8_t numRows = frameData[fsizePtr].frameHeight >> log2_downsize;
    uint16_t pixelSpan = frameData[fsizePtr].sampleRate * RGB888_BYTES;
    uint16_t bitmapWidth = numCols * frameData[fsizePtr].sampleRate;
    uint32_t numSamples = numCols * numRows;
    showInfo("%u samples for %s", numSamples, frameData[fsizePtr].frameSizeStr);
    uint8_t *grayIm = (uint8_t*)ps_calloc(numSamples,1);
    showDebug("image (%u x %u)  pixelSpan: %u, bitmapWidth: %u", numCols, numRows, pixelSpan, bitmapWidth);    
    showDebug("* esp32-motion:");
    showDebug("    - convert to grayscale..");

    bmpBuf += bmpOffset; 
    bmp2gray(grayIm, bmpBuf, numCols, numRows, pixelSpan, bitmapWidth);

    // Init motion context at start or when framesize change
    // In that case don't do motion_estimation because no previous
    // image or image of diff size
    if(first || numCols != me_ctx->width || numRows != me_ctx->height) {
      me_ctx->method = BLOCK_MATCHING_EPZS;            // algo used. LK_OPTICAL_FLOW; <- good too
      me_ctx->width = numCols;                         // image width
      me_ctx->height = numRows;                        // image height
      me_ctx->mbSize = 6;                              // MacroBlock size higher to remove noise, leaf motion,..
      me_ctx->search_param = 9;                        // Higher value for faster movement => more research => slower
      showDebug("    - init motion context ...")
      init_context(me_ctx);                            // allocate mv_tables
      first = false;
      showDebug("Total heap: %d", ESP.getHeapSize());
      showDebug("Free heap: %d", ESP.getFreeHeap());
      showDebug("Total PSRAM: %d", ESP.getPsramSize());
      showDebug("Free PSRAM: %d", ESP.getFreePsram());  
    //in other cases Compute & save motion vectors in mv_tables[0]
    } else if(!motion_estimation(me_ctx, grayIm, &prevGrayIm[0]))
      showError("Motion estimation failed!");

    showDebug("    - copy image buffer for next motion")
    // Copy current image for next process as previous image
    for (int i = numSamples; --i; ) 
        prevGrayIm[i] = grayIm[i];
    free(grayIm);  

    showDebug("    - calculating change.." );
    // Calculate mean/sum of vector mag2
    int sum = 0;
    MotionVector16_t *mv = me_ctx->mv_table[0];
    // number of motion vector is w*h if LK else number of macroblock.
    int nb_mv = (me_ctx->method == LK_OPTICAL_FLOW) ? numSamples : me_ctx->b_count;
    for(int i = 0; i < nb_mv; i++, mv++) 
        sum += (int) sqrtf(mv->mag2);

    showInfo("time: %ums | tot change = %u pix | avg change = %u pix/vector", (uint32_t)millis()-cTime, 
    (uint32_t)sum, (uint32_t)sum / nb_mv); 
      
    // determine if movement has occurred
    if (sum / nb_mv > 10 - motionVal) { 
      // sufficient centre of mass shift
      if (!motionStatus) motionCnt += 1;
      showInfo("### Change detected");
      // need minimum sequence of changes to signal valid movement
      if (!motionStatus && motionCnt >= MOTION_SEQUENCE) {
        showInfo("***** Motion - START");
        motionStatus = true; // motion started
      } 
    } else {
      // insufficient change
      if (motionStatus) {
        motionCnt = 0;
        showDebug("***** Motion - STOP");
        motionStatus = false; // motion stopped
      }
    }
    if (motionStatus) showDebug("*** Motion - ongoing");
    showDebug("Total motion processing for frame %ums", (uint32_t)millis()-mTime);
  } else showError("Image conversion failed");

  // esp32-cam issue #126
  if (bmpBuf == NULL) showError("Memory leak, heap now: %u", (uint32_t)xPortGetFreeHeapSize());
  free(bmpBuf);
  bmpBuf = NULL;
  return motionStatus;
}

bool isNight(uint8_t nightSwitch) {
  // check if night time for switching on lamp during recording
  static bool nightTime = false;
  static uint16_t nightCnt = 0;
  if (!nightTime && lightLevel < nightSwitch) {
    // dark image
    nightCnt += 1;
    // only signal night time after given sequence of dark frames
    if (nightCnt > NIGHT_SEQUENCE) {
      nightTime = true;     
      showInfo("Night time"); 
    }
  } 
  if (lightLevel > nightSwitch) {
    nightCnt = 0;
    if (nightTime) {
      nightTime = false;
      showInfo("Day time");
    }
  }  
  return nightTime;
}

/************* copied and modified from esp32-camera/to_bmp.c to access jpg_scale_t *****************/

typedef struct {
  uint16_t width;
  uint16_t height;
  uint16_t data_offset;
  const uint8_t *input;
  uint8_t *output;
} rgb_jpg_decoder;

static bool _rgb_write(void * arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data) {
  rgb_jpg_decoder * jpeg = (rgb_jpg_decoder *)arg;
  if (!data){
    if (x == 0 && y == 0) {
      // write start
      jpeg->width = w;
      jpeg->height = h;
      // if output is null, this is BMP
      if (!jpeg->output) {
        jpeg->output = (uint8_t*)ps_malloc((w*h*3)+jpeg->data_offset);
        if (!jpeg->output) return false;
      }
    } 
    return true;
  }

  size_t jw = jpeg->width*3;
  size_t t = y * jw;
  size_t b = t + (h * jw);
  size_t l = x * 3;
  uint8_t *out = jpeg->output+jpeg->data_offset;
  uint8_t *o = out;
  size_t iy, ix;
  w = w * 3;

  for (iy=t; iy<b; iy+=jw) {
    o = out+iy+l;
    for (ix=0; ix<w; ix+=3) {
      o[ix] = data[ix+2];
      o[ix+1] = data[ix+1];
      o[ix+2] = data[ix];
    }
    data+=w;
  }
  return true;
}

static uint32_t _jpg_read(void * arg, size_t index, uint8_t *buf, size_t len) {
  rgb_jpg_decoder * jpeg = (rgb_jpg_decoder *)arg;
  if (buf) memcpy(buf, jpeg->input + index, len);
  return len;
}

typedef struct {
  uint32_t filesize;
  uint32_t reserved;
  uint32_t fileoffset_to_pixelarray;
  uint32_t dibheadersize;
  int32_t width;
  int32_t height;
  uint16_t planes;
  uint16_t bitsperpixel;
  uint32_t compression;
  uint32_t imagesize;
  uint32_t ypixelpermeter;
  uint32_t xpixelpermeter;
  uint32_t numcolorspallette;
  uint32_t mostimpcolor;
} bmp_header_t;

static bool jpg2rgb(const uint8_t *src, size_t src_len, uint8_t ** out, size_t * out_len, uint8_t scale, uint8_t bmpOffset) {
  rgb_jpg_decoder jpeg;
  jpeg.width = 0;
  jpeg.height = 0;
  jpeg.input = src;
  jpeg.output = NULL; 
  jpeg.data_offset = bmpOffset;
  if (esp_jpg_decode(src_len, jpg_scale_t(scale), _jpg_read, _rgb_write, (void*)&jpeg) != ESP_OK) {
    *out = jpeg.output;
    return false;
  }

  size_t output_size = jpeg.width*jpeg.height*3;

  if (WANT_BMP) {
    jpeg.output[0] = 'B';
    jpeg.output[1] = 'M';
    bmp_header_t * bitmap = (bmp_header_t*)&jpeg.output[2];
    bitmap->reserved = 0;
    bitmap->filesize = output_size+bmpOffset;
    bitmap->fileoffset_to_pixelarray = bmpOffset;
    bitmap->dibheadersize = 40;
    bitmap->width = jpeg.width;
    bitmap->height = -jpeg.height; //set negative for top to bottom
    bitmap->planes = 1;
    bitmap->bitsperpixel = 24;
    bitmap->compression = 0;
    bitmap->imagesize = output_size;
    bitmap->ypixelpermeter = 0x0B13 ; //2835 , 72 DPI
    bitmap->xpixelpermeter = 0x0B13 ; //2835 , 72 DPI
    bitmap->numcolorspallette = 0;
    bitmap->mostimpcolor = 0;
  }
  *out = jpeg.output;
  *out_len = output_size+bmpOffset;

  return true;
}
