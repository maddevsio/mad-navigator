#include "display/display.h"
#include "../hardware.h"
#include "commons/commons.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifdef DISPLAY_NOKIA5110
#include "display/nokiaLcd/nokia5110.h"
#ifdef BUFFERED_DISPLAY
#include "display/nokiaLcd/nokia5110_buffered.h"
#endif
#endif

#define POINTS_BUFFER_MAX_SIZE 150
static Point2d ff_pointsBuffer[POINTS_BUFFER_MAX_SIZE];
static uint16_t ff_oldestPoint = 0, ff_newestPoint = 0;
static bool ff_bufferFinished = true;

static void floodFill(const Point2d point, const uint16_t fillColor, const uint16_t borderColor);

static Point2d makePoint(int16_t x, int16_t y) {
  Point2d result = {x, y};
  return result;
}
///////////////////////////////////////////////////////

static void ff_pushToBuffer(const Point2d point) {
  ff_pointsBuffer[ff_newestPoint] = point;
  ff_bufferFinished = false;
  ff_newestPoint++;
  if(ff_newestPoint >= POINTS_BUFFER_MAX_SIZE) {
    ff_newestPoint = 0;
  }
}
///////////////////////////////////////////////////////

static Point2d ff_popFromBuffer() {
  Point2d result = ff_pointsBuffer[ff_oldestPoint];
  if(ff_oldestPoint != ff_newestPoint) {
    ff_oldestPoint++;
  } else {
    ff_bufferFinished = true;
  }

  if(ff_oldestPoint >= POINTS_BUFFER_MAX_SIZE) {
    ff_oldestPoint = 0;
  }
  return result;
}
///////////////////////////////////////////////////////

static bool ff_isBufferFinished() {
  return ff_bufferFinished;
}
///////////////////////////////////////////////////////

static void ff_resetBuffer() {
  memset(ff_pointsBuffer, 0, sizeof(ff_pointsBuffer));
  ff_bufferFinished = true;
  ff_oldestPoint = 0;
  ff_newestPoint = 0;
}

Display_t *display = NULL;
static Display_t instance;
static Display_t* getDisplay(void);
#define ARROW_LINES_COUNT 9

const static Point2d arrowLines[][2] = {
  {{-5, 15}, { -5, -6}},
  {{5, 15},  { 5, -6}},
  {{5, 15},  { -5, 15}},

  {{0, -17}, { 18, -2}},
  {{4, -5},  { 18, 6}},
  {{18, -2}, { 18, 6}},

  {{0, -17}, { -18, -2}},
  {{-4, -5},  { -18, 6}},
  {{-18, -2},{ -18, 6}},
};
///////////////////////////////////////////////////////

Display_t *initDisplay() {
  if(display == NULL) {
    display = getDisplay();
    display->init();
  }
  return display;
}
///////////////////////////////////////////////////////

static void dummyVoid() {
}
///////////////////////////////////////////////////////

static void dummyRect(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
  UNUSED(x0); UNUSED(y0);
  UNUSED(x1); UNUSED(y1);
}
///////////////////////////////////////////////////////

static void dummyPixel(int16_t x, int16_t y) {
  UNUSED(x); UNUSED(y);
}
///////////////////////////////////////////////////////

static uint8_t dummyGetPixel(int16_t x, int16_t y) {
  UNUSED(x); UNUSED(y);
  return 0;
}
///////////////////////////////////////////////////////

static int16_t circleXs[37];
static int16_t circleYs[37];

void calcCircleBuffer(uint16_t radius) {
  int i = 0;
  for(float angle = 0.0; angle < (float)M_PI_2; angle += deg2rad_coeff * 2.5f, i++) {
    circleXs[i] = (int16_t) (radius * 1.15f * cosf(angle));
    circleYs[i] = (int16_t) (radius * sinf(angle));
  }
}
///////////////////////////////////////////////////////

void drawCircleFromBuffer(int16_t cx, int16_t cy, bool putPixel) {
  for(int i = 0; i < 37; i++) {
    int16_t x = circleXs[i];
    int16_t y = circleYs[i];
    drawLine(cx - x, cy - y, cx - x, cy + y, putPixel);
    drawLine(cx + x, cy - y, cx + x, cy + y, putPixel);
  }
}
///////////////////////////////////////////////////////

static Point2d rotatePoint(const Point2d point,
                           const float angle) {
  int16_t x = (int16_t) (point.x * cosf(angle) - point.y * sinf(angle));
  int16_t y = (int16_t) (point.x * sinf(angle) + point.y * cosf(angle));
  Point2d result = {(int16_t)(x * 1.15f), y};
  return result;
}
///////////////////////////////////////////////////////

void drawArrow(int16_t arrowCenterX, int16_t arrowCenterY,
               float azimuth, bool paint) {
  for(int i = 0; i < ARROW_LINES_COUNT; i++) {
    Point2d lineStart = rotatePoint(arrowLines[i][0], azimuth);
    Point2d lineEnd   = rotatePoint(arrowLines[i][1], azimuth);
    drawLine(lineStart.x + arrowCenterX,
             lineStart.y + arrowCenterY,
             lineEnd.x + arrowCenterX,
             lineEnd.y + arrowCenterY,
             paint);
  }
  floodFill(makePoint(arrowCenterX, arrowCenterY), 0, 0);
}
///////////////////////////////////////////////////////

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool putPixel) {
  void (*pixel)(int16_t x, int16_t y);
  pixel = putPixel ? display->setPixel : display->clearPixel;
  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = (dx>dy ? dx : -dy)/2, e2;

  for(;;){
    pixel(x0, y0);
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
}
///////////////////////////////////////////////////////

void floodFill(const Point2d point, const uint16_t fillColor, const uint16_t borderColor) {
  UNUSED(borderColor);
  void (*pixel)(int16_t x, int16_t y);
  pixel = (fillColor == 1) ? display->setPixel : display->clearPixel;
  bool isCompleted = false;

  ff_resetBuffer();
  ff_pushToBuffer(point);

  while(!isCompleted) {
    Point2d p = ff_popFromBuffer();
    if(display->getPixel(p.x, p.y) == fillColor) {
      isCompleted = ff_isBufferFinished();
      continue;
    }
    pixel(p.x, p.y);

    if(p.x > 0 && (display->getPixel(p.x-1, p.y) != borderColor)) {
      ff_pushToBuffer(makePoint(p.x-1, p.y));
    }
    if(p.x < 83 && (display->getPixel(p.x+1, p.y) != borderColor)) {
      ff_pushToBuffer(makePoint(p.x+1, p.y));
    }
    if(p.y > 0 && (display->getPixel(p.x, p.y-1) != borderColor)) {
      ff_pushToBuffer(makePoint(p.x, p.y-1));
    }
    if(p.y < 43 && (display->getPixel(p.x, p.y+1) != borderColor)) {
      ff_pushToBuffer(makePoint(p.x, p.y+1));
    }
    isCompleted = ff_isBufferFinished();
  }
}
///////////////////////////////////////////////////////

#ifdef DISPLAY_NOKIA5110
Display_t* getNokia5110() {
  Display_t *result = &instance;
  result->clear = nokia5110_clear;
  result->init = nokia5110_init;
  result->syncAll = dummyVoid;
  result->setBacklight = nokia5110_light;
  result->setPixel = dummyPixel;
  result->getPixel = dummyGetPixel;
  result->clearPixel = dummyPixel;
  result->syncRect = dummyRect;
  result->printText = nokia5110_printText;
  return result;
}

#ifdef BUFFERED_DISPLAY
Display_t* getNokia5110Buffered() {
  Display_t *result = &instance;
  result->clear = nokia5110_clearBuffer;
  result->init = nokia5110_bufferInit;
  result->syncAll = nokia5110_syncBuffer;
  result->setBacklight = nokia5110_light;
  result->setPixel = nokia5110_bufferSetPixel;
  result->getPixel = nokia5110_bufferGetPixel;
  result->clearPixel = nokia5110_bufferClearPixel;
  result->syncRect = nokia5110_syncBufferPart;
  result->printText = nokia5110_bufferWriteString;
  return result;
}
#endif //BUFFERED_DISPLAY

#endif //DISPLAY_NOKIA5110

#ifdef DISPLAY_0152G
DisplayStruct* get0152G() {
  DisplayStruct *result = malloc(sizeof(DisplayStruct));
  return result;
}
#endif

Display_t* getDummy() {
  Display_t *result = &instance;
  result->init = dummyVoid;
  result->clear = dummyVoid;
  result->printText = nokia5110_printText;
  result->setBacklight = nokia5110_light;
  result->syncRect = dummyRect;
  result->syncAll = dummyVoid;
  result->setPixel = dummyPixel;
  result->clearPixel = dummyPixel;
  result->getPixel = dummyGetPixel;
  return result;
}
///////////////////////////////////////////////////////

Display_t* getDisplay() {
#ifdef DISPLAY_NOKIA5110
#ifdef BUFFERED_DISPLAY
  return getNokia5110Buffered();
#else //NOT BUFFERED_DISPLAY
  return getNokia5110();
#endif //BUFFERED_DISPLAY
#endif //DISPLAY_NOKIA5110
#ifdef DISPLAY_0152G
  return get0152G();
#endif //DISPLAY_0152G
  return getDummy();
}
