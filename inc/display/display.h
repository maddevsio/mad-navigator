#ifndef __DISPLAY_H
#define __DISPLAY_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  int16_t x;
  int16_t y;
} Point2d;

typedef struct {
  void (*clear)(void);
  void (*init)(void);
  void (*syncAll)(void);
  void (*setBacklight)(bool isOn);

  void (*setPixel)(int16_t x, int16_t y);
  uint8_t (*getPixel)(int16_t x, int16_t y);
  void (*clearPixel)(int16_t x, int16_t y);

  void (*syncRect)(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
  void (*printText)(int16_t x, int16_t y, const char *value);
} Display_t;

void calcCircleBuffer(uint16_t radius);
void drawCircleFromBuffer(int16_t cx, int16_t cy, bool putPixel);
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool putPixel);
void drawArrow(int16_t arrowCenterX, int16_t arrowCenterY, float azimuth, bool paint);

extern Display_t *display;
Display_t *initDisplay(void);

#endif
