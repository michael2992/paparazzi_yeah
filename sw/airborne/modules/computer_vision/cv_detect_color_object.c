/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;


uint32_t orange_strip1 = 0;
uint32_t orange_strip2 = 0;
uint32_t orange_strip3 = 0;
uint32_t orange_strip4 = 0;
uint32_t orange_strip5 = 0;

uint32_t orange_strip6 = 0;
uint32_t orange_strip7 = 0;
uint32_t orange_strip8 = 0;
uint32_t orange_strip9 = 0;
uint32_t orange_strip10 = 0;


uint32_t orange_strip11 = 0;
uint32_t orange_strip12 = 0;
uint32_t orange_strip13= 0;
uint32_t orange_strip14 = 0;
uint32_t orange_strip15= 0;




uint32_t green_strip1 = 0;
uint32_t green_strip2 = 0;
uint32_t green_strip3 = 0;
uint32_t green_strip4 = 0;
uint32_t green_strip5 = 0;

uint32_t green_strip6 = 0;
uint32_t green_strip7 = 0;
uint32_t green_strip8 = 0;
uint32_t green_strip9 = 0;
uint32_t green_strip10 = 0;


uint32_t green_strip11 = 0;
uint32_t green_strip12 = 0;
uint32_t green_strip13 = 0;
uint32_t green_strip14 = 0;
uint32_t green_strip15 = 0;







// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;

  uint32_t strip1;
  uint32_t strip2;
  uint32_t strip3;
  uint32_t strip4;
  uint32_t strip5;
  uint32_t strip6;
  uint32_t strip7;
  uint32_t strip8;
  uint32_t strip9;
  uint32_t strip10;
  uint32_t strip11;
  uint32_t strip12;
  uint32_t strip13;
  uint32_t strip14;
  uint32_t strip15;

  bool updated;
};
struct color_object_t global_filters[2];

// Function
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              uint32_t* p_strip1, uint32_t* p_strip2, 
                              uint32_t* p_strip3, uint32_t* p_strip4,
                              uint32_t* p_strip5,
                              
                              uint32_t* p_strip6, uint32_t* p_strip7,
                              uint32_t* p_strip8, uint32_t* p_strip9, 
                              uint32_t* p_strip10,

                              uint32_t* p_strip11, uint32_t* p_strip12, 
                              uint32_t* p_strip13, uint32_t* p_strip14,
                              uint32_t* p_strip15
                              );

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;

  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;
      break;
    case 2:
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;
      break;
    default:
      return img;
  };

  int32_t x_c, y_c;

  uint32_t strip1 = 0;
  uint32_t strip2 = 0;
  uint32_t strip3 = 0;
  uint32_t strip4 = 0;
  uint32_t strip5 = 0;

  uint32_t strip6 = 0;
  uint32_t strip7 = 0;
  uint32_t strip8 = 0;
  uint32_t strip9 = 0;
  uint32_t strip10 = 0;

  uint32_t strip11 = 0;
  uint32_t strip12 = 0;
  uint32_t strip13 = 0;
  uint32_t strip14 = 0;
  uint32_t strip15 = 0;


  // Filter and find centroid
  uint32_t count = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max,
  &strip1, &strip2, &strip3, &strip4, &strip5,
  &strip6, &strip7, &strip8, &strip9, &strip10,
  &strip11, &strip12, &strip13, &strip14, &strip15);


  VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
        hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count;
  global_filters[filter-1].x_c = x_c;
  global_filters[filter-1].y_c = y_c;

  global_filters[filter-1].strip1 = strip1;
  global_filters[filter-1].strip2 = strip2;
  global_filters[filter-1].strip3 = strip3;
  global_filters[filter-1].strip4 = strip4;
  global_filters[filter-1].strip5 = strip5;
  
  global_filters[filter-1].strip6 = strip6;
  global_filters[filter-1].strip7 = strip7;
  global_filters[filter-1].strip8 = strip8;
  global_filters[filter-1].strip9 = strip9;
  global_filters[filter-1].strip10 = strip10;

  global_filters[filter-1].strip11 = strip11;
  global_filters[filter-1].strip12 = strip12;
  global_filters[filter-1].strip13 = strip13;
  global_filters[filter-1].strip14 = strip14;
  global_filters[filter-1].strip15 = strip15;

  

  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

struct image_t *object_detector2(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 2);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
  cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
  cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
  cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
  cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
  cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
  cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
  cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
  cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
  cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
  cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
  cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2, 1);
#endif
}

/*
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
 */


// The function has been extended to also divide the image into 15 vertical strips
// and count the pixels in each strip that are within the filter parameter values. 
// The function now also returns these counts as unsigned integers.

uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              uint32_t* strip1, uint32_t* strip2, 
                              uint32_t* strip3, uint32_t* strip4,
                              uint32_t* strip5,
                              uint32_t* strip6, uint32_t* strip7, 
                              uint32_t* strip8, uint32_t* strip9,
                              uint32_t* strip10,
                              uint32_t* strip11, uint32_t* strip12, 
                              uint32_t* strip13, uint32_t* strip14,
                              uint32_t* strip15) {
  // Initialize counters and totalizers
  uint32_t cnt = 0, tot_x = 0, tot_y = 0;
  uint32_t stripCounts[15] = {0}; // Use an array for strip counts for easier indexing
  uint8_t *buffer = img->buf;
  uint16_t stripHeight = img->h / 15; // Determine the height of each strip for horizontal division

  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y++) {
    // Determine which horizontal strip the pixel belongs to
    uint16_t currentStrip = y / stripHeight;
    currentStrip = currentStrip >= 15 ? 14 : currentStrip; // Ensure the last row of pixels falls into the last strip

    for (uint16_t x = 0; x < img->w; x++) {
      // Calculate the buffer position once for efficiency
      size_t bufPos = y * 2 * img->w + 2 * x;
      
      // Directly access the Y, U, and V components
      uint8_t ypVal = buffer[bufPos + (x % 2 == 0 ? 1 : 3)];
      uint8_t upVal = buffer[bufPos + (x % 2 == 0 ? 0 : -2)];
      uint8_t vpVal = buffer[bufPos + 2];

      if (ypVal >= lum_min && ypVal <= lum_max &&
          upVal >= cb_min  && upVal <= cb_max  &&
          vpVal >= cr_min  && vpVal <= cr_max) {
        cnt++;
        tot_x += x;
        tot_y += y;
        stripCounts[currentStrip]++; // Increment the corresponding strip counter directly

        if (draw) {
          buffer[bufPos + (x % 2 == 0 ? 1 : 3)] = 255; // Make pixel brighter if draw is true
        }
      }
    }
  }

  // Update the centroid coordinates
  if (cnt > 0) {
    *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
    *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }

  // Update strip counts
  if (strip1) *strip1 = stripCounts[0];
  if (strip2) *strip2 = stripCounts[1];
  if (strip3) *strip3 = stripCounts[2];
  if (strip4) *strip4 = stripCounts[3];
  if (strip5) *strip5 = stripCounts[4];

  if (strip6) *strip6 = stripCounts[5];
  if (strip7) *strip7 = stripCounts[6];
  if (strip8) *strip8 = stripCounts[7];
  if (strip9) *strip9 = stripCounts[8];
  if (strip10) *strip10 = stripCounts[9];

  if (strip11) *strip11 = stripCounts[10];
  if (strip12) *strip12 = stripCounts[11];
  if (strip13) *strip13 = stripCounts[12];
  if (strip14) *strip14 = stripCounts[13];
  if (strip15) *strip15 = stripCounts[14];

  return cnt;
}



// The ABI messages were also modified to send back the pixel counts to the orange_avoider_guided.c file
void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        0, 0, local_filters[0].color_count,
        local_filters[0].strip1, local_filters[0].strip2 , local_filters[0].strip3 ,local_filters[0].strip4 ,local_filters[0].strip5,
        local_filters[0].strip6, local_filters[0].strip7 , local_filters[0].strip8 ,local_filters[0].strip9,local_filters[0].strip10,
        local_filters[0].strip11, local_filters[0].strip12 , local_filters[0].strip13 ,local_filters[0].strip14 ,local_filters[0].strip15, 0);
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
        0, 0, local_filters[1].color_count,

        local_filters[1].strip1, local_filters[1].strip2 , local_filters[1].strip3 ,local_filters[1].strip4 ,local_filters[1].strip5,
        local_filters[1].strip6, local_filters[1].strip7 , local_filters[1].strip8 ,local_filters[1].strip9,local_filters[1].strip10,
        local_filters[1].strip11, local_filters[1].strip12 , local_filters[1].strip13 ,local_filters[1].strip14 ,local_filters[1].strip15, 1);
    local_filters[1].updated = false;
  }
}
