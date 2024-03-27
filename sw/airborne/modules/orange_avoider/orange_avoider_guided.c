/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>
#include <math.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA
};

// define settings
float oag_color_count_frac = 0.18f  * 0.20;       // obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac = 0.05f * 2;       // floor detection threshold as a fraction of total of image
float oag_max_speed = 0.8f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(30.f);  // heading change setpoint for avoidance [rad/s]

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.

const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

//////////////////
int32_t orange_count1 = 0;
int32_t orange_count2 = 0;
int32_t orange_count3 = 0;
int32_t orange_count4 = 0;
int32_t orange_count5 = 0;

int32_t orange_count6 = 0;
int32_t orange_count7 = 0;
int32_t orange_count8 = 0;
int32_t orange_count9 = 0;
int32_t orange_count10 = 0;

int32_t orange_count11 = 0;
int32_t orange_count12 = 0;
int32_t orange_count13 = 0;
int32_t orange_count14 = 0;
int32_t orange_count15 = 0;


int32_t green_count1 = 0;
int32_t green_count2 = 0;
int32_t green_count3 = 0;
int32_t green_count4 = 0;
int32_t green_count5 = 0;

int32_t green_count6 = 0;
int32_t green_count7 = 0;
int32_t green_count8 = 0;
int32_t green_count9 = 0;
int32_t green_count10 = 0;

int32_t green_count11 = 0;
int32_t green_count12 = 0;
int32_t green_count13 = 0;
int32_t green_count14 = 0;
int32_t green_count15 = 0;

int32_t pixels_per_strip = (240*520)/15; 
int32_t alpha = 1.4f;


int32_t temp_heading;

u_int32_t new_color_count;

int32_t safe_heading;


int32_t temp_arrray[15] = {0};


float heading_stored = 0;               // heading stored for search for safe heading [deg]
float heading_increment = 0.f;          // heading angle increment [deg]
const int16_t threshold_decrease = 2;   // decrease color count threshold when heading is divisible by 360

// declare out_heading
float our_heading = 0;

//////////////////



// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality,
                               int32_t strip1, int32_t strip2, int32_t strip3, int32_t strip4, int32_t strip5,
                               int32_t strip6, int32_t strip7, int32_t strip8, int32_t strip9, int32_t strip10,
                               int32_t strip11, int32_t strip12, int32_t strip13, int32_t strip14, int32_t strip15,
                               int16_t __attribute__((unused)) extra)
{
  color_count = quality;

  orange_count1 = strip1;
  orange_count2 = strip2;
  orange_count3 = strip3;
  orange_count4 = strip4;
  orange_count5 = strip5;
  
  orange_count6 = strip6;
  orange_count7 = strip7;
  orange_count8 = strip8;
  orange_count9 = strip9;
  orange_count10 = strip10;

  orange_count11 = strip11;
  orange_count12 = strip12;
  orange_count13 = strip13;
  orange_count14 = strip14;
  orange_count15 = strip15;
}

#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality,
                               int32_t strip1, int32_t strip2, int32_t strip3, int32_t strip4, int32_t strip5,
                               int32_t strip6, int32_t strip7, int32_t strip8, int32_t strip9, int32_t strip10,
                               int32_t strip11, int32_t strip12, int32_t strip13, int32_t strip14, int32_t strip15,
                               int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;

  green_count1 = pixels_per_strip - strip1;
  green_count2 = pixels_per_strip - strip2;
  green_count3 = pixels_per_strip - strip3;
  green_count4 = pixels_per_strip - strip4;
  green_count5 = pixels_per_strip - strip5;
  
  green_count6 = pixels_per_strip - strip6;
  green_count7 = pixels_per_strip - strip7;
  green_count8 = pixels_per_strip - strip8;
  green_count9 = pixels_per_strip - strip9;
  green_count10 = pixels_per_strip - strip10;

  green_count11 = pixels_per_strip - strip11;
  green_count12 = pixels_per_strip - strip12;
  green_count13 = pixels_per_strip - strip13;
  green_count14 = pixels_per_strip - strip14;
  green_count15 = pixels_per_strip - strip15;
    
    
    

}

int findMinIndex(int arr[], int size);


int findMinIndex(int arr[], int size) {
    if (size <= 0) return -1; // Return an invalid index if the array is empty

    int minIndex = 0; // Start with the first element as the minimum
    for (int i = 1; i < size; i++) {
        if (arr[i] < arr[minIndex]) {
            minIndex = i; // Update the index of the new minimum
        }
    }
    return minIndex;
}

// float adjustAngle(float temp_heading) {
//     float adjusted_heading = temp_heading + RadOfDeg(150);

//     // Wrap the heading to be within -PI and PI
//     adjusted_heading = fmod(adjusted_heading + M_PI, 2 * M_PI);
//     if (adjusted_heading < 0)
//         adjusted_heading += 2 * M_PI;
//     adjusted_heading -= M_PI;

//     return adjusted_heading;
// }
  



/*
 * Initialisation function
 */
void orange_avoider_guided_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
  


}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void orange_avoider_guided_periodic(void)
{
  temp_arrray[0] = orange_count1 + alpha*green_count1;
  temp_arrray[1] = orange_count2 + alpha*green_count2;
  temp_arrray[2] = orange_count3 + alpha*green_count3;
  temp_arrray[3] = orange_count4 + alpha*green_count4;
  temp_arrray[4] = orange_count5 + alpha*green_count5;

  temp_arrray[5] = orange_count1 + alpha*green_count1;
  temp_arrray[6] = orange_count2 + alpha*green_count2;
  temp_arrray[7] = orange_count3 + alpha*green_count3;
  temp_arrray[8] = orange_count4 + alpha*green_count4;
  temp_arrray[9] = orange_count5 + alpha*green_count5;

  temp_arrray[10] = orange_count1 + alpha*green_count1;
  temp_arrray[11] = orange_count2 + alpha*green_count2;
  temp_arrray[12] = orange_count3 + alpha*green_count3;
  temp_arrray[13] = orange_count4 + alpha*green_count4;
  temp_arrray[14] = orange_count5 + alpha*green_count5;

  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  }

  // u_int32_t new_color_count = orange_count2 + orange_count3 + orange_count4;
  u_int32_t new_color_count = orange_count6 + orange_count7 + orange_count8 + orange_count9 + orange_count10;

  // compute current color thresholds
  int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h ;
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h ;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  //VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", new_color_count, color_count_threshold, navigation_state);
  //VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  // VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);

  // Change the color count threshold if we have turned 360 degrees
  if (heading_increment >= 2*M_PI){
    color_count_threshold = color_count_threshold/threshold_decrease;
    VERBOSE_PRINT("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
  }


  VERBOSE_PRINT("state: %u", navigation_state);

  // update our safe confidence using color threshold


  if(new_color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);



  float speed_sp = fminf(oag_max_speed, 0.35f * obstacle_free_confidence);

  
  // VERBOSE_PRINT("orange: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i",
  //   orange_count1, orange_count2, orange_count3, orange_count4, orange_count5,
  //   orange_count6, orange_count7, orange_count8, orange_count9, orange_count10, 
  //   orange_count11, orange_count12, orange_count13, orange_count14, orange_count15);


  // VERBOSE_PRINT("green: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i",
  // green_count1, green_count2, green_count3, green_count4, green_count5,
  // green_count6, green_count7, green_count8, green_count9, green_count10, 
  // green_count11, green_count12, green_count13, green_count14, green_count15);
  // VERBOSE_PRINT("green: %i, %i, %i, %i, %i", green_count1, green_count2, green_count3, green_count4, green_count5);

  

  switch (navigation_state){
    case SAFE:
      if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
        temp_heading = stateGetNedToBodyEulers_f()->psi;
        heading_stored = temp_heading;

        navigation_state = OUT_OF_BOUNDS;
      } else if (new_color_count > color_count_threshold){
        navigation_state = OBSTACLE_FOUND;
      } else {
        guidance_h_set_body_vel(speed_sp, 0);
        heading_stored = stateGetNedToBodyEulers_f()->psi;
      }

      break;
    case OBSTACLE_FOUND:
      // stop
       guidance_h_set_body_vel(0, 0);

      // randomly select new search direction
      // chooseRandomIncrementAvoidance();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      //guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

      guidance_h_set_body_vel(0, 0);



      safe_heading = (findMinIndex(temp_arrray, 15) - 7) * RadOfDeg(10);

      VERBOSE_PRINT("safe_heading: %f", (findMinIndex(temp_arrray, 15) - 7) * RadOfDeg(10));

      our_heading = stateGetNedToBodyEulers_f()->psi;
      
      // Make our heading positive
      if (our_heading < 0){
        our_heading += 2*M_PI;
      }

      if (heading_stored < 0){
        heading_stored += 2*M_PI;
      }

      // Find the absolute change in heading
      heading_increment = our_heading - heading_stored;

      guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi + safe_heading);


      navigation_state = SAFE;
      // make sure we have a couple of good readings before declaring the way safe
      // if (obstacle_free_confidence >= 2){
      //   guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
      //   navigation_state = SAFE;
      // }
      break;
    case OUT_OF_BOUNDS:
      // stop
      guidance_h_set_body_vel(0, 0);

      // start turn back into arena
      guidance_h_set_heading( stateGetNedToBodyEulers_f()->psi + avoidance_heading_direction*RadOfDeg(90));

      //VERBOSE_PRINT("temp_heading %f", temp_heading);
      //VERBOSE_PRINT("lower %f, current_heading %f, upper %f ",
      // adjustAngle(temp_heading+RadOfDeg(180)), stateGetNedToBodyEulers_f()->psi, adjustAngle(temp_heading + RadOfDeg(190)));

      
      // if (adjustAngle((temp_heading + RadOfDeg(160))) < stateGetNedToBodyEulers_f()->psi && stateGetNedToBodyEulers_f()->psi < adjustAngle(temp_heading + RadOfDeg(210))) {
      //   navigation_state = SAFE;
      // }

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      //guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi + RadOfDeg(180));

      

      break;
    case REENTER_ARENA:
      // force floor center to opposite side of turn to head back into arena
      // if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
      //   // return to heading mode
      //   guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);

      //   // reset safe counter
      //   obstacle_free_confidence = 0;

      //   // ensure direction is safe before continuing
      //   navigation_state = SAFE;
      // }
      navigation_state = SAFE;
      break;
    default:
      break;
  }


  // orange_count1 = 0;
  // orange_count2 = 0;
  // orange_count3 = 0;
  // orange_count4 = 0;
  // orange_count5 = 0;
  return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    avoidance_heading_direction = 1.f;
    // VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  } else {
    avoidance_heading_direction = -1.f;
    // VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  }
  return false;
}
