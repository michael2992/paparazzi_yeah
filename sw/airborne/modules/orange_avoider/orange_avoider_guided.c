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
  ADJUST_HEADING
};

// Settings
float oag_color_count_frac = 0.18f * 0.20;  // Obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac = 0.05f * 2;     // Floor detection threshold as a fraction of total of image
float oag_max_speed = 0.8f;                 // Max flight speed [m/s]
float oag_heading_rate = RadOfDeg(30.f);    // Heading change setpoint for avoidance [rad/s]

// Global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
int32_t color_count = 0;
int32_t floor_count = 0;
int32_t floor_centroid = 0;
float avoidance_heading_direction = 0;
int16_t obstacle_free_confidence = 0;
const int16_t max_trajectory_confidence = 5;

// Additional counts for further analysis
int32_t orange_count[15] = {0};
int32_t green_count[15] = {0};
int32_t pixels_per_strip = (240*520)/15; 
float alpha = 1.4f;
int32_t temp_heading;
u_int32_t new_color_count;
int32_t safe_heading;
int32_t temp_arrray[15] = {0};
float heading_stored = 0;               // heading stored for search for safe heading [deg]
float heading_increment = 0.f;          // heading angle increment [deg]
const int16_t threshold_decrease = 2;   // decrease color count threshold when heading is divisible by 360

// Out heading variables
float our_heading = 0;
u_int16_t safe_counter = 0;
u_int16_t safe_counter_thresehold = 10; // TUNABLE: number of consecutive safe readings before storing heading
float breakout_heading = 0;

// Counter variables for navigation states
int counter = 0;
int heading_counter = 0;
int stuck_counter = 0;

// Function prototypes
uint8_t chooseRandomIncrementAvoidance(void);
int findMinIndex(int arr[], int size);



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
  
  // A for loop would have been more efficient, but we receive integers from the filter callback, and we cannot assume they are stored sequentially
  orange_count[0] = strip1;
  orange_count[1] = strip2;
  orange_count[2] = strip3;
  orange_count[3] = strip4;
  orange_count[4] = strip5;
  
  orange_count[5] = strip6;
  orange_count[6] = strip7;
  orange_count[7] = strip8;
  orange_count[8] = strip9;
  orange_count[9] = strip10;

  orange_count[10]= strip11;
  orange_count[11]= strip12;
  orange_count[12] = strip13;
  orange_count[13] = strip14;
  orange_count[14]= strip15;
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


  // A for loop would have been more efficient, but we receive integers from the filter callback, and we cannot assume they are stored sequentially
  green_count[0] = pixels_per_strip - strip1;
  green_count[1] = pixels_per_strip - strip2;
  green_count[2] = pixels_per_strip - strip3;
  green_count[3] = pixels_per_strip - strip4;
  green_count[4] = pixels_per_strip - strip5;
  
  green_count[5] = pixels_per_strip - strip6;
  green_count[6] = pixels_per_strip - strip7;
  green_count[7] = pixels_per_strip - strip8;
  green_count[8] = pixels_per_strip - strip9;
  green_count[9]= pixels_per_strip - strip10;

  green_count[10] = pixels_per_strip - strip11;
  green_count[11] = pixels_per_strip - strip12;
  green_count[12] = pixels_per_strip - strip13;
  green_count[13]= pixels_per_strip - strip14;
  green_count[14]= pixels_per_strip - strip15;

}

int findMinIndex(int arr[], int size);

// Function to find the index of the minimum element in an array.
int findMinIndex(int arr[], int size) {
    // Check if the array is empty or has invalid size, return -1 as error indicator.
    if (size <= 0) return -1;

    // Initialize minIndex with the index of the first element.
    int minIndex = 0;
    // Loop through the array starting from the second element.
    for (int i = 1; i < size; i++) {
        // If a smaller element is found, update minIndex.
        if (arr[i] < arr[minIndex]) {
            minIndex = i;
        }
    }
    // Return the index of the smallest element.
    return minIndex;
}


/*
 * Initialisation function
 */
void orange_avoider_guided_init(void)
{
  // Initialise random values
  srand(time(NULL));
  // Randomly sets the avoidance_heading_direction to 1 or -1 (right or left)
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb); // Orange Filter
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb); // Floor/Green Filter

}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void orange_avoider_guided_periodic(void)
{
  // Calculates Cost for each strip and stores it in the temp_array
  for (int i = 0; i < 15; i++) {
    temp_arrray[i] = orange_count[i] + alpha * green_count[i];
  } 

  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  }

  // Summing the pixels of the center 5 strips as this allows the drone to determine whether it is safe to move forward
  u_int32_t new_color_count = 0;
  for (int i = 5; i <= 9; i++) { 
      new_color_count += orange_count[i];
  }

  // Prepare heading_array with summed pixel data across adjacent groups of 5 strips
  u_int32_t heading_array[11] = {0};
  for (int i = 0; i < 11; i++) {
      for (int j = 0; j < 5; j++) {
          heading_array[i] += temp_arrray[i + j];
      }
  }

  // Compute current color thresholds
  int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h ;
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h ;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  // Change the color count threshold if we have turned 360 degrees
  if (heading_increment >= 2*M_PI){
    color_count_threshold = color_count_threshold/threshold_decrease;
  }

  // update our safe confidence using color threshold
  if(new_color_count < color_count_threshold){
      obstacle_free_confidence++;
  } else {
      obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  // Set the speed based on obstacle confidence
  float speed_sp = fminf(oag_max_speed, 0.4f * obstacle_free_confidence);

  // The switch expression used to control the transition between the navigation states the drone has
  switch (navigation_state){
    // Navigation state SAFE, indicating it is safe to move forward
    case SAFE:

      // Check whether the drone has been in the safe state for more than 10 iterations, if so, go into adjust heading mode
      if (heading_counter > 10){
          heading_counter = 0;
          navigation_state = ADJUST_HEADING;
      }
      
      // Check whether enough of the floor is still visible to determine whether it is still within bounds, or not in front of an obstacle
      if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
          temp_heading = stateGetNedToBodyEulers_f()->psi;
          heading_stored = temp_heading;
          navigation_state = OUT_OF_BOUNDS;
      } else if (new_color_count > color_count_threshold){
          navigation_state = OBSTACLE_FOUND;

      } else {
          guidance_h_set_body_vel(speed_sp, 0);
          if (safe_counter < safe_counter_thresehold){
              safe_counter++;
          } else {
              safe_counter = 0;
              heading_stored = stateGetNedToBodyEulers_f()->psi;
          }

          heading_counter++;
      }
      break;

    case OBSTACLE_FOUND:
      // Stop the drone immediately
      guidance_h_set_body_vel(0, 0);
      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;

    case SEARCH_FOR_SAFE_HEADING:
      // Stop the drone immediately
      guidance_h_set_body_vel(0, 0);


      // Adding very high constant to ensure the drone does not choose these forward directions
      heading_array[4] = heading_array[4] + 15000;
      heading_array[5] = heading_array[5] + 15000;
      heading_array[6] = heading_array[6] + 15000;

      // Calculate new heading by finding strip with minimum cost
      safe_heading = (findMinIndex(heading_array, 11) - 5) * RadOfDeg(10);

      // Checks to ensure drone turns properly
      our_heading = stateGetNedToBodyEulers_f()->psi;      
      if (our_heading < 0){
        our_heading += 2*M_PI;
      }

      if (heading_stored < 0){
        heading_stored += 2*M_PI;
      }

      // Set the drone heading to calculated heading with no obstaccles
      heading_increment = our_heading - heading_stored;
      breakout_heading = RadOfDeg(safe_counter);
      guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi + safe_heading + breakout_heading);


      // Check to prevent drone getting stuck in the search for safe heading mode
      if (stuck_counter>5){
        stuck_counter= 0;
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi + avoidance_heading_direction* RadOfDeg(45));
        navigation_state = SAFE;
      }

      stuck_counter++;
      break;

    case OUT_OF_BOUNDS:
      // Stop the drone immediately
      guidance_h_set_body_vel(0, 0);

      // Turn the drone for 5 iterations to direct it back into the CyberZoo
      guidance_h_set_heading_rate(avoidance_heading_direction* RadOfDeg(45));

      if (counter > 5){
        counter = 0;
        guidance_h_set_heading_rate(0);
        navigation_state = SAFE;
      }

      counter++;
      break;

    case ADJUST_HEADING:
      // New heading is calculated by finding strip with minimum cost, without reducing speed
      safe_heading = (findMinIndex(heading_array, 11) - 5) * RadOfDeg(10);
      guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi + safe_heading + breakout_heading);
      navigation_state = SAFE;
      break; 

    default:
      break;
  }
  
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
  } else {
      avoidance_heading_direction = -1.f;
  }
  return false;
}
