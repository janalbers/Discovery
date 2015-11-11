/*----------------------------------------------------------------------------
 * Name:    debuginfo.h
 * Purpose: generating debug info
 * Note(s):
 *----------------------------------------------------------------------------
*/
#define DEBUG_INFO_ENABLE

#ifndef DEBUG_INFO_ENABLE
  #define DEBUG_INFO_DAC        1
  #define DEBUG_INFO_CAN        1
  #define DEBUG_INFO_GENERIC    1
  #define DEBUG_INFO_STEPPER    1
  #define DEBUG_INFO_ENDSTOP    1
#else
  #define DEBUG_INFO_DAC        0
  #define DEBUG_INFO_CAN        0
  #define DEBUG_INFO_GENERIC    0
  #define DEBUG_INFO_STEPPER    0
  #define DEBUG_INFO_ENDSTOP    0
#endif

//Define max 16 modules that can be used in debug info
  #define DEBUG_MODULE_DEBUG    0
  #define DEBUG_MODULE_MAIN     1
  #define DEBUG_MODULE_DAC      2
  #define DEBUG_MODULE_CAN      3
  #define DEBUG_MODULE_GENERIC  4
  #define DEBUG_MODULE_STEPPER  5
  #define DEBUG_MODULE_ENDSTOP  6
  #define DEBUG_MODULE_HAL      7




