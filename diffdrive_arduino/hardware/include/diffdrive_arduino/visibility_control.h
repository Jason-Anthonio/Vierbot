#ifndef DIFFDRIVE_ARDUINO__VISIBILITY_CONTROL_H_
#define DIFFDRIVE_ARDUINO__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DIFFDRIVE_ARDUINO_EXPORT __attribute__ ((dllexport))
    #define DIFFDRIVE_ARDUINO_IMPORT __attribute__ ((dllimport))
  #else
    #define DIFFDRIVE_ARDUINO_EXPORT __declspec(dllexport)
    #define DIFFDRIVE_ARDUINO_IMPORT __declspec(dllimport)
  #endif
  #ifdef DIFFDRIVE_ARDUINO_BUILDING_DLL
    #define DIFFDRIVE_ARDUINO_PUBLIC DIFFDRIVE_ARDUINO_EXPORT
  #else
    #define DIFFDRIVE_ARDUINO_PUBLIC DIFFDRIVE_ARDUINO_IMPORT
  #endif
  #define DIFFDRIVE_ARDUINO_PUBLIC_TYPE DIFFDRIVE_ARDUINO_PUBLIC
  #define DIFFDRIVE_ARDUINO_LOCAL
#else
  #define DIFFDRIVE_ARDUINO_EXPORT __attribute__ ((visibility ("default")))
  #define DIFFDRIVE_ARDUINO_IMPORT
  #if __GNUC__ >= 4
    #define DIFFDRIVE_ARDUINO_PUBLIC __attribute__ ((visibility ("default")))
    #define DIFFDRIVE_ARDUINO_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define DIFFDRIVE_ARDUINO_PUBLIC
    #define DIFFDRIVE_ARDUINO_LOCAL
  #endif
  #define DIFFDRIVE_ARDUINO_PUBLIC_TYPE
#endif

#endif  // DIFFDRIVE_ARDUINO__VISIBILITY_CONTROL_H_
