#ifndef MOTION_MODEL_BASE__VISIBILITY_CONTROL_H_
#define MOTION_MODEL_BASE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOTION_MODEL_BASE_EXPORT __attribute__ ((dllexport))
    #define MOTION_MODEL_BASE_IMPORT __attribute__ ((dllimport))
  #else
    #define MOTION_MODEL_BASE_EXPORT __declspec(dllexport)
    #define MOTION_MODEL_BASE_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOTION_MODEL_BASE_BUILDING_LIBRARY
    #define MOTION_MODEL_BASE_PUBLIC MOTION_MODEL_BASE_EXPORT
  #else
    #define MOTION_MODEL_BASE_PUBLIC MOTION_MODEL_BASE_IMPORT
  #endif
  #define MOTION_MODEL_BASE_PUBLIC_TYPE MOTION_MODEL_BASE_PUBLIC
  #define MOTION_MODEL_BASE_LOCAL
#else
  #define MOTION_MODEL_BASE_EXPORT __attribute__ ((visibility("default")))
  #define MOTION_MODEL_BASE_IMPORT
  #if __GNUC__ >= 4
    #define MOTION_MODEL_BASE_PUBLIC __attribute__ ((visibility("default")))
    #define MOTION_MODEL_BASE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOTION_MODEL_BASE_PUBLIC
    #define MOTION_MODEL_BASE_LOCAL
  #endif
  #define MOTION_MODEL_BASE_PUBLIC_TYPE
#endif

#endif  // MOTION_MODEL_BASE__VISIBILITY_CONTROL_H_
