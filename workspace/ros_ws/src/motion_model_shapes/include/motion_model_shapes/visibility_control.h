/** \file
 * \brief Symbol-visibility export/import macros for `motion_model_shapes`.
 * See \ref plugin_architecture (section 2) for why every plugin package
 * defines one of these. Logic borrowed (then namespaced) from the examples
 * on the gcc wiki: https://gcc.gnu.org/wiki/Visibility
 */
#ifndef MOTION_MODEL_SHAPES__VISIBILITY_CONTROL_H_
#define MOTION_MODEL_SHAPES__VISIBILITY_CONTROL_H_


#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOTION_MODEL_SHAPES_EXPORT __attribute__ ((dllexport))
    #define MOTION_MODEL_SHAPES_IMPORT __attribute__ ((dllimport))
  #else
    #define MOTION_MODEL_SHAPES_EXPORT __declspec(dllexport)
    #define MOTION_MODEL_SHAPES_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOTION_MODEL_SHAPES_BUILDING_LIBRARY
    #define MOTION_MODEL_SHAPES_PUBLIC MOTION_MODEL_SHAPES_EXPORT
  #else
    #define MOTION_MODEL_SHAPES_PUBLIC MOTION_MODEL_SHAPES_IMPORT
  #endif
  #define MOTION_MODEL_SHAPES_PUBLIC_TYPE MOTION_MODEL_SHAPES_PUBLIC
  #define MOTION_MODEL_SHAPES_LOCAL
#else
  #define MOTION_MODEL_SHAPES_EXPORT __attribute__ ((visibility("default")))
  #define MOTION_MODEL_SHAPES_IMPORT
  #if __GNUC__ >= 4
    #define MOTION_MODEL_SHAPES_PUBLIC __attribute__ ((visibility("default")))
    #define MOTION_MODEL_SHAPES_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOTION_MODEL_SHAPES_PUBLIC
    #define MOTION_MODEL_SHAPES_LOCAL
  #endif
  #define MOTION_MODEL_SHAPES_PUBLIC_TYPE
#endif

#endif  // MOTION_MODEL_SHAPES__VISIBILITY_CONTROL_H_
