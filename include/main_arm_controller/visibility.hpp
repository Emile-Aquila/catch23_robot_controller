//
// Created by emile on 23/06/09.
//

#ifndef ROS2_WS_VISBILITY_HPP
#define ROS2_WS_VISBILITY_HPP

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
    #define COMPOSITION_EXPORT __attribute__ ((dllexport))
    #define COMPOSITION_IMPORT __attribute__ ((dllimport))
  #else
    #define COMPOSITION_EXPORT __declspec(dllexport)
    #define COMPOSITION_IMPORT __declspec(dllimport)
  #endif
  #ifdef COMPOSITION_BUILDING_DLL
    #define COMPOSITION_PUBLIC COMPOSITION_EXPORT
  #else
    #define COMPOSITION_PUBLIC COMPOSITION_IMPORT
  #endif
  #define COMPOSITION_PUBLIC_TYPE COMPOSITION_PUBLIC
  #define COMPOSITION_LOCAL
#else
#define COMPOSITION_EXPORT __attribute__ ((visibility("default")))
#define COMPOSITION_IMPORT
#if __GNUC__ >= 4
#define COMPOSITION_PUBLIC __attribute__ ((visibility("default")))
    #define COMPOSITION_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define COMPOSITION_PUBLIC
#define COMPOSITION_LOCAL
#endif
#define COMPOSITION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif //ROS2_WS_VISBILITY_HPP
