#ifndef MOVE_ROVER_PACKAGE__CPP_HEADER_H_
#define MOVE_ROVER_PACKAGE__CPP_HEADER_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOVE_ROVER_PACKAGE_EXPORT __attribute__ ((dllexport))
    #define MOVE_ROVER_PACKAGE_IMPORT __attribute__ ((dllimport))
  #else
    #define MOVE_ROVER_PACKAGE_EXPORT __declspec(dllexport)
    #define MOVE_ROVER_PACKAGE_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOVE_ROVER_PACKAGE_BUILDING_DLL
    #define MOVE_ROVER_PACKAGE_PUBLIC MOVE_ROVER_PACKAGE_EXPORT
  #else
    #define MOVE_ROVER_PACKAGE_PUBLIC MOVE_ROVER_PACKAGE_IMPORT
  #endif
  #define MOVE_ROVER_PACKAGE_PUBLIC_TYPE MOVE_ROVER_PACKAGE_PUBLIC
  #define MOVE_ROVER_PACKAGE_LOCAL
#else
  #define MOVE_ROVER_PACKAGE_EXPORT __attribute__ ((visibility("default")))
  #define MOVE_ROVER_PACKAGE_IMPORT
  #if __GNUC__ >= 4
    #define MOVE_ROVER_PACKAGE_PUBLIC __attribute__ ((visibility("default")))
    #define MOVE_ROVER_PACKAGE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOVE_ROVER_PACKAGE_PUBLIC
    #define MOVE_ROVER_PACKAGE_LOCAL
  #endif
  #define MOVE_ROVER_PACKAGE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // MOVE_ROVER_PACKAGE__VISIBILITY_CONTROL_H_