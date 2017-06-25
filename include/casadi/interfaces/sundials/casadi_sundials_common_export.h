
#ifndef CASADI_SUNDIALS_COMMON_EXPORT_H
#define CASADI_SUNDIALS_COMMON_EXPORT_H

#ifdef CASADI_SUNDIALS_COMMON_STATIC_DEFINE
#  define CASADI_SUNDIALS_COMMON_EXPORT
#  define CASADI_SUNDIALS_COMMON_NO_EXPORT
#else
#  ifndef CASADI_SUNDIALS_COMMON_EXPORT
#    ifdef casadi_sundials_common_EXPORTS
        /* We are building this library */
#      define CASADI_SUNDIALS_COMMON_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CASADI_SUNDIALS_COMMON_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CASADI_SUNDIALS_COMMON_NO_EXPORT
#    define CASADI_SUNDIALS_COMMON_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CASADI_SUNDIALS_COMMON_DEPRECATED
#  define CASADI_SUNDIALS_COMMON_DEPRECATED __attribute__ ((__deprecated__))
#  define CASADI_SUNDIALS_COMMON_DEPRECATED_EXPORT CASADI_SUNDIALS_COMMON_EXPORT __attribute__ ((__deprecated__))
#  define CASADI_SUNDIALS_COMMON_DEPRECATED_NO_EXPORT CASADI_SUNDIALS_COMMON_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define CASADI_SUNDIALS_COMMON_NO_DEPRECATED
#endif

#endif
