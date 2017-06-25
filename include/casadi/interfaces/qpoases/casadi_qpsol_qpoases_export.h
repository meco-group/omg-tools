
#ifndef CASADI_QPSOL_QPOASES_EXPORT_H
#define CASADI_QPSOL_QPOASES_EXPORT_H

#ifdef CASADI_QPSOL_QPOASES_STATIC_DEFINE
#  define CASADI_QPSOL_QPOASES_EXPORT
#  define CASADI_QPSOL_QPOASES_NO_EXPORT
#else
#  ifndef CASADI_QPSOL_QPOASES_EXPORT
#    ifdef casadi_qpsol_qpoases_EXPORTS
        /* We are building this library */
#      define CASADI_QPSOL_QPOASES_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CASADI_QPSOL_QPOASES_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CASADI_QPSOL_QPOASES_NO_EXPORT
#    define CASADI_QPSOL_QPOASES_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CASADI_QPSOL_QPOASES_DEPRECATED
#  define CASADI_QPSOL_QPOASES_DEPRECATED __attribute__ ((__deprecated__))
#  define CASADI_QPSOL_QPOASES_DEPRECATED_EXPORT CASADI_QPSOL_QPOASES_EXPORT __attribute__ ((__deprecated__))
#  define CASADI_QPSOL_QPOASES_DEPRECATED_NO_EXPORT CASADI_QPSOL_QPOASES_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define CASADI_QPSOL_QPOASES_NO_DEPRECATED
#endif

#endif
