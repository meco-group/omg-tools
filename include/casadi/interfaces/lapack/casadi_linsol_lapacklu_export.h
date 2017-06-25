
#ifndef CASADI_LINSOL_LAPACKLU_EXPORT_H
#define CASADI_LINSOL_LAPACKLU_EXPORT_H

#ifdef CASADI_LINSOL_LAPACKLU_STATIC_DEFINE
#  define CASADI_LINSOL_LAPACKLU_EXPORT
#  define CASADI_LINSOL_LAPACKLU_NO_EXPORT
#else
#  ifndef CASADI_LINSOL_LAPACKLU_EXPORT
#    ifdef casadi_linsol_lapacklu_EXPORTS
        /* We are building this library */
#      define CASADI_LINSOL_LAPACKLU_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CASADI_LINSOL_LAPACKLU_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CASADI_LINSOL_LAPACKLU_NO_EXPORT
#    define CASADI_LINSOL_LAPACKLU_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CASADI_LINSOL_LAPACKLU_DEPRECATED
#  define CASADI_LINSOL_LAPACKLU_DEPRECATED __attribute__ ((__deprecated__))
#  define CASADI_LINSOL_LAPACKLU_DEPRECATED_EXPORT CASADI_LINSOL_LAPACKLU_EXPORT __attribute__ ((__deprecated__))
#  define CASADI_LINSOL_LAPACKLU_DEPRECATED_NO_EXPORT CASADI_LINSOL_LAPACKLU_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define CASADI_LINSOL_LAPACKLU_NO_DEPRECATED
#endif

#endif
