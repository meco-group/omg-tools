
#ifndef CASADI_LINSOL_SYMBOLICQR_EXPORT_H
#define CASADI_LINSOL_SYMBOLICQR_EXPORT_H

#ifdef CASADI_LINSOL_SYMBOLICQR_STATIC_DEFINE
#  define CASADI_LINSOL_SYMBOLICQR_EXPORT
#  define CASADI_LINSOL_SYMBOLICQR_NO_EXPORT
#else
#  ifndef CASADI_LINSOL_SYMBOLICQR_EXPORT
#    ifdef casadi_linsol_symbolicqr_EXPORTS
        /* We are building this library */
#      define CASADI_LINSOL_SYMBOLICQR_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CASADI_LINSOL_SYMBOLICQR_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CASADI_LINSOL_SYMBOLICQR_NO_EXPORT
#    define CASADI_LINSOL_SYMBOLICQR_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CASADI_LINSOL_SYMBOLICQR_DEPRECATED
#  define CASADI_LINSOL_SYMBOLICQR_DEPRECATED __attribute__ ((__deprecated__))
#  define CASADI_LINSOL_SYMBOLICQR_DEPRECATED_EXPORT CASADI_LINSOL_SYMBOLICQR_EXPORT __attribute__ ((__deprecated__))
#  define CASADI_LINSOL_SYMBOLICQR_DEPRECATED_NO_EXPORT CASADI_LINSOL_SYMBOLICQR_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define CASADI_LINSOL_SYMBOLICQR_NO_DEPRECATED
#endif

#endif
