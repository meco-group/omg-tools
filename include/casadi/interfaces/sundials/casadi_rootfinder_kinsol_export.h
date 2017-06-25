
#ifndef CASADI_ROOTFINDER_KINSOL_EXPORT_H
#define CASADI_ROOTFINDER_KINSOL_EXPORT_H

#ifdef CASADI_ROOTFINDER_KINSOL_STATIC_DEFINE
#  define CASADI_ROOTFINDER_KINSOL_EXPORT
#  define CASADI_ROOTFINDER_KINSOL_NO_EXPORT
#else
#  ifndef CASADI_ROOTFINDER_KINSOL_EXPORT
#    ifdef casadi_rootfinder_kinsol_EXPORTS
        /* We are building this library */
#      define CASADI_ROOTFINDER_KINSOL_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CASADI_ROOTFINDER_KINSOL_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CASADI_ROOTFINDER_KINSOL_NO_EXPORT
#    define CASADI_ROOTFINDER_KINSOL_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CASADI_ROOTFINDER_KINSOL_DEPRECATED
#  define CASADI_ROOTFINDER_KINSOL_DEPRECATED __attribute__ ((__deprecated__))
#  define CASADI_ROOTFINDER_KINSOL_DEPRECATED_EXPORT CASADI_ROOTFINDER_KINSOL_EXPORT __attribute__ ((__deprecated__))
#  define CASADI_ROOTFINDER_KINSOL_DEPRECATED_NO_EXPORT CASADI_ROOTFINDER_KINSOL_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define CASADI_ROOTFINDER_KINSOL_NO_DEPRECATED
#endif

#endif
