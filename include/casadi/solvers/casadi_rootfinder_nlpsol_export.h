
#ifndef CASADI_ROOTFINDER_NLPSOL_EXPORT_H
#define CASADI_ROOTFINDER_NLPSOL_EXPORT_H

#ifdef CASADI_ROOTFINDER_NLPSOL_STATIC_DEFINE
#  define CASADI_ROOTFINDER_NLPSOL_EXPORT
#  define CASADI_ROOTFINDER_NLPSOL_NO_EXPORT
#else
#  ifndef CASADI_ROOTFINDER_NLPSOL_EXPORT
#    ifdef casadi_rootfinder_nlpsol_EXPORTS
        /* We are building this library */
#      define CASADI_ROOTFINDER_NLPSOL_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CASADI_ROOTFINDER_NLPSOL_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CASADI_ROOTFINDER_NLPSOL_NO_EXPORT
#    define CASADI_ROOTFINDER_NLPSOL_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CASADI_ROOTFINDER_NLPSOL_DEPRECATED
#  define CASADI_ROOTFINDER_NLPSOL_DEPRECATED __attribute__ ((__deprecated__))
#  define CASADI_ROOTFINDER_NLPSOL_DEPRECATED_EXPORT CASADI_ROOTFINDER_NLPSOL_EXPORT __attribute__ ((__deprecated__))
#  define CASADI_ROOTFINDER_NLPSOL_DEPRECATED_NO_EXPORT CASADI_ROOTFINDER_NLPSOL_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define CASADI_ROOTFINDER_NLPSOL_NO_DEPRECATED
#endif

#endif
