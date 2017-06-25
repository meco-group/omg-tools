
#ifndef CASADI_ROOTFINDER_NEWTON_EXPORT_H
#define CASADI_ROOTFINDER_NEWTON_EXPORT_H

#ifdef CASADI_ROOTFINDER_NEWTON_STATIC_DEFINE
#  define CASADI_ROOTFINDER_NEWTON_EXPORT
#  define CASADI_ROOTFINDER_NEWTON_NO_EXPORT
#else
#  ifndef CASADI_ROOTFINDER_NEWTON_EXPORT
#    ifdef casadi_rootfinder_newton_EXPORTS
        /* We are building this library */
#      define CASADI_ROOTFINDER_NEWTON_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CASADI_ROOTFINDER_NEWTON_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CASADI_ROOTFINDER_NEWTON_NO_EXPORT
#    define CASADI_ROOTFINDER_NEWTON_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CASADI_ROOTFINDER_NEWTON_DEPRECATED
#  define CASADI_ROOTFINDER_NEWTON_DEPRECATED __attribute__ ((__deprecated__))
#  define CASADI_ROOTFINDER_NEWTON_DEPRECATED_EXPORT CASADI_ROOTFINDER_NEWTON_EXPORT __attribute__ ((__deprecated__))
#  define CASADI_ROOTFINDER_NEWTON_DEPRECATED_NO_EXPORT CASADI_ROOTFINDER_NEWTON_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define CASADI_ROOTFINDER_NEWTON_NO_DEPRECATED
#endif

#endif
