
#ifndef CASADI_COMPILER_CLANG_EXPORT_H
#define CASADI_COMPILER_CLANG_EXPORT_H

#ifdef CASADI_COMPILER_CLANG_STATIC_DEFINE
#  define CASADI_COMPILER_CLANG_EXPORT
#  define CASADI_COMPILER_CLANG_NO_EXPORT
#else
#  ifndef CASADI_COMPILER_CLANG_EXPORT
#    ifdef casadi_compiler_clang_EXPORTS
        /* We are building this library */
#      define CASADI_COMPILER_CLANG_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CASADI_COMPILER_CLANG_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CASADI_COMPILER_CLANG_NO_EXPORT
#    define CASADI_COMPILER_CLANG_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CASADI_COMPILER_CLANG_DEPRECATED
#  define CASADI_COMPILER_CLANG_DEPRECATED __attribute__ ((__deprecated__))
#  define CASADI_COMPILER_CLANG_DEPRECATED_EXPORT CASADI_COMPILER_CLANG_EXPORT __attribute__ ((__deprecated__))
#  define CASADI_COMPILER_CLANG_DEPRECATED_NO_EXPORT CASADI_COMPILER_CLANG_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define CASADI_COMPILER_CLANG_NO_DEPRECATED
#endif

#endif
