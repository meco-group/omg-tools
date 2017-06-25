
#ifndef CASADI_XMLFILE_TINYXML_EXPORT_H
#define CASADI_XMLFILE_TINYXML_EXPORT_H

#ifdef CASADI_XMLFILE_TINYXML_STATIC_DEFINE
#  define CASADI_XMLFILE_TINYXML_EXPORT
#  define CASADI_XMLFILE_TINYXML_NO_EXPORT
#else
#  ifndef CASADI_XMLFILE_TINYXML_EXPORT
#    ifdef casadi_xmlfile_tinyxml_EXPORTS
        /* We are building this library */
#      define CASADI_XMLFILE_TINYXML_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CASADI_XMLFILE_TINYXML_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CASADI_XMLFILE_TINYXML_NO_EXPORT
#    define CASADI_XMLFILE_TINYXML_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CASADI_XMLFILE_TINYXML_DEPRECATED
#  define CASADI_XMLFILE_TINYXML_DEPRECATED __attribute__ ((__deprecated__))
#  define CASADI_XMLFILE_TINYXML_DEPRECATED_EXPORT CASADI_XMLFILE_TINYXML_EXPORT __attribute__ ((__deprecated__))
#  define CASADI_XMLFILE_TINYXML_DEPRECATED_NO_EXPORT CASADI_XMLFILE_TINYXML_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define CASADI_XMLFILE_TINYXML_NO_DEPRECATED
#endif

#endif
