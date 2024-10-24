
#ifndef EASY3D_FILEIO_EXPORT_H
#define EASY3D_FILEIO_EXPORT_H

#ifdef EASY3D_FILEIO_STATIC_DEFINE
#  define EASY3D_FILEIO_EXPORT
#  define EASY3D_FILEIO_NO_EXPORT
#else
#  ifndef EASY3D_FILEIO_EXPORT
#    ifdef easy3d_fileio_EXPORTS
        /* We are building this library */
#      define EASY3D_FILEIO_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define EASY3D_FILEIO_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef EASY3D_FILEIO_NO_EXPORT
#    define EASY3D_FILEIO_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef EASY3D_FILEIO_DEPRECATED
#  define EASY3D_FILEIO_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef EASY3D_FILEIO_DEPRECATED_EXPORT
#  define EASY3D_FILEIO_DEPRECATED_EXPORT EASY3D_FILEIO_EXPORT EASY3D_FILEIO_DEPRECATED
#endif

#ifndef EASY3D_FILEIO_DEPRECATED_NO_EXPORT
#  define EASY3D_FILEIO_DEPRECATED_NO_EXPORT EASY3D_FILEIO_NO_EXPORT EASY3D_FILEIO_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef EASY3D_FILEIO_NO_DEPRECATED
#    define EASY3D_FILEIO_NO_DEPRECATED
#  endif
#endif

#endif /* EASY3D_FILEIO_EXPORT_H */
