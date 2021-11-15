#ifndef TEENSY_DRIVE_UTILS_H_
#define TEENSY_DRIVE_UTILS_H_

#ifdef __cplusplus

#ifdef NDEBUG
#define debug(x) ((void) 0)
#else

#define debug(x) std::cerr << x << std::endl
#endif

#else

#ifdef NDEBUG
#define debug_printf(...) ((void) 0)
#else
#define debug_printf(...) fprintf(stdout, __VA_ARGS__)
#endif

#endif

#endif // TEENSY_DRIVE_UTILS_H_
