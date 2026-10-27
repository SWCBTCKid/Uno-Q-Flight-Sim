#ifndef GX_FDM_H
#define GX_FDM_H

#include <stddef.h>

#if defined(_WIN32)
#  if defined(GX_FDM_BUILD)
#    define GX_FDM_API __declspec(dllexport)
#  else
#    define GX_FDM_API
#  endif
#else
#  define GX_FDM_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gx_fdm gx_fdm;

GX_FDM_API gx_fdm* gx_fdm_create(const char* jsbsim_root, char* err, size_t err_len);
GX_FDM_API int gx_fdm_load_model(gx_fdm* fdm, const char* aircraft, double dt_s);
GX_FDM_API int gx_fdm_load_script(gx_fdm* fdm, const char* script_path, double dt_s);
GX_FDM_API int gx_fdm_load_script_ic(gx_fdm* fdm, const char* script_path, double dt_s,
                                     const char* initfile);
GX_FDM_API int gx_fdm_run_ic(gx_fdm* fdm);
GX_FDM_API int gx_fdm_step(gx_fdm* fdm);
GX_FDM_API int gx_fdm_bind(gx_fdm* fdm, const char* property);
GX_FDM_API double gx_fdm_get(gx_fdm* fdm, int handle);
GX_FDM_API void gx_fdm_set(gx_fdm* fdm, int handle, double value);
GX_FDM_API int gx_fdm_reset(gx_fdm* fdm);
GX_FDM_API const char* gx_fdm_last_error(gx_fdm* fdm);
GX_FDM_API void gx_fdm_destroy(gx_fdm* fdm);

#ifdef __cplusplus
}
#endif

#endif
