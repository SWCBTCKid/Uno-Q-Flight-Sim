#include "gx_fdm.h"

#include <FGFDMExec.h>
#include <input_output/FGPropertyManager.h>
#include <simgear/misc/sg_path.hxx>

#include <cmath>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <vector>

struct gx_fdm {
  std::unique_ptr<JSBSim::FGFDMExec> exec;
  std::vector<SGPropertyNode*> handles;
  std::string last_error;
};

namespace {

template <typename F>
int guarded(gx_fdm* fdm, const char* what, F&& fn) noexcept {
  if (fdm == nullptr) return -2;
  try {
    fdm->last_error.clear();
    return fn();
  } catch (const std::exception& error) {
    try { fdm->last_error = std::string(what) + ": " + error.what(); } catch (...) {}
  } catch (const std::string& error) {
    try { fdm->last_error = std::string(what) + ": " + error; } catch (...) {}
  } catch (...) {
    try { fdm->last_error = std::string(what) + ": unknown C++ exception"; } catch (...) {}
  }
  return -1;
}

void copy_error(char* destination, size_t length, const char* message) noexcept {
  if (destination == nullptr || length == 0) return;
  size_t count = std::strlen(message);
  if (count >= length) count = length - 1;
  std::memcpy(destination, message, count);
  destination[count] = '\0';
}

SGPath path_of(const char* path) { return SGPath::fromLocal8Bit(path); }

}  // namespace

extern "C" {

gx_fdm* gx_fdm_create(const char* root, char* error, size_t error_length) {
  if (root == nullptr) {
    copy_error(error, error_length, "gx_fdm_create: jsbsim_root is NULL");
    return nullptr;
  }
  gx_fdm* fdm = nullptr;
  try {
    fdm = new gx_fdm();
    fdm->exec = std::make_unique<JSBSim::FGFDMExec>();
    fdm->exec->SetRootDir(path_of(root));
    fdm->exec->SetAircraftPath(SGPath("aircraft"));
    fdm->exec->SetEnginePath(SGPath("engine"));
    fdm->exec->SetSystemsPath(SGPath("systems"));
    copy_error(error, error_length, "");
    return fdm;
  } catch (const std::exception& exception) {
    copy_error(error, error_length, exception.what());
  } catch (const std::string& message) {
    copy_error(error, error_length, message.c_str());
  } catch (...) {
    copy_error(error, error_length, "gx_fdm_create: unknown C++ exception");
  }
  delete fdm;
  return nullptr;
}

int gx_fdm_load_model(gx_fdm* fdm, const char* aircraft, double dt) {
  return guarded(fdm, "gx_fdm_load_model", [&]() -> int {
    if (aircraft == nullptr) { fdm->last_error = "gx_fdm_load_model: aircraft is NULL"; return -1; }
    fdm->exec->Setdt(dt);
    if (!fdm->exec->LoadModel(aircraft)) {
      fdm->last_error = std::string("gx_fdm_load_model: LoadModel failed for ") + aircraft;
      return -1;
    }
    return 0;
  });
}

int gx_fdm_load_script_ic(gx_fdm* fdm, const char* script, double dt, const char* initfile) {
  return guarded(fdm, "gx_fdm_load_script", [&]() -> int {
    if (script == nullptr) { fdm->last_error = "gx_fdm_load_script: script_path is NULL"; return -1; }
    SGPath initial = (initfile != nullptr && initfile[0] != '\0') ? path_of(initfile) : SGPath();
    if (!fdm->exec->LoadScript(path_of(script), dt, initial)) {
      fdm->last_error = std::string("gx_fdm_load_script: LoadScript failed for ") + script;
      return -1;
    }
    return 0;
  });
}

int gx_fdm_load_script(gx_fdm* fdm, const char* script, double dt) {
  return gx_fdm_load_script_ic(fdm, script, dt, nullptr);
}

int gx_fdm_run_ic(gx_fdm* fdm) {
  return guarded(fdm, "gx_fdm_run_ic", [&]() -> int {
    if (!fdm->exec->RunIC()) { fdm->last_error = "gx_fdm_run_ic: RunIC failed"; return -1; }
    return 0;
  });
}

int gx_fdm_step(gx_fdm* fdm) {
  return guarded(fdm, "gx_fdm_step", [&]() -> int { return fdm->exec->Run() ? 0 : 1; });
}

int gx_fdm_bind(gx_fdm* fdm, const char* property) {
  return guarded(fdm, "gx_fdm_bind", [&]() -> int {
    if (property == nullptr) { fdm->last_error = "gx_fdm_bind: property is NULL"; return -1; }
    SGPropertyNode* node = fdm->exec->GetPropertyManager()->GetNode(property, false);
    if (node == nullptr) {
      fdm->last_error = std::string("gx_fdm_bind: no property ") + property;
      return -1;
    }
    fdm->handles.push_back(node);
    return static_cast<int>(fdm->handles.size() - 1);
  });
}

double gx_fdm_get(gx_fdm* fdm, int handle) {
  if (fdm == nullptr || handle < 0 || static_cast<size_t>(handle) >= fdm->handles.size()) return std::nan("");
  try { return fdm->handles[static_cast<size_t>(handle)]->getDoubleValue(); }
  catch (...) { return std::nan(""); }
}

void gx_fdm_set(gx_fdm* fdm, int handle, double value) {
  if (fdm == nullptr) return;
  if (handle < 0 || static_cast<size_t>(handle) >= fdm->handles.size()) {
    try { fdm->last_error = "gx_fdm_set: invalid handle"; } catch (...) {}
    return;
  }
  try { fdm->handles[static_cast<size_t>(handle)]->setDoubleValue(value); }
  catch (const std::exception& error) {
    try { fdm->last_error = std::string("gx_fdm_set: ") + error.what(); } catch (...) {}
  } catch (...) {
    try { fdm->last_error = "gx_fdm_set: unknown C++ exception"; } catch (...) {}
  }
}

int gx_fdm_reset(gx_fdm* fdm) {
  return guarded(fdm, "gx_fdm_reset", [&]() -> int {
    fdm->exec->ResetToInitialConditions(0);
    return 0;
  });
}

const char* gx_fdm_last_error(gx_fdm* fdm) {
  return fdm == nullptr ? "gx_fdm: NULL handle" : fdm->last_error.c_str();
}

void gx_fdm_destroy(gx_fdm* fdm) {
  try { delete fdm; } catch (...) {}
}

}  // extern "C"
