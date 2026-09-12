#pragma once

// Debian's liblgpio1 runtime package provides liblgpio.so.1 and the Python
// SWIG wrapper but not lgpio.h. These declarations mirror the installed
// liblgpio API 0x00020200 (verified against joan2937/lgpio.h and the symbols
// exported by /lib/aarch64-linux-gnu/liblgpio.so.1).
#include <cstdint>

extern "C" {
struct lgGpioReport {
  std::uint64_t timestamp;
  std::uint8_t chip;
  std::uint8_t gpio;
  std::uint8_t level;
  std::uint8_t flags;
};
struct lgGpioAlert {
  lgGpioReport report;
  int nfyHandle;
};
using lgGpioAlertPtr = lgGpioAlert *;
using lgGpioAlertsFunc = void (*)(int, lgGpioAlertPtr, void *);

int lgGpiochipOpen(int gpioDev);
int lgGpiochipClose(int handle);
int lgGpioGetMode(int handle, int gpio);
int lgGpioClaimInput(int handle, int lFlags, int gpio);
int lgGpioClaimOutput(int handle, int lFlags, int gpio, int level);
int lgGpioClaimAlert(int handle, int lFlags, int eFlags, int gpio, int nfyHandle);
int lgGpioSetAlertsFunc(int handle, int gpio, lgGpioAlertsFunc cbf, void * userdata);
void lgGpioSetSamplesFunc(lgGpioAlertsFunc cbf, void * userdata);
int lgGpioFree(int handle, int gpio);
int lgGpioRead(int handle, int gpio);
int lgGpioWrite(int handle, int gpio, int level);
int lgTxPwm(int handle, int gpio, float frequency, float duty, int offset, int cycles);
const char * lguErrorText(int error);
}

constexpr int LG_BOTH_EDGES = 3;
constexpr int LG_SET_PULL_UP = 32;
