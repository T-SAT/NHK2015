#ifndef _SOFTWARE_TEST_H_INCLUDED
#define _SOFTWARE_TEST_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define __ASSERT_USE_STDERR
#include <assert.h>

class SoftwareTest {
  public:
    void no_error(void);
    bool is_inRangeEqual(float min_value, float target_value, float max_value);
    bool is_inRangeNotEqual(float min_value, float target_value, float max_value);
    void printd(const char *tab, const char* format, ...);
};

extern SoftwareTest SoftTest;

#endif
