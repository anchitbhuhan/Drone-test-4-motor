#ifndef Utils_h
#define Utils_h
#include <Arduino.h>

enum DebugToken
{
  Token_All = 0,
  Token_Controller,
  Token_Sensor
};

class Utils
{
public:
    static void debug_print(DebugToken token, const __FlashStringHelper *fmt, ... );

  private:
    static bool tokenStatus[];
};




#endif;
