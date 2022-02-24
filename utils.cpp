#include "utils.h"
#include <stdarg.h>
#include <Arduino.h>

static bool Utils::tokenStatus[] = {
  false,
  true,
  true
};

void Utils::debug_print(DebugToken token, const __FlashStringHelper *fmt, ... )
{
  if (tokenStatus[token])
  {

    char buf[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt);
#ifdef __AVR__
    vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
#else
    vsnprintf(buf, sizeof(buf), (const char *)fmt, args); // for the rest of the world
#endif
    va_end(args);
    Serial.print(buf);

    //        char buf[128];
    //        va_list args;
    //        va_start(args, format);
    //        char buf[vsnprintf(NULL, 0, format, args) + 1];
    //        vsprintf(buf, format, args);
    //        Serial.print(buf);
    //        va_end(args);
  }
}

//#define PRINTF_BUF 80 // define the tmp buffer size (change if desired)
//void printf(const char *format, ...)
//{
//  char buf[PRINTF_BUF];
//  va_list ap;
//  va_start(ap, format);
//  vsnprintf(buf, sizeof(buf), format, ap);
//  for (char *p = &buf[0]; *p; p++) // emulate cooked mode for newlines
//  {
//    if (*p == '\n')
//      write('\r');
//    write(*p);
//  }
//  va_end(ap);
//}
//#ifdef F // check to see if F() macro is available
//void printf(const __FlashStringHelper *format, ...)
//{
//  char buf[PRINTF_BUF];
//  va_list ap;
//  va_start(ap, format);
//#ifdef __AVR__
//  vsnprintf_P(buf, sizeof(buf), (const char *)format, ap); // progmem for AVR
//#else
//  vsnprintf(buf, sizeof(buf), (const char *)format, ap); // for the rest of the world
//#endif
//  for (char *p = &buf[0]; *p; p++) // emulate cooked mode for newlines
//  {
//    if (*p == '\n')
//      write('\r');
//    write(*p);
//  }
//  va_end(ap);
//}
