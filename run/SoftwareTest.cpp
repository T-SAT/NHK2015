#include "SoftwareTest.h"

SoftwareTest SoftTest;

void SoftwareTest::no_error(void)
{
  Serial.println("no problem");
  Serial.flush();
  abort();
}

bool SoftwareTest::is_inRangeEqual(float min_value, float target_value, float max_value)
{
  if (target_value >= min_value && target_value <= max_value)
    return (true);
  else
    return (false);
}

bool SoftwareTest::is_inRangeNotEqual(float min_value, float target_value, float max_value)
{
  if (target_value > min_value && target_value < max_value)
    return (true);
  else
    return (false);
}

//使用例
void SoftwareTest::printd(const char *tab, const char* format, ...)
{
  const char *p, *j, *tmp;
  va_list args;
  int tmpd, i;
  float tmpf;
  double tmpl;
  char tmpc, ch;
  long unsigned int tmpu;

  tmp = tab;
  Serial.println();
  va_start(args, format);
  Serial.println("-----------------------------------------------------");
  Serial.println(tab);
  Serial.println("-----------------------------------------------------");

  for (j = tab, i = 0; i < 2; ++j) {
    if (*j == '>')
      i++;
    else if (*j == '\0')
      break;
  }

  for (p = format; *p != '\0'; ++p) {
    switch (*p) {
      case 'd':
        tmpd = va_arg(args, int);
        for (; *j != ',' && *j != '\0'; j++) {
          Serial.print(*j);
        }
        j++;
        Serial.print(" = ");
        Serial.println(tmpd);
        break;

      case 'f':
        tmpf = va_arg(args, float);
        for (; *j != ',' && *j != '\0'; j++) {
          Serial.print(*j);
        }
        j++;
        Serial.print(" = ");
        Serial.println(tmpf);
        break;

      case 'c':
        tmpc = va_arg(args, int);
        for (; *j != ',' && *j != '\0'; j++) {
          Serial.print(*j);
        }
        j++;
        Serial.print(" = ");
        Serial.println(tmpc);
        break;

      case 'u':
        tmpu = va_arg(args, long unsigned int);
        for (; *j != ',' && *j != '\0'; j++) {
          Serial.print(*j);
        }
        j++;
        Serial.print(" = ");
        Serial.println(tmpu);
        break;

      default:
        Serial.println("The wrong type specified!");
        break;
    }
  }

  Serial.println("-----------------------------------------------------");
  Serial.println();
  va_end(args);

  while (1) {
    ch = Serial.read();
    if (ch == 'n')
      break;
  }

}

void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
  // transmit diagnostic informations through serial link.
  Serial.println("Assertion failed");
  Serial.print("Function: "); Serial.println(__func);
  Serial.print("File: ");     Serial.println(__file);
  Serial.print("LineNo: ");  Serial.println(__lineno, DEC);
  Serial.print("SexPoint: ");     Serial.println(__sexp);
  Serial.flush();
  // abort program execution.

  abort();
}
