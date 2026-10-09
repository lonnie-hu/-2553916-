#include "cmsis_os.h"
#include "io/led/led.hpp"
sp::LED led(&htim5);
extern "C" void led_task(void const * argument)
{
  (void)argument;

  led.start();

  for (;;) {
    for (int color = 0; color < 3; ++color) {
      for (int step = 0; step <= 100; ++step) {
        float level = step / 100.0f;
        led.set(color == 0 ? level : 0.0f, color == 1 ? level : 0.0f, color == 2 ? level : 0.0f);
        osDelay(10);
      }
      for (int step = 100; step >= 0; --step) {
        float level = step / 100.0f;
        led.set(color == 0 ? level : 0.0f, color == 1 ? level : 0.0f, color == 2 ? level : 0.0f);
        osDelay(10);
      }
    }
  }
}
