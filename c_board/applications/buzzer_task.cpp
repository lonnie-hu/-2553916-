#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6f);
extern "C" void buzzer_task(void const * argument)
{
  (void)argument;

  buzzer.set(4000.0f, 0.5f);

  for (int i = 0; i < 3; ++i) {
    buzzer.start();
    osDelay(120);
    buzzer.stop();
    osDelay(120);
  }

  for (;;) {
    osDelay(1000);
  }
}