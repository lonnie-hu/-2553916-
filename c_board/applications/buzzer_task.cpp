#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

extern "C" void StartTask02(void const * argument)
{
  (void)argument;
  sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6f);
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