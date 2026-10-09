#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "usart.h"

extern "C" void plot_task(void const * argument)
{
  (void)argument;

  sp::Plotter plotter(&huart1, false);

  for (;;) {
    plotter.plot(0.0f);
    osDelay(20);
  }
}
