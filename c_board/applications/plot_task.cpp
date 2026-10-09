#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "usart.h"

extern volatile float imu_pitch;
extern volatile float imu_roll;
extern volatile float imu_yaw_unwrapped;

extern "C" void plot_task(void const * argument)
{
  (void)argument;

  sp::Plotter plotter(&huart1, false);

  for (;;) {
    plotter.plot(imu_roll, imu_pitch, imu_yaw_unwrapped);
    osDelay(20);
  }
}
