#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "usart.h"

extern sp::BMI088 bmi088;
extern volatile float imu_yaw_unwrapped;

extern "C" void plot_task(void const * argument)
{
  (void)argument;

  sp::Plotter plotter(&huart1, false);

  for (;;) {
    plotter.plot(
      bmi088.acc[0], bmi088.acc[1], bmi088.acc[2], bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2],
      imu_yaw_unwrapped);
    osDelay(20);
  }
}
