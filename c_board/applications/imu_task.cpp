#include <cmath>

#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "main.h"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"

namespace
{

// BMI088 axes to C board axes, as documented by the middleware example.
constexpr float kBoardAxes[3][3] = {
  {0.0f, -1.0f, 0.0f},
  {1.0f, 0.0f, 0.0f},
  {0.0f, 0.0f, 1.0f},
};

}  // namespace

sp::BMI088 bmi088(
  &hspi1, CS1_ACC_GPIO_Port, CS1_ACC_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, kBoardAxes);
sp::Mahony mahony(0.002f);
sp::AngleUnwrapper yaw_unwrapper;
volatile float imu_yaw_unwrapped = 0.0f;
volatile float imu_yaw_speed = 0.0f;
volatile uint32_t imu_last_update_ms = 0;
volatile bool imu_ready = false;

extern "C" void imu_task(void const * argument)
{
  (void)argument;

  bmi088.init();

  for (;;) {
    bmi088.update();
    mahony.update(bmi088.acc, bmi088.gyro);
    const float yaw = yaw_unwrapper.update(mahony.yaw);
    const bool valid = std::isfinite(yaw) && std::isfinite(mahony.vyaw);
    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    imu_yaw_unwrapped = yaw;
    imu_yaw_speed = mahony.vyaw;
    imu_last_update_ms = osKernelSysTick();
    imu_ready = valid;
    __set_PRIMASK(irq_state);

    osDelay(2);
  }
}
