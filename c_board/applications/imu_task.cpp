#include <cstdio>

#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "main.h"
#include "usart.h"

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
extern "C" void imu_task(void const * argument)
{
  (void)argument;

  char status[] = "BMI088 initializing...\r\n";
  HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t *>(status), sizeof(status) - 1, 100);
  bmi088.init();

  for (;;) {
    bmi088.update();

    char line[160];
    const int length = std::snprintf(
      line, sizeof(line), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", bmi088.acc[0], bmi088.acc[1],
      bmi088.acc[2], bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2], bmi088.temp);
    if (length > 0 && length < static_cast<int>(sizeof(line))) {
      HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t *>(line), length, 100);
    }

    osDelay(100);
  }
}
