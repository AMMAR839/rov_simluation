/*
 * STM32 HAL example for the Gamantara ROV serial PWM bridge.
 *
 * This is a CubeIDE integration example, not a complete generated project.
 * Configure these peripherals in CubeMX first:
 * - UART for USB/serial, baud 115200.
 * - Timer PWM outputs for 6 ESC channels plus 1 gripper channel.
 * - Timer period suitable for 50 Hz servo PWM.
 *
 * Protocol from ROS:
 *   PWM,t1,t2,t3,t4,t5,t6,gripper\n
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"  /* Change this include to your STM32 family. */

#define PWM_NEUTRAL_US 1500
#define PWM_MIN_US 1100
#define PWM_MAX_US 1900
#define WATCHDOG_MS 500
#define RX_BUFFER_SIZE 96

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

static uint8_t rx_byte;
static char rx_line[RX_BUFFER_SIZE];
static uint8_t rx_index = 0;
static uint32_t last_command_ms = 0;

/*
 * Replace these mappings with your actual timer channels.
 * The timer must be configured so one timer tick equals 1 us.
 * If your timer tick is not 1 us, adjust set_pwm_us().
 */
static TIM_HandleTypeDef *esc_timer[7] = {
  &htim1, &htim1, &htim1, &htim1, &htim2, &htim2, &htim3
};

static uint32_t esc_channel[7] = {
  TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
  TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_1
};

static uint16_t clamp_pwm(long value)
{
  if (value < PWM_MIN_US) return PWM_MIN_US;
  if (value > PWM_MAX_US) return PWM_MAX_US;
  return (uint16_t)value;
}

static void set_pwm_us(uint8_t index, uint16_t pulse_us)
{
  if (index >= 7) return;
  __HAL_TIM_SET_COMPARE(esc_timer[index], esc_channel[index], pulse_us);
}

static void write_neutral(void)
{
  for (uint8_t i = 0; i < 6; ++i) {
    set_pwm_us(i, PWM_NEUTRAL_US);
  }
}

static bool parse_pwm_line(char *line, uint16_t *values, uint8_t expected)
{
  if (strncmp(line, "PWM,", 4) != 0) {
    return false;
  }

  uint8_t count = 0;
  char *token = strtok(line + 4, ",");
  while (token != NULL && count < expected) {
    values[count++] = clamp_pwm(strtol(token, NULL, 10));
    token = strtok(NULL, ",");
  }
  return count >= 6;
}

static void apply_pwm(uint16_t *values)
{
  for (uint8_t i = 0; i < 6; ++i) {
    set_pwm_us(i, values[i]);
  }
  set_pwm_us(6, values[6]);
  last_command_ms = HAL_GetTick();
}

void rov_bridge_start(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  write_neutral();
  set_pwm_us(6, 1100);
  last_command_ms = HAL_GetTick();
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

void rov_bridge_update(void)
{
  if (HAL_GetTick() - last_command_ms > WATCHDOG_MS) {
    write_neutral();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart2) {
    return;
  }

  if (rx_byte == '\n') {
    rx_line[rx_index] = '\0';
    uint16_t values[7] = {
      PWM_NEUTRAL_US, PWM_NEUTRAL_US, PWM_NEUTRAL_US,
      PWM_NEUTRAL_US, PWM_NEUTRAL_US, PWM_NEUTRAL_US,
      1100
    };
    if (parse_pwm_line(rx_line, values, 7)) {
      apply_pwm(values);
    }
    rx_index = 0;
  } else if (rx_byte != '\r') {
    if (rx_index < RX_BUFFER_SIZE - 1) {
      rx_line[rx_index++] = (char)rx_byte;
    } else {
      rx_index = 0;
    }
  }

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

