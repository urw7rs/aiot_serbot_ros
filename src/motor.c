#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <math.h>

#define I2C_ADDR 0x40

#define MODE1 0x00

#define LED0_OFF_L 0x08
#define LED1_OFF_L 0x0c
#define LED2_OFF_L 0x10
#define LED3_OFF_L 0x14
#define LED4_OFF_L 0x18
#define LED5_OFF_L 0x1c

#define PRE_SCALE 0xfe

#define MIN_FREQ 24
#define MAX_FREQ 1526

#define OSC_CLK 25000000

#define BASE_D 40.0
#define WHEEL_R 10.0
#define MOTOR_RATIO 1000

#include "motor.h"

/* frequency in hertz */
int init(int pwm_freq) {
  if (pwm_freq < MIN_FREQ || pwm_freq > MAX_FREQ)
    return -1;

  int file = open("/dev/i2c-0", O_RDWR);

  // set I2C slave address
  if (ioctl(file, I2C_SLAVE, I2C_ADDR) < 0)
      return -1;

  // enable sleep
  int32_t oldmode = i2c_smbus_read_byte_data(file, MODE1);
  i2c_smbus_write_byte_data(file, MODE1, oldmode | 0x10);

  // set prescale value according to Equation 1
  u_int8_t prescale_value = round(OSC_CLK / (pwm_freq * 4096.0)) - 1;
  i2c_smbus_write_byte_data(file, PRE_SCALE, prescale_value);

  // disable sleep and enable register auto increment
  u_int8_t newmode = oldmode & 0xef | 0xa1;
  i2c_smbus_write_byte_data(file, MODE1, newmode);

  return file;
}

/* set 12 bit speed */
int set_motor_speed(int file, int *speed) {
  u_int16_t data[] = {0, 0, 0, 0, 0, 0};

  for (int i = 0; i < 3; i++) {
    u_int16_t low = 0;
    u_int16_t high = 0;

    if (speed[i] < 0) {
      low = (-speed[i]) & 0x0fff;
      high = 0x0000;
    } else {
      low = 0x0000;
      high = speed[i] & 0x0fff;
    }

    data[2 * i] = low;
    data[2 * i + 1] = high;
  }

  u_int8_t pca9685_regs[6] = {
    LED0_OFF_L, LED1_OFF_L,
    LED2_OFF_L, LED3_OFF_L,
    LED4_OFF_L, LED5_OFF_L
  };

  for (int i = 0; i < 6; i++) {
    int32_t res = i2c_smbus_write_word_data(file, pca9685_regs[i], data[i]);
    if (res < 0) {
      for (int j = 0; j < i; j++)
        res = i2c_smbus_write_word_data(file, pca9685_regs[i], data[i]);

      return -1;
    }
  }

  return 0;
}

int set_twist(int file,
              float w, float v_x, float v_y) {
  float U[] = {0.0, 0.0, 0.0};
  float V[] = {w, v_x, v_y};

  float H[3][3] = {
    { -BASE_D, 0.5, sin(M_PI / 3) },
    { -BASE_D, 0.5, -sin(M_PI / 3) },
    { -BASE_D, -1, 0.0 }
  };

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      U[i] += H[i][j] * V[j];

    U[i] = U[i] / WHEEL_R * MOTOR_RATIO;
  }

  int speed[] = {U[0], U[1], U[2]};

  set_motor_speed(file, speed);

  return 0;
}
