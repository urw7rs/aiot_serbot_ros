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

int linux_kbhit(void){
  struct termios oldt, newt;
    int ch;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag = ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

void start_serial(void){
  int serial_port = open("/dev/ttyUSB0", O_RDWR);

    if (serial_port < 0) {
      printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    struct termios tty;

    if (tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }


    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
>>  tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_iflag &= ~ISIG;
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, B115200);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
    }


    char read_buf [256];
    memset(&read_buf, '\0', sizeof(read_buf));

    // Write to serial port
--  char* msg = "1.2,3.4\n";
    int cnt = 0;
    char buffer[256];
    unsigned char key[10];
    unsigned char key_mode = 's';
    float steering = 0.0;
    float speed = 0.0;
    int loop_cnt = 1;

    while (key_mode != 'q') {
  //    key[0] = fgetc(stdin);
      key[0] = linux_kbhit();
      int i_key = key[0];
      if(key[0] >= 'a' && key[0] <= 'z'){
        key_mode = key[0];
    }
  //    if(key[0] == 's'){
  //      key_mode = 's'; // plus steering mode
  //    }
  //    else if (key[0] == 'd'){
  //      key_mode = 'd'; // minus steering mode
  //    }
  //    else if (key[0] == 'k'){
  //      key_mode = 'k'; // speed mode
  //    }
  //    else if (key[0] == 'l'){
  //      key_mode = 'l'; // loop_cnt 지정 mode(유지시간)
  //    }
      int speed_cnt = 0;
      if(key[0] >= '0'&& key[0] <= '9'){
        if(key_mode == 's'){
          steering = i_key - 48;
        }
        else if (key_mode == 'd'){
          steering = 48 - i_key;
        }
        else if (key_mode == 'k'){
          speed_cnt = loop_cnt;
          speed = i_key - 48;
        }
        else if (key_mode == 'l'){
          loop_cnt = i_key - 48;
        }
      }
      for (int i = 0; i<speed_cnt; i++){ // loop_cnt초 만큼 loop
      sprintf(buffer,"%d,%.1f,%.1f\n",cnt++, steering, speed);
        printf("Send : %s",buffer);
        write(serial_port, buffer, strlen(buffer));
        sleep(1);
  //    usleep(100000);
        read(serial_port, &read_buf, sizeof(read_buf));
        printf("%s\n",read_buf);
      }
      // 속도 0으로 멈춤
        sprintf(buffer,"%d,%.1f,%.1f\n",cnt++, steering, 0.0);
        printf("Send : %s",buffer);
        write(serial_port, buffer, strlen(buffer));
        sleep(1);
  //    usleep(100000);
        read(serial_port, &read_buf, sizeof(read_buf));
        printf("%s\n",read_buf);

    }

    // Allocate memory for read buffer, set size according to your needs

    // Normally you wouldn't do this memset() call, but since we will just receive
    // ASCII data for this example, we'll set everything to 0 so we can
    // call printf() easily.

    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1   to signal an error.
    if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
      return 1;
    }

    // Here we assume we received ASCII data, but you might be sending raw bytes (in that case  , don't try and
    // print it to the screen like this!)
    printf("Read %i bytes. Received message: %s", num_bytes, read_buf);

    close(serial_port);
    return 0; // success
}
