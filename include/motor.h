#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

/* frequency in hertz */
int init(int freq);

/* set 12 bit speed */
int set_motor_speed(int file, int *speed);

/* set twist */
int set_twist(int file, float w, float v_x, float v_y);

#endif
