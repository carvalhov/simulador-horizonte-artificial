#ifndef GYROSCOPE_H
#define GYROSCOPE_H

void gyro_init(void); 
void mpu6050_get_pitch_roll(float *pitch, float *roll);

#endif 
