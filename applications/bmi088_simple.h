#ifndef BMI088_SIMPLE_H
#define BMI088_SIMPLE_H


int bmi088_init(void);

int bmi088_read(float acc[3], float gyro[3]);

#endif
