#ifndef MOTOR_REPORT_HPP
#define MOTOR_REPORT_HPP
#include "motor_link.hpp"
void motor_report_record(const MotorState * motor, const RemoteState * remote,
                         const ImuYawState * imu, uint32_t now_ms);
void motor_report_print(void);
#endif
