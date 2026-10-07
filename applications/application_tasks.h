#ifndef APPLICATION_TASKS_H
#define APPLICATION_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

void app_heartbeat(void);
void led_task(void const * argument);
void buzzer_task(void const * argument);
void imu_task(void const * argument);
void telemetry_task(void const * argument);

#ifdef __cplusplus
}
#endif

#endif
 