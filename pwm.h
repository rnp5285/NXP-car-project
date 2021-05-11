#ifndef _PWM_H_
#define _PWM_H_

// === DC Motor ===
// PWM frequency
#define PWM_DCMOT_FREQ    (10000u)
// Minimum and maximum duty
#define MAX_DCMOT_DUTY    (100.0f)
#define MIN_DCMOT_DUTY    (-100.0f)
// === Servo Motor ===
// PWM frequency
#define PWM_SERVO_FREQ    (50u)
// PWM correction duty
#define PWM_SERVO_CORRECT (0.0f)
// Maximum and minimum duty
#define MIN_SERVO_DUTY    (5.0f + PWM_SERVO_CORRECT)
#define MAX_SERVO_DUTY    (10.0f + PWM_SERVO_CORRECT)
// Center servo duty
#define CTR_SERVO_DUTY    ((MAX_SERVO_DUTY+MIN_SERVO_DUTY)/2.0f)

void SetDCMotDuty(float lduty, float rduty);
void SetServoDuty(float duty);

//uint32_t servo_ready(void);

void InitDCMotPWM(void);
void InitServoPWM(void);

#endif /* PWM_H_ */
