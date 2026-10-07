#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

/* PID output type */
typedef enum
{
  PID_POSITIONAL = 0,
  PID_INCREMENTAL = 1
} PidType_t;

typedef struct
{
  float kp;
  float ki;
  float kd;

  float target;
  float feedback;
  float err;

  float last_err;
  float prev_err;
  float integral;

  float output;
  float max_out;
  float max_iout;
  float deadband;

  PidType_t type;
} Pid_t;

void PID_Init(Pid_t *pid, float kp, float ki, float kd,
              float max_out, float max_iout, float deadband, PidType_t type);
void PID_Clear(Pid_t *pid);
void PID_SetKpPiKd(Pid_t *pid, float kp, float ki, float kd);
void PID_SetTarget(Pid_t *pid, float target);
void PID_SetFeedback(Pid_t *pid, float feedback);
void PID_Calc(Pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
