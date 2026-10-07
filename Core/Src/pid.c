#include "pid.h"
#include <math.h>
#include <stddef.h>

void PID_Init(Pid_t *pid, float kp, float ki, float kd,
              float max_out, float max_iout, float deadband, PidType_t type)
{
  if (pid == NULL) return;

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->max_out  = max_out;
  pid->max_iout = max_iout;
  pid->deadband = deadband;
  pid->type     = type;

  PID_Clear(pid);
}

void PID_Clear(Pid_t *pid)
{
  if (pid == NULL) return;

  pid->target   = 0.0f;
  pid->feedback = 0.0f;
  pid->err      = 0.0f;
  pid->last_err = 0.0f;
  pid->prev_err = 0.0f;
  pid->integral = 0.0f;
  pid->output   = 0.0f;
}

void PID_SetKpPiKd(Pid_t *pid, float kp, float ki, float kd)
{
  if (pid == NULL) return;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

void PID_SetTarget(Pid_t *pid, float target)
{
  if (pid == NULL) return;
  pid->target = target;
}

/* Feedback MUST be refreshed by the caller every control period, otherwise
   err = target - 0 and the loop degenerates into an open-loop ramp. */
void PID_SetFeedback(Pid_t *pid, float feedback)
{
  if (pid == NULL) return;
  pid->feedback = feedback;
}

/* Positional or incremental PID calculation. */
void PID_Calc(Pid_t *pid)
{
  if (pid == NULL) return;

  pid->err = pid->target - pid->feedback;

  /* Deadband: treat small error as zero to avoid jitter. */
  if (fabsf(pid->err) < pid->deadband)
  {
    pid->err = 0.0f;
  }

  if (pid->type == PID_POSITIONAL)
  {
    /* ---- Positional PID ---- */
    pid->integral += pid->err;
    if (pid->integral >  pid->max_iout) pid->integral =  pid->max_iout;
    if (pid->integral < -pid->max_iout) pid->integral = -pid->max_iout;

    pid->output = pid->kp * pid->err
                + pid->ki * pid->integral
                + pid->kd * (pid->err - pid->last_err);

    if (pid->output >  pid->max_out) pid->output =  pid->max_out;
    if (pid->output < -pid->max_out) pid->output = -pid->max_out;

    pid->last_err = pid->err;
  }
  else
  {
    /* ---- Incremental PID ---- */
    float inc = pid->kp * (pid->err - pid->last_err)
              + pid->ki *  pid->err
              + pid->kd * (pid->err - 2.0f * pid->last_err + pid->prev_err);

    pid->output += inc;

    if (pid->output >  pid->max_out) pid->output =  pid->max_out;
    if (pid->output < -pid->max_out) pid->output = -pid->max_out;

    pid->prev_err = pid->last_err;
    pid->last_err = pid->err;
  }
}
