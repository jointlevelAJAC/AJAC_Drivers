#include "pi_regulators.h"
#include "config.h"
#include "math.h"
#include "motor_config.h"

Cur_PI_Handler_t cur_pi_regulator_;
Vel_PI_Handler_t vel_pi_regulator_;

// ************************************* self min-max function ************************************
static float my_fmax(float x, float y)
{
    return (x > y) ? x : y;
}

static float my_fmin(float x, float y)
{
    return (x < y) ? x : y;
}

// ************************************* current loop pi ******************************************
//
// ************************************************************************************************
float sat_lower_bound_q, sat_upper_bound_q;
float vd_raw, vq_raw, vd, vq, v_dq_norm;
static void cur_pi_run(qd_f_t *iqd_error, qd_f_t *BMF, float vlimit)
{
    cur_pi_regulator_.Ka_out.d = cur_pi_regulator_.Ka * iqd_error->d;
    cur_pi_regulator_.Ka_out.q = cur_pi_regulator_.Ka * iqd_error->q;

    // nominal Kb_out
    cur_pi_regulator_.Kb_out.d += cur_pi_regulator_.Ka_out.d * cur_pi_regulator_.Kb_const;
    cur_pi_regulator_.Kb_out.q += cur_pi_regulator_.Ka_out.q * cur_pi_regulator_.Kb_const;

    // pi saturation, only q axis
    if (cur_pi_regulator_.Ka_out.q > 0)
    {
        sat_lower_bound_q = my_fmin(0.f, -vlimit + cur_pi_regulator_.Ka_out.q - BMF->q);
        sat_upper_bound_q = my_fmax(0.f, vlimit - cur_pi_regulator_.Ka_out.q - BMF->q);

        cur_pi_regulator_.Kb_out.q = my_fmax(sat_lower_bound_q, my_fmin(cur_pi_regulator_.Kb_out.q, sat_upper_bound_q));
    }
    else
    {
        sat_lower_bound_q = my_fmin(0.f, -vlimit - cur_pi_regulator_.Ka_out.q - BMF->q);
        sat_upper_bound_q = my_fmax(0.f, vlimit + cur_pi_regulator_.Ka_out.q - BMF->q);

        cur_pi_regulator_.Kb_out.q = my_fmax(sat_lower_bound_q, my_fmin(cur_pi_regulator_.Kb_out.q, sat_upper_bound_q));
    }

    float vq = cur_pi_regulator_.Ka_out.q + BMF->q + cur_pi_regulator_.Kb_out.q;
    float vd = cur_pi_regulator_.Ka_out.d + BMF->d + cur_pi_regulator_.Kb_out.d;

    float V_qd_norm = hypotf(vd, vq);
    if (V_qd_norm > vlimit)
    {
        cur_pi_regulator_.Vqd_out.d = vd * vlimit / V_qd_norm;
        cur_pi_regulator_.Vqd_out.q = vq * vlimit / V_qd_norm;
        cur_pi_regulator_.sat_flag = 1;
    }
    else
    {
        cur_pi_regulator_.Vqd_out.d = vd;
        cur_pi_regulator_.Vqd_out.q = vq;
        cur_pi_regulator_.sat_flag = 0;
    }
    //cur_pi_regulator_.Vqd_out.d = 3.;
    //cur_pi_regulator_.Vqd_out.q = 0.;
}

static void cur_set_ka(float value)
{
    cur_pi_regulator_.Ka = value;
}
static void cur_set_kb(float value)
{
    cur_pi_regulator_.Kb = value;
}
static void cur_pi_reset(void)
{
    cur_pi_regulator_.Ka_out.d = cur_pi_regulator_.Ka_out.q = 0;
    cur_pi_regulator_.Kb_out.d = cur_pi_regulator_.Kb_out.q = 0;
    cur_pi_regulator_.Vqd_out.q = cur_pi_regulator_.Vqd_out.d = 0;
}

void Current_PI_Handler_Init(void)
{
    cur_pi_regulator_.pfct_cur_run = cur_pi_run;
    cur_pi_regulator_.pfct_cur_setka = cur_set_ka;
    cur_pi_regulator_.pfct_cur_setkb = cur_set_kb;
    cur_pi_regulator_.pfct_reset_cur = cur_pi_reset;

    cur_pi_regulator_.pfct_cur_setka(K_a);
    cur_pi_regulator_.pfct_cur_setkb(K_b);
    cur_pi_regulator_.Kb_const = K_b * T_PWM;
}

// ************************************* velocity loop pi ******************************************
//
// ************************************************************************************************
static void vel_pi_run(float fb_vel_value)
{
    vel_pi_regulator_.vel_last_ref = vel_pi_regulator_.vel_ref;
    vel_pi_regulator_.error = vel_pi_regulator_.vel_ref - fb_vel_value;
    vel_pi_regulator_.Ka_out = vel_pi_regulator_.error * vel_pi_regulator_.Ka;
    vel_pi_regulator_.Kb_out += (vel_pi_regulator_.Ka_out * vel_pi_regulator_.Kb_const);
    vel_pi_regulator_.iq_ref = vel_pi_regulator_.Ka_out + vel_pi_regulator_.Kb_out;
}
static void vel_set_ka(float value)
{
    vel_pi_regulator_.Ka = value;
}
static void vel_set_kb(float value)
{
    vel_pi_regulator_.Kb = value;
}
static void vel_pi_reset(void)
{
    vel_pi_regulator_.iq_ref = 0;
    vel_pi_regulator_.vel_last_ref = vel_pi_regulator_.vel_ref = 0;
    vel_pi_regulator_.Ka_out = vel_pi_regulator_.Kb_out = vel_pi_regulator_.error = 0;
}

void Vel_PI_Handler_Init(void)
{
    vel_pi_regulator_.Ka = K_av;
    vel_pi_regulator_.Kb = K_bv;
    vel_pi_regulator_.control_mode = Idle_Mode;
    vel_pi_regulator_.Kb_const = K_bv * T_vel;
    vel_pi_regulator_.pfct_vel_run = vel_pi_run;
    vel_pi_regulator_.pfct_vel_setka = vel_set_ka;
    vel_pi_regulator_.pfct_vel_setkb = vel_set_kb;
    vel_pi_regulator_.pfct_vel_reset = vel_pi_reset;
}

Cur_PI_Handler_t *get_cur_pi_handler(void)
{
    return &cur_pi_regulator_;
}

Vel_PI_Handler_t *get_vel_pi_handler(void)
{
    return &vel_pi_regulator_;
}