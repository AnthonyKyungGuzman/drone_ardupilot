// #include "AC_AttitudeControl_Custom.h"
// #include <AP_HAL/AP_HAL.h>
// #include <AP_Math/AP_Math.h>

// // table of user settable parameters
// const AP_Param::GroupInfo AC_AttitudeControl_Custom::var_info[] = {
//     // parameters from parent vehicle
//     AP_NESTEDGROUPINFO(AC_AttitudeControl_Custom, 0),

//     // @Param: RAT_RLL_P
//     // @DisplayName: Roll axis rate controller P gain
//     // @Description: Roll axis rate controller P gain.  Corrects in proportion to the difference between the desired roll rate vs actual roll rate
//     // @Range: 0.01 0.5
//     // @Increment: 0.005
//     // @User: Standard

//     // @Param: RAT_RLL_I
//     // @DisplayName: Roll axis rate controller I gain
//     // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
//     // @Range: 0.01 2.0
//     // @Increment: 0.01
//     // @User: Standard

//     // @Param: RAT_RLL_IMAX
//     // @DisplayName: Roll axis rate controller I gain maximum
//     // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum that the I term will output
//     // @Range: 0 1
//     // @Increment: 0.01
//     // @User: Standard

//     // @Param: RAT_RLL_D
//     // @DisplayName: Roll axis rate controller D gain
//     // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
//     // @Range: 0.0 0.05
//     // @Increment: 0.001
//     // @User: Standard

//     // @Param: RAT_RLL_FF
//     // @DisplayName: Roll axis rate controller feed forward
//     // @Description: Roll axis rate controller feed forward
//     // @Range: 0 0.5
//     // @Increment: 0.001
//     // @User: Standard

//     // @Param: RAT_RLL_FLTT
//     // @DisplayName: Roll axis rate controller target frequency in Hz
//     // @Description: Roll axis rate controller target frequency in Hz
//     // @Range: 5 100
//     // @Increment: 1
//     // @Units: Hz
//     // @User: Standard

//     // @Param: RAT_RLL_FLTE
//     // @DisplayName: Roll axis rate controller error frequency in Hz
//     // @Description: Roll axis rate controller error frequency in Hz
//     // @Range: 0 100
//     // @Increment: 1
//     // @Units: Hz
//     // @User: Standard

//     // @Param: RAT_RLL_FLTD
//     // @DisplayName: Roll axis rate controller derivative frequency in Hz
//     // @Description: Roll axis rate controller derivative frequency in Hz
//     // @Range: 5 100
//     // @Increment: 1
//     // @Units: Hz
//     // @User: Standard

//     // @Param: RAT_RLL_SMAX
//     // @DisplayName: Roll slew rate limit
//     // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
//     // @Range: 0 200
//     // @Increment: 0.5
//     // @User: Advanced

//     AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_Custom, AC_PID),

//     // @Param: RAT_PIT_P
//     // @DisplayName: Pitch axis rate controller P gain
//     // @Description: Pitch axis rate controller P gain.  Corrects in proportion to the difference between the desired pitch rate vs actual pitch rate output
//     // @Range: 0.01 0.50
//     // @Increment: 0.005
//     // @User: Standard

//     // @Param: RAT_PIT_I
//     // @DisplayName: Pitch axis rate controller I gain
//     // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
//     // @Range: 0.01 2.0
//     // @Increment: 0.01
//     // @User: Standard

//     // @Param: RAT_PIT_IMAX
//     // @DisplayName: Pitch axis rate controller I gain maximum
//     // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum that the I term will output
//     // @Range: 0 1
//     // @Increment: 0.01
//     // @User: Standard

//     // @Param: RAT_PIT_D
//     // @DisplayName: Pitch axis rate controller D gain
//     // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
//     // @Range: 0.0 0.05
//     // @Increment: 0.001
//     // @User: Standard

//     // @Param: RAT_PIT_FF
//     // @DisplayName: Pitch axis rate controller feed forward
//     // @Description: Pitch axis rate controller feed forward
//     // @Range: 0 0.5
//     // @Increment: 0.001
//     // @User: Standard

//     // @Param: RAT_PIT_FLTT
//     // @DisplayName: Pitch axis rate controller target frequency in Hz
//     // @Description: Pitch axis rate controller target frequency in Hz
//     // @Range: 5 100
//     // @Increment: 1
//     // @Units: Hz
//     // @User: Standard

//     // @Param: RAT_PIT_FLTE
//     // @DisplayName: Pitch axis rate controller error frequency in Hz
//     // @Description: Pitch axis rate controller error frequency in Hz
//     // @Range: 0 100
//     // @Increment: 1
//     // @Units: Hz
//     // @User: Standard

//     // @Param: RAT_PIT_FLTD
//     // @DisplayName: Pitch axis rate controller derivative frequency in Hz
//     // @Description: Pitch axis rate controller derivative frequency in Hz
//     // @Range: 5 100
//     // @Increment: 1
//     // @Units: Hz
//     // @User: Standard

//     // @Param: RAT_PIT_SMAX
//     // @DisplayName: Pitch slew rate limit
//     // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
//     // @Range: 0 200
//     // @Increment: 0.5
//     // @User: Advanced

//     AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_Custom, AC_PID),

//     // @Param: RAT_YAW_P
//     // @DisplayName: Yaw axis rate controller P gain
//     // @Description: Yaw axis rate controller P gain.  Corrects in proportion to the difference between the desired yaw rate vs actual yaw rate
//     // @Range: 0.10 2.50
//     // @Increment: 0.005
//     // @User: Standard

//     // @Param: RAT_YAW_I
//     // @DisplayName: Yaw axis rate controller I gain
//     // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
//     // @Range: 0.010 1.0
//     // @Increment: 0.01
//     // @User: Standard

//     // @Param: RAT_YAW_IMAX
//     // @DisplayName: Yaw axis rate controller I gain maximum
//     // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum that the I term will output
//     // @Range: 0 1
//     // @Increment: 0.01
//     // @User: Standard

//     // @Param: RAT_YAW_D
//     // @DisplayName: Yaw axis rate controller D gain
//     // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
//     // @Range: 0.000 0.02
//     // @Increment: 0.001
//     // @User: Standard

//     // @Param: RAT_YAW_FF
//     // @DisplayName: Yaw axis rate controller feed forward
//     // @Description: Yaw axis rate controller feed forward
//     // @Range: 0 0.5
//     // @Increment: 0.001
//     // @User: Standard

//     // @Param: RAT_YAW_FLTT
//     // @DisplayName: Yaw axis rate controller target frequency in Hz
//     // @Description: Yaw axis rate controller target frequency in Hz
//     // @Range: 1 50
//     // @Increment: 1
//     // @Units: Hz
//     // @User: Standard

//     // @Param: RAT_YAW_FLTE
//     // @DisplayName: Yaw axis rate controller error frequency in Hz
//     // @Description: Yaw axis rate controller error frequency in Hz
//     // @Range: 0 20
//     // @Increment: 1
//     // @Units: Hz
//     // @User: Standard

//     // @Param: RAT_YAW_FLTD
//     // @DisplayName: Yaw axis rate controller derivative frequency in Hz
//     // @Description: Yaw axis rate controller derivative frequency in Hz
//     // @Range: 5 50
//     // @Increment: 1
//     // @Units: Hz
//     // @User: Standard

//     // @Param: RAT_YAW_SMAX
//     // @DisplayName: Yaw slew rate limit
//     // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
//     // @Range: 0 200
//     // @Increment: 0.5
//     // @User: Advanced

//     AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_Custom, AC_PID),

//     // @Param: THR_MIX_MIN
//     // @DisplayName: Throttle Mix Minimum
//     // @Description: Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
//     // @Range: 0.1 0.25
//     // @User: Advanced
//     AP_GROUPINFO("THR_MIX_MIN", 4, AC_AttitudeControl_Custom, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),

//     // @Param: THR_MIX_MAX
//     // @DisplayName: Throttle Mix Maximum
//     // @Description: Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
//     // @Range: 0.5 0.9
//     // @User: Advanced
//     AP_GROUPINFO("THR_MIX_MAX", 5, AC_AttitudeControl_Custom, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

//     // @Param: THR_MIX_MAN
//     // @DisplayName: Throttle Mix Manual
//     // @Description: Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)
//     // @Range: 0.1 0.9
//     // @User: Advanced
//     AP_GROUPINFO("THR_MIX_MAN", 6, AC_AttitudeControl_Custom, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

//     // @Param: THR_G_BOOST
//     // @DisplayName: Throttle-gain boost
//     // @Description: Throttle-gain boost ratio. A value of 0 means no boosting is applied, a value of 1 means full boosting is applied. Describes the ratio increase that is applied to angle P and PD on pitch and roll.
//     // @Range: 0 1
//     // @User: Advanced
//     AP_GROUPINFO("THR_G_BOOST", 7, AC_AttitudeControl_Custom, _throttle_gain_boost, 0.0f),

//     AP_GROUPEND
// };

// AC_AttitudeControl_Custom::AC_AttitudeControl_Custom(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors) :
//     AC_AttitudeControl(ahrs, aparm, motors),
//     _motors_multi(motors),
//     _pid_rate_roll(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ),
//     _pid_rate_pitch(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ),
//     _pid_rate_yaw(AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_YAW_D, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, AC_ATC_MULTI_RATE_YAW_FILT_HZ, 0.0f)
// {
//     AP_Param::setup_object_defaults(this, var_info);
//     if (_singleton != nullptr) {
//             AP_HAL::panic("Can only be one AC_AttitudeControl_Custom");
//         }
//         _singleton = this;
// }

// AC_AttitudeControl_Custom::~AC_AttitudeControl_Custom()
// {}

// // Update Alt_Hold angle maximum
// void AC_AttitudeControl_Custom::update_althold_lean_angle_max(float throttle_in)
// {
//     // calc maximum tilt angle based on throttle
//     float thr_max = _motors_multi.get_throttle_thrust_max();

//     // divide by zero check
//     if (is_zero(thr_max)) {
//         _althold_lean_angle_max = 0.0f;
//         return;
//     }

//     float althold_lean_angle_max = acosf(constrain_float(throttle_in / (AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
//     _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
// }

// void AC_AttitudeControl_Custom::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
// {
//     _throttle_in = throttle_in;
//     update_althold_lean_angle_max(throttle_in);
//     _motors.set_throttle_filter_cutoff(filter_cutoff);
//     if (apply_angle_boost) {
//         // Apply angle boost
//         throttle_in = get_throttle_boosted(throttle_in);
//     } else {
//         // Clear angle_boost for logging purposes
//         _angle_boost = 0.0f;
//     }
//     _motors.set_throttle(throttle_in);
//     _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));
// }

// void AC_AttitudeControl_Custom::set_throttle_mix_max(float ratio)
// {
//     ratio = constrain_float(ratio, 0.0f, 1.0f);
//     _throttle_rpy_mix_desired = (1.0f - ratio) * _thr_mix_min + ratio * _thr_mix_max;
// }

// // returns a throttle including compensation for roll/pitch angle
// // throttle value should be 0 ~ 1
// float AC_AttitudeControl_Custom::get_throttle_boosted(float throttle_in)
// {
//     if (!_angle_boost_enabled) {
//         _angle_boost = 0;
//         return throttle_in;
//     }
//     // inverted_factor is 1 for tilt angles below 60 degrees
//     // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

//     float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
//     float inverted_factor = constrain_float(10.0f * cos_tilt, 0.0f, 1.0f);
//     float cos_tilt_target = cosf(_thrust_angle);
//     float boost_factor = 1.0f / constrain_float(cos_tilt_target, 0.1f, 1.0f);

//     float throttle_out = throttle_in * inverted_factor * boost_factor;
//     _angle_boost = constrain_float(throttle_out - throttle_in, -1.0f, 1.0f);
//     return throttle_out;
// }

// // returns a throttle including compensation for roll/pitch angle
// // throttle value should be 0 ~ 1
// float AC_AttitudeControl_Custom::get_throttle_avg_max(float throttle_in)
// {
//     throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);
//     return MAX(throttle_in, throttle_in * MAX(0.0f, 1.0f - _throttle_rpy_mix) + _motors.get_throttle_hover() * _throttle_rpy_mix);
// }

// // update_throttle_gain_boost - boost angle_p/pd each cycle on high throttle slew
// void AC_AttitudeControl_Custom::update_throttle_gain_boost()
// {
//     // Boost PD and Angle P on very rapid throttle changes
//     if (_motors.get_throttle_slew_rate() > AC_ATTITUDE_CONTROL_THR_G_BOOST_THRESH) {
//         const float pd_boost = constrain_float(_throttle_gain_boost + 1.0f, 1.0, 2.0);
//         set_PD_scale_mult(Vector3f(pd_boost, pd_boost, 1.0f));

//         const float angle_p_boost = constrain_float((_throttle_gain_boost + 1.0f) * (_throttle_gain_boost + 1.0f), 1.0, 4.0);
//         set_angle_P_scale_mult(Vector3f(angle_p_boost, angle_p_boost, 1.0f));
//     }
// }

// // update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
// void AC_AttitudeControl_Custom::update_throttle_rpy_mix()
// {
//     // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
//     if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
//         // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
//         _throttle_rpy_mix += MIN(2.0f * _dt, _throttle_rpy_mix_desired - _throttle_rpy_mix);
//     } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
//         // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
//         _throttle_rpy_mix -= MIN(0.5f * _dt, _throttle_rpy_mix - _throttle_rpy_mix_desired);

//         // if the mix is still higher than that being used, reset immediately
//         const float throttle_hover = _motors.get_throttle_hover();
//         const float throttle_in = _motors.get_throttle();
//         const float throttle_out = MAX(_motors.get_throttle_out(), throttle_in);
//         float mix_used;
//         // since throttle_out >= throttle_in at this point we don't need to check throttle_in < throttle_hover
//         if (throttle_out < throttle_hover) {
//             mix_used = (throttle_out - throttle_in) / (throttle_hover - throttle_in);
//         } else {
//             mix_used = throttle_out / throttle_hover;
//         }

//         _throttle_rpy_mix = MIN(_throttle_rpy_mix, MAX(mix_used, _throttle_rpy_mix_desired));
//     }
//     _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
// }

// void AC_AttitudeControl_Custom::rate_controller_run()
// {
//     // boost angle_p/pd each cycle on high throttle slew
//     update_throttle_gain_boost();

//     // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
//     update_throttle_rpy_mix();

//     _ang_vel_body += _sysid_ang_vel_body;

//     Vector3f gyro_latest = _ahrs.get_gyro_latest();

//     _motors.set_roll(get_rate_roll_pid().update_all(_ang_vel_body.x, gyro_latest.x,  _dt, _motors.limit.roll, _pd_scale.x) + _actuator_sysid.x);
//     _motors.set_roll_ff(get_rate_roll_pid().get_ff());

//     _motors.set_pitch(get_rate_pitch_pid().update_all(_ang_vel_body.y, gyro_latest.y,  _dt, _motors.limit.pitch, _pd_scale.y) + _actuator_sysid.y);
//     _motors.set_pitch_ff(get_rate_pitch_pid().get_ff());

//     _motors.set_yaw(get_rate_yaw_pid().update_all(_ang_vel_body.z, gyro_latest.z,  _dt, _motors.limit.yaw, _pd_scale.z) + _actuator_sysid.z);
//     _motors.set_yaw_ff(get_rate_yaw_pid().get_ff()*_feedforward_scalar);

//     _sysid_ang_vel_body.zero();
//     _actuator_sysid.zero();

//     _pd_scale_used = _pd_scale;
//     _pd_scale = VECTORF_111;

//     control_monitor_update();
// }

// void AC_AttitudeControl_Custom::update()
// {

    
// }

#include <AP_Scripting/AP_Scripting_config.h>

// #if AP_SCRIPTING_ENABLED

#include "AC_AttitudeControl_Custom.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// 6DoF control is extracted from the existing copter code by treating desired angles as thrust angles rather than vehicle attitude.
// Vehicle attitude is then set separately, typically the vehicle would maintain 0 roll and pitch.
// rate commands result in the vehicle behaving as a ordinary copter.

// run lowest level body-frame rate controller and send outputs to the motors
void AC_AttitudeControl_Custom::rate_controller_run() {

    // pass current offsets to motors and run baseclass controller
    // motors require the offsets to know which way is up
    float roll_deg = roll_offset_deg;
    float pitch_deg = pitch_offset_deg;
    // if 6DoF control, always point directly up
    // this stops horizontal drift due to error between target and true attitude
    if (lateral_enable) {
        roll_deg = degrees(AP::ahrs().get_roll());
    }
    if (forward_enable) {
        pitch_deg = degrees(AP::ahrs().get_pitch());
    }
    _motors.set_roll_pitch(roll_deg,pitch_deg);

    AC_AttitudeControl_Multi::rate_controller_run();
}

/*
    override all input to the attitude controller and convert desired angles into thrust angles and substitute
*/

// Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl_Custom::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds) {

    set_forward_lateral(euler_pitch_angle_cd, euler_roll_angle_cd);

    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_euler_rate_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_rate_cds);
}

// Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
void AC_AttitudeControl_Custom::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw) {

    set_forward_lateral(euler_pitch_angle_cd, euler_roll_angle_cd);

    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_angle_cd, slew_yaw);
}

// Command a thrust vector and heading rate
void AC_AttitudeControl_Custom::input_thrust_vector_rate_heading(const Vector3f& thrust_vector, float heading_rate_cds, bool slew_yaw)
{
    // convert thrust vector to a roll and pitch angles
    // this negates the advantage of using thrust vector control, but works just fine
    Vector3f angle_target = attitude_from_thrust_vector(thrust_vector, _ahrs.yaw).to_vector312();

    input_euler_angle_roll_pitch_euler_rate_yaw(degrees(angle_target.x) * 100.0f, degrees(angle_target.y) * 100.0f, heading_rate_cds);
}

// Command a thrust vector, heading and heading rate
void AC_AttitudeControl_Custom::input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_angle_cd, float heading_rate_cds)
{
    // convert thrust vector to a roll and pitch angles
    Vector3f angle_target = attitude_from_thrust_vector(thrust_vector, _ahrs.yaw).to_vector312();

    // note that we are throwing away heading rate here
    input_euler_angle_roll_pitch_yaw(degrees(angle_target.x) * 100.0f, degrees(angle_target.y) * 100.0f, heading_angle_cd, true);
}

void AC_AttitudeControl_Custom::set_forward_lateral(float &euler_pitch_angle_cd, float &euler_roll_angle_cd)
{
    // pitch/forward
    if (forward_enable) {
        _motors.set_forward(-sinf(radians(euler_pitch_angle_cd * 0.01f)));
        euler_pitch_angle_cd = pitch_offset_deg * 100.0f;
    } else {
        _motors.set_forward(0.0f);
        euler_pitch_angle_cd += pitch_offset_deg * 100.0f;
    }
    euler_pitch_angle_cd = wrap_180_cd(euler_pitch_angle_cd);

    // roll/lateral
    if (lateral_enable) {
        _motors.set_lateral(sinf(radians(euler_roll_angle_cd * 0.01f)));
        euler_roll_angle_cd = roll_offset_deg * 100.0f;
    } else {
        _motors.set_lateral(0.0f);
        euler_roll_angle_cd += roll_offset_deg * 100.0f;
    }
    euler_roll_angle_cd = wrap_180_cd(euler_roll_angle_cd);
}

/*
    all other input functions should zero thrust vectoring
*/

// Command euler yaw rate and pitch angle with roll angle specified in body frame
// (used only by tailsitter quadplanes)
void AC_AttitudeControl_Custom::input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_euler_rate_yaw_euler_angle_pitch_bf_roll(plane_controls, euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_rate_cds);
}

// Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl_Custom::input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_euler_rate_roll_pitch_yaw(euler_roll_rate_cds, euler_pitch_rate_cds, euler_yaw_rate_cds);
}

// Command an angular velocity with angular velocity feedforward and smoothing
void AC_AttitudeControl_Custom::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

// Command an angular velocity with angular velocity feedforward and smoothing
void AC_AttitudeControl_Custom::input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw_2(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

// Command an angular velocity with angular velocity smoothing using rate loops only with integrated rate error stabilization
void AC_AttitudeControl_Custom::input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw_3(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

// Command an angular step (i.e change) in body frame angle
void AC_AttitudeControl_Custom::input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_angle_step_bf_roll_pitch_yaw(roll_angle_step_bf_cd, pitch_angle_step_bf_cd, yaw_angle_step_bf_cd);
}

// Command a Quaternion attitude with feedforward and smoothing
// attitude_desired_quat: is updated on each time_step (_dt) by the integral of the angular velocity
// not used anywhere in current code, panic in SITL so this implementation is not overlooked
void AC_AttitudeControl_Custom::input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_target) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP_HAL::panic("input_quaternion not implemented AC_AttitudeControl_Custom");
#endif

    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_quaternion(attitude_desired_quat, ang_vel_target);
}

void AC_AttitudeControl_Custom::update()
{

    
}


AC_AttitudeControl_Custom *AC_AttitudeControl_Custom::_singleton = nullptr;

// #endif // AP_SCRIPTING_ENABLED

