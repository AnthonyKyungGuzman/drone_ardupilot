// #pragma once

// /// @file    AC_AttitudeControl_Custom.h
// /// @brief   ArduCopter attitude control library 

// #include "AC_AttitudeControl.h"
// #include <AP_Motors/AP_MotorsMulticopter.h>

// // default rate controller PID gains
// #ifndef AC_ATC_MULTI_RATE_RP_P
//   # define AC_ATC_MULTI_RATE_RP_P           0.135f
// #endif
// #ifndef AC_ATC_MULTI_RATE_RP_I
//   # define AC_ATC_MULTI_RATE_RP_I           0.135f
// #endif
// #ifndef AC_ATC_MULTI_RATE_RP_D
//   # define AC_ATC_MULTI_RATE_RP_D           0.0036f
// #endif
// #ifndef AC_ATC_MULTI_RATE_RP_IMAX
//  # define AC_ATC_MULTI_RATE_RP_IMAX         0.5f
// #endif
// #ifndef AC_ATC_MULTI_RATE_RP_FILT_HZ
//  # define AC_ATC_MULTI_RATE_RP_FILT_HZ      20.0f
// #endif
// #ifndef AC_ATC_MULTI_RATE_YAW_P
//  # define AC_ATC_MULTI_RATE_YAW_P           0.180f
// #endif
// #ifndef AC_ATC_MULTI_RATE_YAW_I
//  # define AC_ATC_MULTI_RATE_YAW_I           0.018f
// #endif
// #ifndef AC_ATC_MULTI_RATE_YAW_D
//  # define AC_ATC_MULTI_RATE_YAW_D           0.0f
// #endif
// #ifndef AC_ATC_MULTI_RATE_YAW_IMAX
//  # define AC_ATC_MULTI_RATE_YAW_IMAX        0.5f
// #endif
// #ifndef AC_ATC_MULTI_RATE_YAW_FILT_HZ
//  # define AC_ATC_MULTI_RATE_YAW_FILT_HZ     2.5f
// #endif


// class AC_AttitudeControl_Custom : public AC_AttitudeControl {
// public:
// 	AC_AttitudeControl_Custom(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors);

// 	// empty destructor to suppress compiler warning
// 	~AC_AttitudeControl_Custom();

//     static AC_AttitudeControl_Custom *get_singleton() {
//         return _singleton;
//     }

//     // pid accessors
//     AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
//     AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
//     AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }

//     // Update Alt_Hold angle maximum
//     void update_althold_lean_angle_max(float throttle_in) override;

//     // Set output throttle
//     void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

//     // calculate total body frame throttle required to produce the given earth frame throttle
//     float get_throttle_boosted(float throttle_in);

//     // set desired throttle vs attitude mixing (actual mix is slewed towards this value over 1~2 seconds)
//     //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
//     //  has no effect when throttle is above hover throttle
//     void set_throttle_mix_min() override { _throttle_rpy_mix_desired = _thr_mix_min; }
//     void set_throttle_mix_man() override { _throttle_rpy_mix_desired = _thr_mix_man; }
//     void set_throttle_mix_max(float ratio) override;
//     void set_throttle_mix_value(float value) override { _throttle_rpy_mix_desired = _throttle_rpy_mix = value; }
//     float get_throttle_mix(void) const override { return _throttle_rpy_mix; }

//     // are we producing min throttle?
//     bool is_throttle_mix_min() const override { return (_throttle_rpy_mix < 1.25f * _thr_mix_min); }

//     // run lowest level body-frame rate controller and send outputs to the motors
//     void rate_controller_run() override;

//     void update(); //AKGL

//     // user settable parameters
//     static const struct AP_Param::GroupInfo var_info[];

// protected:

//     // boost angle_p/pd each cycle on high throttle slew
//     void update_throttle_gain_boost();

//     // update_throttle_rpy_mix - updates thr_low_comp value towards the target
//     void update_throttle_rpy_mix();

//     // get maximum value throttle can be raised to based on throttle vs attitude prioritisation
//     float get_throttle_avg_max(float throttle_in);

//     AP_MotorsMulticopter& _motors_multi;
//     AC_PID                _pid_rate_roll;
//     AC_PID                _pid_rate_pitch;
//     AC_PID                _pid_rate_yaw;

//     AP_Float              _thr_mix_man;     // throttle vs attitude control prioritisation used when using manual throttle (higher values mean we prioritise attitude control over throttle)
//     AP_Float              _thr_mix_min;     // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
//     AP_Float              _thr_mix_max;     // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)

//     // angle_p/pd boost multiplier
//     AP_Float              _throttle_gain_boost;

//     static AC_AttitudeControl_Custom *_singleton;
// };


#pragma once
// #if AP_SCRIPTING_ENABLED

#include "AC_AttitudeControl_Multi.h"

class AC_AttitudeControl_Custom : public AC_AttitudeControl_Multi {
public:
    AC_AttitudeControl_Custom(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors):
        AC_AttitudeControl_Multi(ahrs,aparm,motors) {

        if (_singleton != nullptr) {
            AP_HAL::panic("Can only be one AC_AttitudeControl_Custom");
        }
        _singleton = this;
    }

    static AC_AttitudeControl_Custom *get_singleton() {
        return _singleton;
    }

    // Command a Quaternion attitude with feedforward and smoothing
    // attitude_desired_quat: is updated on each time_step (_dt) by the integral of the angular velocity
    // not used anywhere in current code, panic so this implementation is not overlooked
    void input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_target) override;
    /*
        override input functions to attitude controller and convert desired angles into thrust angles and substitute for offset angles
    */

    // Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
    void input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)  override;

    // Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
    void input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw) override;

    // Command a thrust vector in the earth frame and a heading angle and/or rate
    void input_thrust_vector_rate_heading(const Vector3f& thrust_vector, float heading_rate_cds, bool slew_yaw = true) override;
    void input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_angle_cd, float heading_rate_cds) override;

    /*
        all other input functions should zero thrust vectoring and behave as a normal copter
    */

    // Command euler yaw rate and pitch angle with roll angle specified in body frame
    // (used only by tailsitter quadplanes)
    void input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds) override;

    // Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
    void input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds) override;

     // Command an angular velocity with angular velocity feedforward and smoothing
    void input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) override;

    // Command an angular velocity with angular velocity feedforward and smoothing
    void input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) override;

    // Command an angular velocity with angular velocity smoothing using rate loops only with integrated rate error stabilization
    void input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) override;

    // Command an angular step (i.e change) in body frame angle
    void input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd) override;

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run() override;

    // limiting lean angle based on throttle makes no sense for 6DoF, always allow 90 deg, return in centi-degrees
    float get_althold_lean_angle_max_cd() const override { return 9000.0f; }

    // set the attitude that will be used in 6DoF flight
    void set_offset_roll_pitch(float roll_deg, float pitch_deg) {
        roll_offset_deg = roll_deg;
        pitch_offset_deg = pitch_deg;
    }

    // these flags enable or disable lateral or forward thrust, with both disabled the vehicle acts like a traditional copter
    // it will roll and pitch to move, its also possible to enable only forward or lateral to suit the vehicle configuration.
    // by default the vehicle is full 6DoF, these can be set in flight via scripting
    void set_forward_enable(bool b) {
        forward_enable = b;
    }
    void set_lateral_enable(bool b) {
        lateral_enable = b;
    }

    void update(); //AKGL

private:

    void set_forward_lateral(float &euler_pitch_angle_cd, float &euler_roll_angle_cd);

    float roll_offset_deg;
    float pitch_offset_deg;

    bool forward_enable = true;
    bool lateral_enable = true;

    static AC_AttitudeControl_Custom *_singleton;

};

// #endif // AP_SCRIPTING_ENABLED
