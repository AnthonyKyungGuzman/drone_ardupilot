#include "Copter.h"

#if MODE_CUSTOM_ENABLED == ENABLED

/*
 * Init and run calls for custom flight mode
 */

static Vector3p custom_pos_target_cm;       // position target (used by posvel controller only)
bool custom_pos_terrain_alt;                // true if custom_pos_target_cm.z is an alt above terrain
static Vector3f custom_vel_target_cms;      // velocity target (used by pos_vel_accel controller and vel_accel controller)
static Vector3f custom_accel_target_cmss;   // acceleration target (used by pos_vel_accel controller vel_accel controller and accel controller)
static uint32_t update_time_ms;             // system time of last target update to pos_vel_accel, vel_accel or accel controller

struct {
    uint32_t update_time_ms;
    Quaternion attitude_quat;
    Vector3f ang_vel;
    float yaw_rate_cds;
    float climb_rate_cms;   // climb rate in cms.  Used if use_thrust is false
    float thrust;           // thrust from -1 to 1.  Used if use_thrust is true
    bool use_yaw_rate;
    bool use_thrust;
} static custom_angle_state;

struct Custom_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that custom is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where custom mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} custom_limit;

// init - initialise custom controller
bool ModeCustom::init(bool ignore_checks)
{
    copter.g2.custom_timeout.set(3.0); //obstained from Parameters got reference from guided mode AKGL
    copter.g2.custom_options.set(0); // set for code standard, but there is only one option   AKGL
    // start in velaccel control mode
    //TODO_AKGL: Initialise pos control
    // velaccel_control_start();
    // custom_vel_target_cms.zero();
    // custom_accel_target_cmss.zero();
    // send_notification = false;

    // clear pause state when entering custom mode
    _paused = false;

    return true;
}


// run - runs the custom controller
// should be called at 100hz or more
void ModeCustom::run() //AKGL call control functions in here
{
    pos_control->update();
    attitude_control->update();
}

// // run - runs the custom controller
// // should be called at 100hz or more
// void ModeCustom::run() //AKGL call control functions in here
// {
//     // run pause control if the vehicle is paused
//     if (_paused) {
//         pause_control_run();
//         return;
//     }

//     // call the correct auto controller
//     switch (custom_mode) {

//     case SubMode::TakeOff:
//         // run takeoff controller
//         takeoff_run();
//         break;

//     case SubMode::WP:
//         // run waypoint controller
//         wp_control_run();
//         if (send_notification && wp_nav->reached_wp_destination()) {
//             send_notification = false;
//             gcs().send_mission_item_reached_message(0);
//         }
//         break;

//     case SubMode::Pos:
//         // run position controller
//         pos_control_run();
//         break;

//     case SubMode::Accel:
//         accel_control_run();
//         break;

//     case SubMode::VelAccel:
//         velaccel_control_run();
//         break;

//     case SubMode::PosVelAccel:
//         posvelaccel_control_run();
//         break;

//     case SubMode::Angle:
//         angle_control_run();
//         break;
//     }
//  }

bool ModeCustom::allows_arming(AP_Arming::Method method) const
{
    // always allow arming from the ground station or scripting
    if (method == AP_Arming::Method::MAVLINK || method == AP_Arming::Method::SCRIPTING) {
        return true;
    }

    // optionally allow arming from the transmitter
    return copter.g2.custom_options.get(); //AKGL TODO: remove function 
};

#if WEATHERVANE_ENABLED == ENABLED
bool ModeCustom::allows_weathervaning() const
{
    // return (copter.g2.custom_options.get() & (uint32_t)Options::AllowWeatherVaning) != 0;
    return copter.g2.custom_options.get(); //AKGL TODO: remove function later

}
#endif

// initialises position controller to implement take-off
// takeoff_alt_cm is interpreted as alt-above-home (in cm) or alt-above-terrain if a rangefinder is available
bool ModeCustom::do_user_takeoff_start(float takeoff_alt_cm)
{
    // calculate target altitude and frame (either alt-above-ekf-origin or alt-above-terrain)
    int32_t alt_target_cm;
    bool alt_target_terrain = false;
    if (wp_nav->rangefinder_used_and_healthy() &&
        wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER &&
        takeoff_alt_cm < copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)) {
        // can't takeoff downwards
        if (takeoff_alt_cm <= copter.rangefinder_state.alt_cm) {
            return false;
        }
        // provide target altitude as alt-above-terrain
        alt_target_cm = takeoff_alt_cm;
        alt_target_terrain = true;
    } else {
        // interpret altitude as alt-above-home
        Location target_loc = copter.current_loc;
        target_loc.set_alt_cm(takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);

        // provide target altitude as alt-above-ekf-origin
        if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
            // this should never happen but we reject the command just in case
            return false;
        }
    }

    custom_mode = SubMode::TakeOff;

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // clear i term when we're taking off
    pos_control->init_z_controller();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff_start(alt_target_cm, alt_target_terrain);

    // record takeoff has not completed
    takeoff_complete = false;

    return true;
}

// initialise custom mode's waypoint navigation controller
void ModeCustom::wp_control_start()
{
    // set to position control mode
    custom_mode = SubMode::WP;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);
    if (!wp_nav->set_wp_destination(stopping_point, false)) {
        // this should never happen because terrain data is not used
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// run custom mode's waypoint navigation controller
void ModeCustom::wp_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    // attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading()); // AKGL removed to avoid error at compile time
}

// initialise position controller
void ModeCustom::pva_control_start()
{
    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise velocity controller
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    // initialise yaw
    auto_yaw.set_mode_to_default(false);

    // initialise terrain alt
    custom_pos_terrain_alt = false;
}

// initialise custom mode's position controller
void ModeCustom::pos_control_start()
{
    // set to position control mode
    custom_mode = SubMode::Pos;

    // initialise position controller
    pva_control_start();
}

// initialise custom mode's velocity controller
void ModeCustom::accel_control_start()
{
    // set custom_mode to velocity controller
    custom_mode = SubMode::Accel;

    // initialise position controller
    pva_control_start();
}

// initialise custom mode's velocity and acceleration controller
void ModeCustom::velaccel_control_start()
{
    // set custom_mode to velocity controller
    custom_mode = SubMode::VelAccel;

    // initialise position controller
    pva_control_start();
}

// initialise custom mode's position, velocity and acceleration controller
void ModeCustom::posvelaccel_control_start()
{
    // set custom_mode to velocity controller
    custom_mode = SubMode::PosVelAccel;

    // initialise position controller
    pva_control_start();
}

bool ModeCustom::is_taking_off() const
{
    return custom_mode == SubMode::TakeOff && !takeoff_complete;
}

bool ModeCustom::set_speed_xy(float speed_xy_cms)
{
    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_xy(speed_xy_cms, wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(speed_xy_cms, wp_nav->get_wp_acceleration());
    return true;
}

bool ModeCustom::set_speed_up(float speed_up_cms)
{
    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), speed_up_cms, wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), speed_up_cms, wp_nav->get_accel_z());
    return true;
}

bool ModeCustom::set_speed_down(float speed_down_cms)
{
    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(speed_down_cms, wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(speed_down_cms, wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    return true;
}

// initialise custom mode's angle controller
void ModeCustom::angle_control_start()
{
    // set custom_mode to velocity controller
    custom_mode = SubMode::Angle;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise targets
    custom_angle_state.update_time_ms = millis();
    custom_angle_state.attitude_quat.initialise();
    custom_angle_state.ang_vel.zero();
    custom_angle_state.climb_rate_cms = 0.0f;
    custom_angle_state.yaw_rate_cds = 0.0f;
    custom_angle_state.use_yaw_rate = false;

    // pilot always controls yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
}

// set_destination - sets custom mode's target destination
// Returns true if the fence is enabled and custom waypoint is within the fence
// else return false if the waypoint is outside the fence
bool ModeCustom::set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool terrain_alt)
{
#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, terrain_alt ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // if configured to use wpnav for position control
    if (use_wpnav_for_position_control()) {
        // ensure we are in position control mode
        if (custom_mode != SubMode::WP) {
            wp_control_start();
        }

        // set yaw state
        set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

        // no need to check return status because terrain data is not used
        wp_nav->set_wp_destination(destination, terrain_alt);

        // log target
        // copter.Log_Write_Custom_Position_Target(custom_mode, destination, terrain_alt, Vector3f(), Vector3f());
        send_notification = true;
        return true;
    }

    // if configured to use position controller for position control
    // ensure we are in position control mode
    if (custom_mode != SubMode::Pos) {
        pos_control_start();
    }

    // initialise terrain following if needed
    if (terrain_alt) {
        // get current alt above terrain
        float origin_terr_offset;
        if (!wp_nav->get_terrain_offset(origin_terr_offset)) {
            // if we don't have terrain altitude then stop
            init(true);
            return false;
        }
        // convert origin to alt-above-terrain if necessary
        if (!custom_pos_terrain_alt) {
            // new destination is alt-above-terrain, previous destination was alt-above-ekf-origin
            pos_control->set_pos_offset_z_cm(origin_terr_offset);
        }
    } else {
        pos_control->set_pos_offset_z_cm(0.0);
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // set position target and zero velocity and acceleration
    custom_pos_target_cm = destination.topostype();
    custom_pos_terrain_alt = terrain_alt;
    custom_vel_target_cms.zero();
    custom_accel_target_cmss.zero();
    update_time_ms = millis();

    // log target
    // copter.Log_Write_Custom_Position_Target(custom_mode, custom_pos_target_cm.tofloat(), custom_pos_terrain_alt, custom_vel_target_cms, custom_accel_target_cmss);

    send_notification = true;

    return true;
}

bool ModeCustom::get_wp(Location& destination) const
{
    switch (custom_mode) {
    case SubMode::WP:
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::Pos:
        destination = Location(custom_pos_target_cm.tofloat(), custom_pos_terrain_alt ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
        return true;
    default:
        return false;
    }

    // should never get here but just in case
    return false;
}

// sets custom mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and custom waypoint is outside the fence
bool ModeCustom::set_destination(const Location& dest_loc, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
#if AP_FENCE_ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // if using wpnav for position control
    if (use_wpnav_for_position_control()) {
        if (custom_mode != SubMode::WP) {
            wp_control_start();
        }

        if (!wp_nav->set_wp_destination_loc(dest_loc)) {
            // failure to set destination can only be because of missing terrain data
            AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
            // failure is propagated to GCS with NAK
            return false;
        }

        // set yaw state
        set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

        // log target
        // copter.Log_Write_Custom_Position_Target(custom_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt), (dest_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN), Vector3f(), Vector3f());
        send_notification = true;
        return true;
    }

    // if configured to use position controller for position control
    // ensure we are in position control mode
    if (custom_mode != SubMode::Pos) {
        pos_control_start();
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // set position target and zero velocity and acceleration
    Vector3f pos_target_f;
    bool terrain_alt;
    if (!wp_nav->get_vector_NEU(dest_loc, pos_target_f, terrain_alt)) {
        return false;
    }

    // initialise terrain following if needed
    if (terrain_alt) {
        // get current alt above terrain
        float origin_terr_offset;
        if (!wp_nav->get_terrain_offset(origin_terr_offset)) {
            // if we don't have terrain altitude then stop
            init(true);
            return false;
        }
        // convert origin to alt-above-terrain if necessary
        if (!custom_pos_terrain_alt) {
            // new destination is alt-above-terrain, previous destination was alt-above-ekf-origin
            pos_control->set_pos_offset_z_cm(origin_terr_offset);
        }
    } else {
        pos_control->set_pos_offset_z_cm(0.0);
    }

    custom_pos_target_cm = pos_target_f.topostype();
    custom_pos_terrain_alt = terrain_alt;
    custom_vel_target_cms.zero();
    custom_accel_target_cmss.zero();
    update_time_ms = millis();

    // log target
    // copter.Log_Write_Custom_Position_Target(custom_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt), custom_pos_terrain_alt, custom_vel_target_cms, custom_accel_target_cmss);

    send_notification = true;

    return true;
}

// set_velaccel - sets custom mode's target velocity and acceleration
void ModeCustom::set_accel(const Vector3f& acceleration, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{
    // check we are in velocity control mode
    if (custom_mode != SubMode::Accel) {
        accel_control_start();
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // set velocity and acceleration targets and zero position
    custom_pos_target_cm.zero();
    custom_pos_terrain_alt = false;
    custom_vel_target_cms.zero();
    custom_accel_target_cmss = acceleration;
    update_time_ms = millis();

    // log target
    if (log_request) {
        // copter.Log_Write_Custom_Position_Target(custom_mode, custom_pos_target_cm.tofloat(), custom_pos_terrain_alt, custom_vel_target_cms, custom_accel_target_cmss);
    }
}

// set_velocity - sets custom mode's target velocity
void ModeCustom::set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{
    set_velaccel(velocity, Vector3f(), use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw, log_request);
}

// set_velaccel - sets custom mode's target velocity and acceleration
void ModeCustom::set_velaccel(const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{
    // check we are in velocity control mode
    if (custom_mode != SubMode::VelAccel) {
        velaccel_control_start();
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // set velocity and acceleration targets and zero position
    custom_pos_target_cm.zero();
    custom_pos_terrain_alt = false;
    custom_vel_target_cms = velocity;
    custom_accel_target_cmss = acceleration;
    update_time_ms = millis();

    // log target
    if (log_request) {
        // copter.Log_Write_Custom_Position_Target(custom_mode, custom_pos_target_cm.tofloat(), custom_pos_terrain_alt, custom_vel_target_cms, custom_accel_target_cmss);
    }
}

// set_destination_posvel - set custom mode position and velocity target
bool ModeCustom::set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    return set_destination_posvelaccel(destination, velocity, Vector3f(), use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);
}

// set_destination_posvelaccel - set custom mode position, velocity and acceleration target
bool ModeCustom::set_destination_posvelaccel(const Vector3f& destination, const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // check we are in velocity control mode
    if (custom_mode != SubMode::PosVelAccel) {
        posvelaccel_control_start();
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    update_time_ms = millis();
    custom_pos_target_cm = destination.topostype();
    custom_pos_terrain_alt = false;
    custom_vel_target_cms = velocity;
    custom_accel_target_cmss = acceleration;

    // log target
    // copter.Log_Write_Guided_Position_Target(custom_mode, custom_pos_target_cm.tofloat(), custom_pos_terrain_alt, custom_vel_target_cms, custom_accel_target_cmss);
    return true;
}

// returns true if GUIDED_OPTIONS param suggests SET_ATTITUDE_TARGET's "thrust" field should be interpreted as thrust instead of climb rate
bool ModeCustom::set_attitude_target_provides_thrust() const
{
    // return ((copter.g2.custom_options.get() & uint32_t(Options::SetAttitudeTarget_ThrustAsThrust)) != 0);
    return copter.g2.custom_options.get(); //AKGL TODO: maybe remove function later 
}

// returns true if GUIDED_OPTIONS param specifies position should be controlled (when velocity and/or acceleration control is active)
bool ModeCustom::stabilizing_pos_xy() const
{
    // return !((copter.g2.custom_options.get() & uint32_t(Options::DoNotStabilizePositionXY)) != 0);
     return copter.g2.custom_options.get(); //AKGL TODO: remove function later
}

// returns true if GUIDED_OPTIONS param specifies velocity should  be controlled (when acceleration control is active)
bool ModeCustom::stabilizing_vel_xy() const
{
    // return !((copter.g2.custom_options.get() & uint32_t(Options::DoNotStabilizeVelocityXY)) != 0);
    return copter.g2.custom_options.get(); //AKGL TODO: remove funciton later

}

// returns true if GUIDED_OPTIONS param specifies waypoint navigation should be used for position control (allow path planning to be used but updates must be slower)
bool ModeCustom::use_wpnav_for_position_control() const
{
    // return ((copter.g2.custom_options.get() & uint32_t(Options::WPNavUsedForPosControl)) != 0);
    return copter.g2.custom_options.get(); //AKGL TODO: remove function later
}

// Sets custom's angular target submode: Using a rotation quaternion, angular velocity, and climbrate or thrust (depends on user option)
// attitude_quat: IF zero: ang_vel (angular velocity) must be provided even if all zeroes
//                IF non-zero: attitude_control is performed using both the attitude quaternion and angular velocity
// ang_vel: angular velocity (rad/s)
// climb_rate_cms_or_thrust: represents either the climb_rate (cm/s) or thrust scaled from [0, 1], unitless
// use_thrust: IF true: climb_rate_cms_or_thrust represents thrust
//             IF false: climb_rate_cms_or_thrust represents climb_rate (cm/s)
void ModeCustom::set_angle(const Quaternion &attitude_quat, const Vector3f &ang_vel, float climb_rate_cms_or_thrust, bool use_thrust)
{
    // check we are in velocity control mode
    if (custom_mode != SubMode::Angle) {
        angle_control_start();
    }

    custom_angle_state.attitude_quat = attitude_quat;
    custom_angle_state.ang_vel = ang_vel;

    custom_angle_state.use_thrust = use_thrust;
    if (use_thrust) {
        custom_angle_state.thrust = climb_rate_cms_or_thrust;
        custom_angle_state.climb_rate_cms = 0.0f;
    } else {
        custom_angle_state.thrust = 0.0f;
        custom_angle_state.climb_rate_cms = climb_rate_cms_or_thrust;
    }

    custom_angle_state.update_time_ms = millis();

    // convert quaternion to euler angles
    float roll_rad, pitch_rad, yaw_rad;
    attitude_quat.to_euler(roll_rad, pitch_rad, yaw_rad);

    // log target
    // copter.Log_Write_Custom_Attitude_Target(custom_mode, roll_rad, pitch_rad, yaw_rad, ang_vel, custom_angle_state.thrust, custom_angle_state.climb_rate_cms * 0.01);
}

// takeoff_run - takeoff in custom mode
//      called by custom_run at 100hz or more
void ModeCustom::takeoff_run()
{
    auto_takeoff_run();
    if (auto_takeoff_complete && !takeoff_complete) {
        takeoff_complete = true;
#if AP_LANDINGGEAR_ENABLED
        // optionally retract landing gear
        copter.landinggear.retract_after_takeoff();
#endif
    }
}

// pos_control_run - runs the custom position controller
// called from custom_run
void ModeCustom::pos_control_run() //AKGL use position and attitude control in here
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // calculate terrain adjustments
    float terr_offset = 0.0f;
    if (custom_pos_terrain_alt && !wp_nav->get_terrain_offset(terr_offset)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // send position and velocity targets to position controller
    custom_accel_target_cmss.zero();
    custom_vel_target_cms.zero();

    // stop rotating if no updates received within timeout_ms
    if (millis() - update_time_ms > get_timeout_ms()) {
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    float pos_offset_z_buffer = 0.0; // Vertical buffer size in m
    if (custom_pos_terrain_alt) {
        pos_offset_z_buffer = MIN(copter.wp_nav->get_terrain_margin() * 100.0, 0.5 * fabsF(custom_pos_target_cm.z));
    }
    pos_control->input_pos_xyz(custom_pos_target_cm, terr_offset, pos_offset_z_buffer);

    // run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    // attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());  // AKGL removed to avoid error at compile time
}

// velaccel_control_run - runs the custom velocity controller
// called from custom_run
void ModeCustom::accel_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        custom_vel_target_cms.zero();
        custom_accel_target_cmss.zero();
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
        pos_control->input_vel_accel_xy(custom_vel_target_cms.xy(), custom_accel_target_cmss.xy(), false);
        pos_control->input_vel_accel_z(custom_vel_target_cms.z, custom_accel_target_cmss.z, false);
    } else {
        // update position controller with new target
        pos_control->input_accel_xy(custom_accel_target_cmss);
        if (!stabilizing_vel_xy()) {
            // set position and velocity errors to zero
            pos_control->stop_vel_xy_stabilisation();
        } else if (!stabilizing_pos_xy()) {
            // set position errors to zero
            pos_control->stop_pos_xy_stabilisation();
        }
        pos_control->input_accel_z(custom_accel_target_cmss.z);
    }

    // call velocity controller which includes z axis controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    // attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading()); // // AKGL removed to avoid error at compile time
}

// velaccel_control_run - runs the custom velocity and acceleration controller
// called from custom_run
void ModeCustom::velaccel_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        custom_vel_target_cms.zero();
        custom_accel_target_cmss.zero();
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    bool do_avoid = false;
#if AC_AVOID_ENABLED
    // limit the velocity for obstacle/fence avoidance
    copter.avoid.adjust_velocity(custom_vel_target_cms, pos_control->get_pos_xy_p().kP(), pos_control->get_max_accel_xy_cmss(), pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), G_Dt);
    do_avoid = copter.avoid.limits_active();
#endif

    // update position controller with new target

    if (!stabilizing_vel_xy() && !do_avoid) {
        // set the current commanded xy vel to the desired vel
        custom_vel_target_cms.x = pos_control->get_vel_desired_cms().x;
        custom_vel_target_cms.y = pos_control->get_vel_desired_cms().y;
    }
    pos_control->input_vel_accel_xy(custom_vel_target_cms.xy(), custom_accel_target_cmss.xy(), false);
    if (!stabilizing_vel_xy() && !do_avoid) {
        // set position and velocity errors to zero
        pos_control->stop_vel_xy_stabilisation();
    } else if (!stabilizing_pos_xy() && !do_avoid) {
        // set position errors to zero
        pos_control->stop_pos_xy_stabilisation();
    }
    pos_control->input_vel_accel_z(custom_vel_target_cms.z, custom_accel_target_cmss.z, false);

    // call velocity controller which includes z axis controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    // attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());  // AKGL removed to avoid error at compile time
}

// pause_control_run - runs the custom mode pause controller
// called from custom_run
void ModeCustom::pause_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set the horizontal velocity and acceleration targets to zero
    Vector2f vel_xy, accel_xy;
    pos_control->input_vel_accel_xy(vel_xy, accel_xy, false);

    // set the vertical velocity and acceleration targets to zero
    float vel_z = 0.0;
    pos_control->input_vel_accel_z(vel_z, 0.0, false);

    // call velocity controller which includes z axis controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0);
}

// posvelaccel_control_run - runs the custom position, velocity and acceleration controller
// called from custom_run
void ModeCustom::posvelaccel_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        custom_vel_target_cms.zero();
        custom_accel_target_cmss.zero();
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    // send position and velocity targets to position controller
    if (!stabilizing_vel_xy()) {
        // set the current commanded xy pos to the target pos and xy vel to the desired vel
        custom_pos_target_cm.x = pos_control->get_pos_target_cm().x;
        custom_pos_target_cm.y = pos_control->get_pos_target_cm().y;
        custom_vel_target_cms.x = pos_control->get_vel_desired_cms().x;
        custom_vel_target_cms.y = pos_control->get_vel_desired_cms().y;
    } else if (!stabilizing_pos_xy()) {
        // set the current commanded xy pos to the target pos
        custom_pos_target_cm.x = pos_control->get_pos_target_cm().x;
        custom_pos_target_cm.y = pos_control->get_pos_target_cm().y;
    }
    pos_control->input_pos_vel_accel_xy(custom_pos_target_cm.xy(), custom_vel_target_cms.xy(), custom_accel_target_cmss.xy(), false);
    if (!stabilizing_vel_xy()) {
        // set position and velocity errors to zero
        pos_control->stop_vel_xy_stabilisation();
    } else if (!stabilizing_pos_xy()) {
        // set position errors to zero
        pos_control->stop_pos_xy_stabilisation();
    }

    // custom_pos_target z-axis should never be a terrain altitude
    if (custom_pos_terrain_alt) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    float pz = custom_pos_target_cm.z;
    pos_control->input_pos_vel_accel_z(pz, custom_vel_target_cms.z, custom_accel_target_cmss.z, false);
    custom_pos_target_cm.z = pz;

    // run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    // attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());  // AKGL removed to avoid error at compile time
}

// angle_control_run - runs the custom angle controller
// called from custom_run
void ModeCustom::angle_control_run()
{
    float climb_rate_cms = 0.0f;
    if (!custom_angle_state.use_thrust) {
        // constrain climb rate
        climb_rate_cms = constrain_float(custom_angle_state.climb_rate_cms, -wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up());

        // get avoidance adjusted climb rate
        climb_rate_cms = get_avoidance_adjusted_climbrate(climb_rate_cms);
    }

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - custom_angle_state.update_time_ms > get_timeout_ms()) {
        custom_angle_state.attitude_quat.initialise();
        custom_angle_state.ang_vel.zero();
        climb_rate_cms = 0.0f;
        if (custom_angle_state.use_thrust) {
            // initialise vertical velocity controller
            pos_control->init_z_controller();
            custom_angle_state.use_thrust = false;
        }
    }

    // interpret positive climb rate or thrust as triggering take-off
    const bool positive_thrust_or_climbrate = is_positive(custom_angle_state.use_thrust ? custom_angle_state.thrust : climb_rate_cms);
    if (motors->armed() && positive_thrust_or_climbrate) {
        copter.set_auto_armed(true);
    }

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || !copter.ap.auto_armed || (copter.ap.land_complete && !positive_thrust_or_climbrate)) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // TODO: use get_alt_hold_state
    // landed with positive desired climb rate, takeoff
    if (copter.ap.land_complete && (custom_angle_state.climb_rate_cms > 0.0f)) {
        zero_throttle_and_relax_ac();
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
            set_land_complete(false);
            pos_control->init_z_controller();
        }
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call attitude controller
    if (custom_angle_state.attitude_quat.is_zero()) {
        attitude_control->input_rate_bf_roll_pitch_yaw(ToDeg(custom_angle_state.ang_vel.x) * 100.0f, ToDeg(custom_angle_state.ang_vel.y) * 100.0f, ToDeg(custom_angle_state.ang_vel.z) * 100.0f);
    } else {
        attitude_control->input_quaternion(custom_angle_state.attitude_quat, custom_angle_state.ang_vel);
    }

    // call position controller
    if (custom_angle_state.use_thrust) {
        attitude_control->set_throttle_out(custom_angle_state.thrust, true, copter.g.throttle_filt);
    } else {
        pos_control->set_pos_target_z_from_climb_rate_cm(climb_rate_cms);
        pos_control->update_z_controller();
    }
}

// helper function to set yaw state and targets
void ModeCustom::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw && relative_angle) {
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    } else if (use_yaw && use_yaw_rate) {
        auto_yaw.set_yaw_angle_rate(yaw_cd * 0.01f, yaw_rate_cds * 0.01f);
    } else if (use_yaw && !use_yaw_rate) {
        auto_yaw.set_yaw_angle_rate(yaw_cd * 0.01f, 0.0f);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate(yaw_rate_cds);
    } else {
        auto_yaw.set_mode_to_default(false);
    }
}

// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeCustom::use_pilot_yaw(void) const
{
    // return (copter.g2.custom_options.get() & uint32_t(Options::IgnorePilotYaw)) == 0;
    return copter.g2.custom_options.get(); //AKGL TODO: remove function later
}

// Custom Limit code

// limit_clear - clear/turn off custom limits
void ModeCustom::limit_clear()
{
    custom_limit.timeout_ms = 0;
    custom_limit.alt_min_cm = 0.0f;
    custom_limit.alt_max_cm = 0.0f;
    custom_limit.horiz_max_cm = 0.0f;
}

// limit_set - set custom timeout and movement limits
void ModeCustom::limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    custom_limit.timeout_ms = timeout_ms;
    custom_limit.alt_min_cm = alt_min_cm;
    custom_limit.alt_max_cm = alt_max_cm;
    custom_limit.horiz_max_cm = horiz_max_cm;
}

// limit_init_time_and_pos - initialise custom start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_custom_start function
void ModeCustom::limit_init_time_and_pos()
{
    // initialise start time
    custom_limit.start_time = AP_HAL::millis();

    // initialise start position from current position
    custom_limit.start_pos = inertial_nav.get_position_neu_cm();
}

// limit_check - returns true if custom mode has breached a limit
//  used when custom is invoked from the NAV_GUIDED_ENABLE mission command
bool ModeCustom::limit_check()
{
    // check if we have passed the timeout
    if ((custom_limit.timeout_ms > 0) && (millis() - custom_limit.start_time >= custom_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position_neu_cm();

    // check if we have gone below min alt
    if (!is_zero(custom_limit.alt_min_cm) && (curr_pos.z < custom_limit.alt_min_cm)) {
        return true;
    }

    // check if we have gone above max alt
    if (!is_zero(custom_limit.alt_max_cm) && (curr_pos.z > custom_limit.alt_max_cm)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (custom_limit.horiz_max_cm > 0.0f) {
        const float horiz_move = get_horizontal_distance_cm(custom_limit.start_pos.xy(), curr_pos.xy());
        if (horiz_move > custom_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}

const Vector3p &ModeCustom::get_target_pos() const
{
    return custom_pos_target_cm;
}

const Vector3f& ModeCustom::get_target_vel() const
{
    return custom_vel_target_cms;
}

const Vector3f& ModeCustom::get_target_accel() const
{
    return custom_accel_target_cmss;
}

uint32_t ModeCustom::wp_distance() const
{
    switch(custom_mode) {
    case SubMode::WP:
        return wp_nav->get_wp_distance_to_destination();
    case SubMode::Pos:
        return get_horizontal_distance_cm(inertial_nav.get_position_xy_cm(), custom_pos_target_cm.tofloat().xy());
    case SubMode::PosVelAccel:
        return pos_control->get_pos_error_xy_cm();
        break;
    default:
        return 0;
    }
}

int32_t ModeCustom::wp_bearing() const
{
    switch(custom_mode) {
    case SubMode::WP:
        return wp_nav->get_wp_bearing_to_destination();
    case SubMode::Pos:
        return get_bearing_cd(inertial_nav.get_position_xy_cm(), custom_pos_target_cm.tofloat().xy());
    case SubMode::PosVelAccel:
        return pos_control->get_bearing_to_target_cd();
        break;
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::Angle:
        // these do not have bearings
        return 0;
    }
    // compiler guarantees we don't get here
    return 0.0;
}

float ModeCustom::crosstrack_error() const
{
    switch (custom_mode) {
    case SubMode::WP:
        return wp_nav->crosstrack_error();
    case SubMode::Pos:
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::PosVelAccel:
        return pos_control->crosstrack_error();
    case SubMode::Angle:
        // no track to have a crosstrack to
        return 0;
    }
    // compiler guarantees we don't get here
    return 0;
}

// return custom mode timeout in milliseconds. Only used for velocity, acceleration, angle control, and angular rates
uint32_t ModeCustom::get_timeout_ms() const
{
    return MAX(copter.g2.custom_timeout, 0.1) * 1000;
}

// pause guide mode
bool ModeCustom::pause()
{
    _paused = true;
    return true;
}

// resume custom mode
bool ModeCustom::resume()
{
    _paused = false;
    return true;
}

#endif
