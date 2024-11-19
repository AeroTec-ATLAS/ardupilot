#include "mode.h"
#include "Plane.h"

#if MODE_SYSTEMID_ENABLED == ENABLED

/*
 * Init and run calls for systemId, flight mode
 */

const AP_Param::GroupInfo ModeSystemId::var_info[] = {

    // @Param: _AXIS
    // @DisplayName: System identification axis
    // @Description: Controls which surfaces are being excited.  Set to non-zero to see more parameters
    // @User: Standard
    // @Values: 0:None, 1:Elevator, 2:Aileron, 3:Rudder
    AP_GROUPINFO_FLAGS("_AXIS", 1, ModeSystemId, axis, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _MAGNITUDE
    // @DisplayName: System identification Chirp Magnitude
    // @Description: Magnitude of sweep in outputs.
    // @Range: 0.01 100
    // @User: Standard
    AP_GROUPINFO("_MAGNITUDE", 2, ModeSystemId, waveform_magnitude, 0.5f),

    // @Param: _F_START_HZ
    // @DisplayName: System identification Start Frequency
    // @Description: Frequency at the start of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_START_HZ", 3, ModeSystemId, frequency_start, 0.5f),

    // @Param: _F_STOP_HZ
    // @DisplayName: System identification Stop Frequency
    // @Description: Frequency at the end of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_STOP_HZ", 4, ModeSystemId, frequency_stop, 10),

    // @Param: _T_FADE_IN
    // @DisplayName: System identification Fade in time
    // @Description: Time to reach maximum amplitude of sweep
    // @Range: 0 20
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_IN", 5, ModeSystemId, time_fade_in, 5),

    // @Param: _T_REC
    // @DisplayName: System identification Total Sweep length
    // @Description: Time taken to complete the sweep
    // @Range: 0 255
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_REC", 6, ModeSystemId, time_record, 20),

    // @Param: _T_FADE_OUT
    // @DisplayName: System identification Fade out time
    // @Description: Time to reach zero amplitude at the end of the sweep
    // @Range: 0 5
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_OUT", 7, ModeSystemId, time_fade_out, 2),

    // @Param: _MIN_ALT
    // @DisplayName: System identification Minimum Altitude
    // @Description: Minimum altitude needed for system id mode to function. In case of failure, automatically switches mode to loiter.
    // @Range: -10 500
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("_MIN_ALT", 8, ModeSystemId, min_alt, 50),


    // @Param: _ARM_ALT
    // @DisplayName: System identification Arm Altitude
    // @Description: Minimum altitude needed to arm the system id mode
    // @Range: -10 500
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("_ARM_ALT", 9, ModeSystemId, arm_alt, 80),

    AP_GROUPEND
};

ModeSystemId::ModeSystemId(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#define SYSTEM_ID_DELAY     1.0f      // time in seconds waited after system id mode change for frequency sweep injection

// systemId_init - initialise systemId controller
bool ModeSystemId::_enter()
{
    // check if enabled
    if (axis == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "No axis selected, SID_AXIS = 0");
        return false;
    }

    //check if altitude is higher than allowed to arm
    if ( arm_alt > plane.relative_ground_altitude(plane.g.rangefinder_landing)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Arm altitude lower than allowed");
        return false;
    }

    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    ///if (quadplane.motors->armed() && copter.ap.land_complete && !copter.flightmode->has_manual_throttle()) {
       /// return false;
    /// }

    // att_bf_feedforward = attitude_control->get_bf_feedforward();
    waveform_time = 0.0f;
    time_const_freq = 2.0f / frequency_start; // Two full cycles at the starting frequency
    systemid_state = SystemIDModeState::SYSTEMID_STATE_TESTING;
    log_subsample = 0;

    chirp_input.init(time_record, frequency_start, frequency_stop, time_fade_in, time_fade_out, time_const_freq);

    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Starting: axis=%d", (unsigned)axis);

    plane.Log_Write_SysID_Setup(axis, waveform_magnitude, frequency_start, frequency_stop, time_fade_in, time_const_freq, time_record, time_fade_out);

    return true;
}

// systemId_exit - clean up systemId controller before exiting
//void ModeSystemId::exit()
 // {}

// systemId_run - runs the systemId controller
// should be called at 100hz or more
void ModeSystemId::update()
{
    // apply simple mode transform to pilot inputs
    // update_simple_mode();

    // convert pilot input to lean angles
    // float target_roll, target_pitch;
    // quadplane.get_pilot_desired_lean_angles(target_roll, target_pitch, plane.quadplane.aparm.angle_max, plane.quadplane.aparm.angle_max);

    // get pilot's desired yaw rate
 // float target_yaw_rate = 0; /// get_pilot_desired_yaw_rate(plane.channel_yaw->norm_input_dz());

    // if (!motors->armed()) {
    //     // Motors should be Stopped
    //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    // // Tradheli doesn't set spool state to ground idle when throttle stick is zero.  Ground idle only set when
    // // motor interlock is disabled.
    // } else if (copter.ap.throttle_zero && !copter.is_tradheli()) {
    //     // Attempting to Land
    //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    // } else {
    //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    // }

    // switch (motors->get_spool_state()) {
    // case AP_Motors::SpoolState::SHUT_DOWN:
    //     // Motors Stopped
    //     attitude_control->reset_yaw_target_and_rate();
    //     attitude_control->reset_rate_controller_I_terms();
    //     break;

    // case AP_Motors::SpoolState::GROUND_IDLE:
    //     // Landed
    //     // Tradheli initializes targets when going from disarmed to armed state. 
    //     // init_targets_on_arming is always set true for multicopter.
    //     if (motors->init_targets_on_arming()) {
    //         attitude_control->reset_yaw_target_and_rate();
    //         attitude_control->reset_rate_controller_I_terms_smoothly();
    //     }
    //     break;

    // case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    //     // clear landing flag above zero throttle
    //     if (!motors->limit.throttle_lower) {
    //         set_land_complete(false);
    //     }
    //     break;

    // case AP_Motors::SpoolState::SPOOLING_UP:
    // case AP_Motors::SpoolState::SPOOLING_DOWN:
    //     // do nothing
    //     break;
    // }

    // get pilot's desired throttle

    // float pilot_throttle_scaled = get_pilot_desired_throttle();


    // If an error is detected, set system-id to stop
    if ((systemid_state == SystemIDModeState::SYSTEMID_STATE_TESTING) && (!is_positive(frequency_start) || !is_positive(frequency_stop) || is_negative(time_fade_in) || !is_positive(time_record) || is_negative(time_fade_out) || (time_record <= time_fade_in) || (min_alt > plane.relative_ground_altitude(plane.g.rangefinder_landing)))) {
        systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
        gcs().send_text(MAV_SEVERITY_INFO, "SystemID Parameter Error");
    }

    // Update the control surface movement
    waveform_time += plane.G_Dt;
    waveform_sample = chirp_input.update(waveform_time - SYSTEM_ID_DELAY, waveform_magnitude);
    waveform_freq_rads = chirp_input.get_frequency_rads();

    switch (systemid_state) {

        // If the state is set to be stopped, change to mode to the one chosen
        case SystemIDModeState::SYSTEMID_STATE_STOPPED:
            plane.set_mode(plane.mode_loiter, ModeReason::MISSION_END);
            break;
        
        //If the system-id is running
        case SystemIDModeState::SYSTEMID_STATE_TESTING:

            /* if (!plane.is_flying()) {  
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: Landed");
                break;
            } */
            /* if (attitude_control->lean_angle_deg()*100 > attitude_control->lean_angle_max_cd()) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: lean=%f max=%f", (double)attitude_control->lean_angle_deg(), (double)attitude_control->lean_angle_max_cd());
                break;
            } */
            if (waveform_time > SYSTEM_ID_DELAY + time_fade_in + time_const_freq + time_record + time_fade_out) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Finished");
                break;
            }

            // Depending on the axis, set the chosen surface control to the waveform and the rest to 0 
            switch ((AxisType)axis.get()) {
                case AxisType::NONE:
                    systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: axis = 0");
                    break;

                 case AxisType::INPUT_ELEVATOR:
                    SRV_Channels::set_output_norm(SRV_Channel::k_aileron, 0);
                    SRV_Channels::set_output_norm(SRV_Channel::k_rudder, 0);
                    SRV_Channels::set_output_norm(SRV_Channel::k_elevator, waveform_sample);
                    // target_roll += waveform_sample*100.0f;;
                    break;

                case AxisType::INPUT_AILERON:
                    SRV_Channels::set_output_norm(SRV_Channel::k_elevator, 0);
                    SRV_Channels::set_output_norm(SRV_Channel::k_rudder, 0);
                    SRV_Channels::set_output_norm(SRV_Channel::k_aileron, waveform_sample);
                    // arget_pitch += waveform_sample*100.0f;
                    break;

                 case AxisType::INPUT_RUDDER:
                    SRV_Channels::set_output_norm(SRV_Channel::k_aileron, 0);
                    SRV_Channels::set_output_norm(SRV_Channel::k_elevator, 0);
                    SRV_Channels::set_output_norm(SRV_Channel::k_rudder, waveform_sample);
                    // target_yaw_rate += waveform_sample*100.0f;
                    break;
            }
            break;

        }

    // call attitude controller
    // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle




    if (log_subsample <= 0) {
        log_data();
        if (plane.should_log(MASK_LOG_ATTITUDE_FAST) && plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 1;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_FAST)) {
            log_subsample = 2;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 4;
        } else {
            log_subsample = 8;
        }
    }
    log_subsample -= 1;
}

// log system id and attitude
void ModeSystemId::log_data() const
{
    Vector3f delta_angle;
    float delta_angle_dt;
    plane.ins.get_delta_angle(delta_angle, delta_angle_dt);

    Vector3f delta_velocity;
    float delta_velocity_dt;
    plane.ins.get_delta_velocity(delta_velocity, delta_velocity_dt);

    if (is_positive(delta_angle_dt) && is_positive(delta_velocity_dt)) {
        plane.Log_Write_SysID_Data(waveform_time, waveform_sample, waveform_freq_rads / (2 * M_PI), degrees(delta_angle.x / delta_angle_dt), degrees(delta_angle.y / delta_angle_dt), degrees(delta_angle.z / delta_angle_dt), delta_velocity.x / delta_velocity_dt, delta_velocity.y / delta_velocity_dt, delta_velocity.z / delta_velocity_dt);
    }

    // Full rate logging of attitude, rate loops
    plane.Log_Write_Attitude();
}

#endif