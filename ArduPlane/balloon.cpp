#include "Plane.h"

const AP_Param::GroupInfo BalloonRelease::var_info[] = {

    // @Param: ANGLE
    // @DisplayName: Tailsitter fixed wing transition angle
    // @Description: This is the pitch angle at which tailsitter aircraft will change from VTOL control to fixed wing control.
    // @Units: deg
    // @Range: 1 16
    AP_GROUPINFO("OVRD_CH", 1, BalloonRelease, override_channel, 7),

    // @Param: ANG_VT
    // @DisplayName: Tailsitter VTOL transition angle
    // @Description: This is the pitch angle at which tailsitter aircraft will change from fixed wing control to VTOL control, if zero Q_TAILSIT_ANGLE will be used
    // @Units: ms
    // @Range: 0 10000
    AP_GROUPINFO("WAIT_MS", 2, BalloonRelease, wait_time, 5000),

    // @Param: INPUT
    // @DisplayName: Tailsitter input type bitmask
    // @Description: This controls whether stick input when hovering as a tailsitter follows the conventions for fixed wing hovering or multicopter hovering. When PlaneMode is not enabled (bit0 = 0) the roll stick will roll the aircraft in earth frame and yaw stick will yaw in earth frame. When PlaneMode input is enabled, the roll and yaw sticks are swapped so that the roll stick controls earth-frame yaw and rudder controls earth-frame roll. When body-frame roll is enabled (bit1 = 1), the yaw stick controls earth-frame yaw rate and the roll stick controls roll in the tailsitter's body frame when flying level.
    // @Bitmask: 0:PlaneMode,1:BodyFrameRoll
    AP_GROUPINFO("WAIT_ALT", 3, BalloonRelease, wait_altitude, 15),

    AP_GROUPEND
};

BalloonRelease::BalloonRelease(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    balloon_released = false;
    balloon_release_time_ms = 0;
    balloon_release_altitude_cm = 0;
}


/* 
   trigger release servo for balloon release
*/
void BalloonRelease::balloon_release()
{
    // If aircraft is armed, set released flag to stop servo closing
    if (!balloon_released && plane.arming.is_armed()) {
        balloon_released = true;
        balloon_release_time_ms = millis();
        balloon_release_altitude_cm = plane.current_loc.alt;
    }
    // Activate release servo
    SRV_Channels::set_output_norm(SRV_Channel::k_scripting16, 1.0f);
    // Wait 3 seconds after release to complete command
    //uint32_t time_release_ms = millis();
    // while (millis() - time_release_ms < 3000) {
    //     // Wait for 3 seconds after release
    // }
}

void BalloonRelease::balloon_release_override()
{
    // If aircraft is armed, set released flag to stop servo closing
    if (!balloon_released && plane.arming.is_armed()) {
        balloon_released = true;
        balloon_release_time_ms = millis();
        balloon_release_altitude_cm = plane.current_loc.alt;
        gcs().send_text(MAV_SEVERITY_INFO,"Manual balloon release activated");
    }
    // Activate release servo
    SRV_Channels::set_output_norm(SRV_Channel::k_scripting16, 1.0f);
}

bool BalloonRelease::pilot_release_override(){
    if (balloon_released){
        balloon_release_override();
        return true;
    }
    // Get position input from Scripting8 RC channel
    int switchPWM = rc().channel((uint8_t) override_channel-1)->get_radio_in();
    // Check if switch is in the release position (above 0.5)
    if (switchPWM > 1500) {
        // Activate release servo
        balloon_release_override();
        return true;
    }
    SRV_Channels::set_output_norm(SRV_Channel::k_scripting16, -1.0f);
    return false;
}

bool BalloonRelease::balloon_safety_check()
{
    // If RC failsafe is active, return true
    if (plane.failsafe.rc_failsafe) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"EMERGENCY BALLOON RELEASE: RC failsafe activated");
        balloon_release();
        // Trigger short failsafe action
        plane.failsafe_short_on_event(FAILSAFE_SHORT, ModeReason::RADIO_FAILSAFE);
        return true;
    }
  return false;
}

