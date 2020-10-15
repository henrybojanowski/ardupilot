#include "mode.h"
#include "Plane.h"

uint32_t starting_alt = 0; // Starting altitude global
uint32_t tstart = 0; // Starting time global in milliseconds
int32_t roll_angle_prev_cd = 0; // Roll angle from the previous iteration global in centi-degrees
// We can detect angle wrapping events by comparing the current value to the previous iteration value

bool ModeCircle::_enter()
{

    starting_alt = plane.current_loc.alt; // Store the starting altitude.
    // We could later add logic to take corrective action if we fall below it.

    tstart = millis(); // Store the starting time in milliseconds.

    roll_angle_prev_cd = plane.ahrs.roll_sensor; // Store the initial roll angle as calculated by the Attitude and Heading Reference System (AHRS)
    // Learn more about the Extended Kalman Filters (EKFs) used for state estimation: https://ardupilot.org/dev/docs/ekf.html

    // The lines below are copied from "mode_acro.cpp". I thought it might be a good starting point since the Acrobatic mode allows full rotations in pitch and roll
    // However since we are using setpoints for roll and pitch (i.e. plane.nav_roll_cd), you also have to increase the limits for the make allowed angles.
    // To do this, search on "lim_" in the configuration list.
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;    // In Acro mode this was false. I changed it to true. Is the cruise speed parameter used as the setpoint?
    plane.auto_navigation_mode = false; // We will be setting the roll and pitch command setpoints, so I don't think we can use navigation?
    // Could we use navigation to keep the bottom of the fuselage pointed at the ground base to avoid tangling up the tether line?
    // Or just calculate that angle ourselves and make it the yaw setpoint?

    // I don't know what the two lines below do, so commenting out for now. Need to study the ACRO controller. Where is the code for it?
//    plane.acro_state.locked_roll = true;
//    plane.acro_state.locked_pitch = true;


    plane.nav_roll_cd  = 0; // Set the initial roll angle command to zero
    plane.nav_pitch_cd = 800; // Set the initial pitch angle command to 8 degrees (8 * 100 = 600)
    //plane.calc_throttle();  // Update the throttle to fly at the setpoint speed


    return true;
}

void ModeCircle::update()
{
    // Let's create sine and cosine functions to represent the circle angles.
    // (i.e. pitch_angle_deg = sin(w*t)*90 where w = 2*pi*f, f is the frequency in Hz, and we want an amplitude of 90 degrees).
    uint32_t tnow = millis();
    float tnowf = (tnow-tstart) * 1e-3f;    // Divide by 1000 to convert to seconds. However, always use multiplication when dividing by a constant since it is much less CPU intensive.

    float circle_freq_Hz = 1/20.0f; // Set the frequency of our circle maneuver.
    float circle_angle_rad = M_2PI * circle_freq_Hz * tnowf;    // How long before tnowf is so big that this overflows single precision max continuous integer value (2^24)? Should we reset tstart periodically? ,,,,,,,,,,,Should we add wrap_2PI() or wrap_PI() around this?
//    float pitch_angle_setpoint_cd = sinf(circle_angle_rad) * 9000.0f;  // The span for pitch angles is -90 to 90 degrees. This is in centi-degrees (cd), so we multiply by 100.
    float roll_angle_setpoint_cd = cosf(circle_angle_rad) * 18000.0f;    // The span for roll angles is -180 to 180 degrees. This is in centi-degrees (cd), so we multiply by 100.

    // For this first test, rather than trying to fly a vertical circle, let's take one step at a time
    // and see if we can control the roll angle.
    // So, let's set the pitch command to 8 degrees to slowly climb
    // and flip the roll angle from -170 to 10 degrees. We are intentionally trying to avoid crossing
    // a roll angle of 180 degrees so we don't have to deal with angle wrap (yet).
    // However, the PID gains for roll will have to be tuned well enough that the roll angle doesn't
    // cross -180.

    plane.nav_pitch_cd = 800; //static_cast<int>(pitch_angle_cd);

    // The sign of roll_angle_setpoint_cd should change every half cycle, so we can use the sign change to roll over
    if (roll_angle_setpoint_cd < 0.0f) {
        plane.nav_roll_cd  = -17000;    // Set roll angle command to -170 degrees
    }
    else
    {
        plane.nav_roll_cd  = 1000;    // Set roll angle command to +10 degrees
    }

    int32_t roll_angle_current_cd = plane.ahrs.roll_sensor; // Get the current roll angle from the AHRS

    // Here is a first pass attempt at detecting the angle wrap rollover of the roll angle.
    // This is based on detecting a big jump in the value. There is probably a better way to detect it.
    // I'm also testing out sending debug messages using the GCS send_text command.
    float roll_angle_change_cd = fabsf(roll_angle_current_cd - roll_angle_prev_cd);
    if (roll_angle_change_cd >= 18000) {
        // I noticed that the GCS send_text string has a limited length (49 characters?) Extra characters are sent in a second message.
        gcs().send_text(MAV_SEVERITY_NOTICE, "Roll wrap detected. roll_angle_change = %.1f deg",
                        (double)((roll_angle_change_cd)*0.01));
    }


    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 50);   // Set throttle to 50 percent

    roll_angle_prev_cd = roll_angle_current_cd; // Save the roll angle on this iteration for comparison in the next iteration, so we can detect angle wrapping

}

