package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */

    /*
    * 20:1 TICKS PER REV = 537.6
    * 20:1 MAX_RPM = 312.5
    * FORMULA FOR ULTRA-PLANETARY GEARBOX TICKS AND RPM
    * TICKS_PER_REV = (DESIRED GEAR RATIO / 20:1) * TPR 20:1
    *
    * MAX_RPM = (MAX_RPM 20:1 * GEAR RATIO 20:1) / DESIRED GEAR RATIO
    *
    */
    //Core Hex = 288
    //HD Hex 20:1 = 560
    //HD Hex free or with planetary= 28
    public static final double TICKS_PER_REV = 403.2;
    //Core Hex = 125
    //HD Hex 20:1 = 300
    //HD Hex free or with planetary = 6000
    public static final double MAX_RPM = 416.66;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double P_DRIVE_GAIN = 0.03;
    public static double P_TURN_GAIN = 0.0055;
    public static double HEADING_THRESHOLD = 1.0;

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 3.54331; // in
    public static double GEAR_RATIO = 1;//16; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 13.9375; // in
    public static double TRACK_SCRUB_FACTOR = 1.0;
    //-----------calculos-----------//
    public static double TICKS_PER_INCH = ( TICKS_PER_REV * GEAR_RATIO ) / (2 * Math.PI * WHEEL_RADIUS );
    public static double INCH_PER_TICK = (2 * Math.PI * WHEEL_RADIUS ) / ( TICKS_PER_REV * GEAR_RATIO ) ;
    //--------CENTNRAL ANGLES--------//
    public static double CENTRAL_ANGLE_TO_INCHES = (DriveConstants.TRACK_WIDTH * Math.PI) / 360;
    public static double INCHES_TO_CENTRAL_ANGLE = 360 /(DriveConstants.TRACK_WIDTH * Math.PI);
    //--------SIDED ANGLES--------//
    public static double SIDED_ANGLE_TO_INCHES = (2 * Math.PI * (DriveConstants.TRACK_WIDTH + 0)) / 360;
    public static double INCHES_TO_SIDED_ANGLE =   360 / (2 * Math.PI * (DriveConstants.TRACK_WIDTH + 1));

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    public static double MAX_VEL = 20;//30;
    public static double MAX_ACCEL = 20;//30;
    public static double MAX_ANG_VEL = Math.toRadians(60);
    public static double MAX_ANG_ACCEL = Math.toRadians(60);

    /*
     * Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
     */
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
