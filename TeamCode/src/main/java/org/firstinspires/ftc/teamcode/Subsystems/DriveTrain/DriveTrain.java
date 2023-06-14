package org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CerbLib.DriveSignal;
import org.firstinspires.ftc.teamcode.CerbLib.TankDrive;
import org.firstinspires.ftc.teamcode.CerbLib.Twist2d;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;

public class DriveTrain extends TankDrive {
    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;
    //private BNO055IMU imu; Checar que onda con esto un dia libre

    public DriveTrain(HardwareMap hardwareMap){
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");

        //imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        resetSensors();

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setPositionPIDFCoefficients(2);
        rightDrive.setPositionPIDFCoefficients(2);

        leftDrive.setTargetPositionTolerance(20);
        rightDrive.setTargetPositionTolerance(20);
    }

    @Override
    public void setLeftPower(double power){
        leftDrive.setPower(power);
    }

    @Override
    public void setRightPower(double power){
        rightDrive.setPower(power);
    }

    @Override
    public double getLeftVelocity(){
        return leftDrive.getVelocity(AngleUnit.DEGREES);
    }

    @Override
    public double getRightVelocity(){
        return rightDrive.getVelocity(AngleUnit.DEGREES);
    }

    @Override
    public void setLeftMotorTargetPos(int ticks){
        leftDrive.setTargetPosition(ticks);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void setRightMotorTargetPos(int ticks){
        rightDrive.setTargetPosition(ticks);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public double getLeftCurrentPos(){
        return leftDrive.getCurrentPosition();
    }

    @Override
    public double getRightCurrentPos(){
        return rightDrive.getCurrentPosition();
    }

    @Override
    public void resetSensors(){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public boolean isLeftBusy(){
        return leftDrive.isBusy();
    }

    @Override
    public boolean isRightBusy(){
        return rightDrive.isBusy();
    }

    public void invertMotors(DcMotorSimple.Direction leftdirection,
                             DcMotorSimple.Direction rightDirection){
        leftDrive.setDirection(leftdirection);
        rightDrive.setDirection(rightDirection);
    }

    /********** Cheesy Drive **********/
    public void setOpenLoop(DriveSignal signal) {
        leftDrive.setPower(signal.getLeft());
        rightDrive.setPower(signal.getRight());
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    private static final double kEpsilon = 1E-9;//-------

    public static DriveSignal inverseKinematics(Twist2d velocity) {
        if (Math.abs(velocity.dtheta) < kEpsilon) {
            return new DriveSignal(velocity.dx, velocity.dx);
        }
        double delta_v = DriveConstants.TRACK_WIDTH * velocity.dtheta / (2 * DriveConstants.TRACK_SCRUB_FACTOR);
        return new DriveSignal(velocity.dx - delta_v, velocity.dx + delta_v);
    }

    public void setCheesyishDrive(Gamepad gamepad) {
        setCheesyishDrive(
                -gamepad.left_stick_y,
                -gamepad.right_stick_x,
                true);
    }

    public void setCheesyishDrive(double throttle, double wheel, boolean quickTurn){

        if (epsilonEquals(throttle, 0.0, 0.075)) {
            throttle = 0.0;
        }

        if (epsilonEquals(wheel, 0.0, 0.075)) {
            wheel = 0.0;
        }

        final double kWheelGain = 0.05;
        final double kWheelNonlinearity = 0.05;
        final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
        // Apply a sin function that's scaled to make it feel better.
        if (!quickTurn) {
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = wheel / (denominator * denominator) * Math.abs(throttle);
        }

        wheel *= kWheelGain;
        DriveSignal signal = inverseKinematics(new Twist2d(throttle, 0.0, wheel));
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
        setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
    }
}

