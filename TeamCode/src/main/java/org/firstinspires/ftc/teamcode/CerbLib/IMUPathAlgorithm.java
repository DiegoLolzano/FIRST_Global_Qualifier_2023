package org.firstinspires.ftc.teamcode.CerbLib;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;

public class IMUPathAlgorithm {
    TankDrive tankDrive;

    private int leftTarget = 0;
    private int rightTarget = 0;
    private double turnSpeed = 0.0;

    private double robotHeading = 0.0;
    private double targetHeading = 0.0;
    private double headingOffset = 0.0;
    private double headingError = 0.0;

    public int lastLeftPose = 0;
    public int lastRightPose = 0;

    public IMUPathAlgorithm(TankDrive tankDrive){
        this.tankDrive = tankDrive;
    }

    public void driveStraight(double maxDriveSpeed, double distance, double heading){
        int moveCounts = (int)(distance * DriveConstants.TICKS_PER_INCH);
        //leftTarget = (int)tankDrive.getLeftCurrentPos() + moveCounts;
        //rightTarget = (int)tankDrive.getRightCurrentPos() + moveCounts;

        tankDrive.setLeftMotorTargetPos(lastLeftPose + moveCounts);
        tankDrive.setRightMotorTargetPos(lastRightPose + moveCounts);

        maxDriveSpeed = Math.abs(maxDriveSpeed);

        //tankDrive.moveRobot(maxDriveSpeed, 0);

        while(tankDrive.isLeftBusy() && tankDrive.isRightBusy()){
            turnSpeed = getSteeringCorrection(heading, DriveConstants.P_DRIVE_GAIN);

            if(distance < 0)
                turnSpeed *= -1.0;

            tankDrive.moveRobot(maxDriveSpeed, turnSpeed);
        }

        tankDrive.moveRobot(0, 0);
        tankDrive.setLeftMode();
        tankDrive.setRightMode();
    }


    public void turnToHeading(double maxTurnSpeed, double heading){
        getSteeringCorrection(heading, DriveConstants.P_DRIVE_GAIN);

        while (Math.abs(headingError) > DriveConstants.HEADING_THRESHOLD){
            turnSpeed = getSteeringCorrection(heading, DriveConstants.P_TURN_GAIN);

            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            tankDrive.moveRobot(0, turnSpeed);
        }

        tankDrive.moveRobot(0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = tankDrive.getAngle() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void UpdateLastPose(){
        lastLeftPose = (int)tankDrive.getLeftCurrentPos();
        lastRightPose = (int)tankDrive.getRightCurrentPos();
    }

    public TankDrive getTankDrive(){
        return tankDrive;
    }
}
