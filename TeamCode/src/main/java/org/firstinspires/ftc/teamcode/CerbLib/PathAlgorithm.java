package org.firstinspires.ftc.teamcode.CerbLib;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;

public class PathAlgorithm {
    private double ticks = 0;
    private double COMPENSATED_POWER = 0;

    private int lastLeftPose = 0;
    private int lastRightPose = 0;
    private double error = 0;

    private TankDrive tankDrive;


    public enum DriveSides{
        LEFT,RIGHT
    }

    public PathAlgorithm(TankDrive tankDrive){
        this.tankDrive = tankDrive;
    }

    public void straight(double distance_IN){
        double ticks = DriveConstants.TICKS_PER_INCH * distance_IN;
        COMPENSATED_POWER = setCompensatedPower(distance_IN);

        tankDrive.setLeftMotorTargetPos(lastLeftPose + (int)ticks);
        tankDrive.setRightMotorTargetPos(lastRightPose + (int)ticks );

        tankDrive.setRightPower(0.8);
        tankDrive.setLeftPower(0.8);
    }

    public void centralTurn(double angle){
        double arcLength = DriveConstants.CENTRAL_ANGLE_TO_INCHES * angle;
        double ticks = arcLength * DriveConstants.TICKS_PER_INCH;

        COMPENSATED_POWER = setCompensatedPower(arcLength);

        tankDrive.setLeftMotorTargetPos(lastLeftPose - (int)ticks);
        tankDrive.setRightMotorTargetPos(lastRightPose + (int)ticks );

        tankDrive.setRightPower(0.8);
        tankDrive.setLeftPower(0.8);
    }

    public void sideTurn(double angle, DriveSides side2Turn){//TODO: checar si si se ocupa el 2 * PI
        double arcLength = DriveConstants.SIDED_ANGLE_TO_INCHES * angle;
        double ticks = arcLength * DriveConstants.TICKS_PER_INCH;

        COMPENSATED_POWER = setCompensatedPower(arcLength);

        switch (side2Turn){
            case LEFT:
                tankDrive.setRightMotorTargetPos(lastRightPose + (int)ticks );

                tankDrive.setRightPower(0.8);
                tankDrive.setLeftPower(0);
                break;

            case RIGHT:
                tankDrive.setLeftMotorTargetPos(lastLeftPose + (int)ticks);

                tankDrive.setRightPower(0);
                tankDrive.setLeftPower(0.8);
                break;
        }
    }
    public void UpdateLastPose(){
        lastLeftPose = (int)tankDrive.getLeftCurrentPos();
        lastRightPose = (int)tankDrive.getRightCurrentPos();
    }

    private double setCompensatedPower(double distance){
        return distance / DriveConstants.MAX_VEL;
    }
    public double getCompensatedPower(){
        return COMPENSATED_POWER;
    }


    public TankDrive getTankDrive(){
        return tankDrive;
    }
    //Batman en tanga
}
