package org.firstinspires.ftc.teamcode.CerbLib;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class TankDrive extends SubsystemBase {

    public void setLeftPower(double power){}
    public void setRightPower(double power){}

    public void setLeftMotorTargetPos(int ticks){}
    public void setRightMotorTargetPos(int ticks){}

    public double getLeftCurrentPos(){return 0;}
    public double getRightCurrentPos(){return 0;}

    public int getLeftPos(){return 0;}
    public int getRightPos(){return 0;}

    public double getLeftVelocity(){return 0;}
    public double getRightVelocity(){return 0;}

    public void setLeftVelocity(double velocity){}
    public void setRightVelocity(double velocity){}

    public boolean isLeftBusy(){return false;}
    public boolean isRightBusy(){return false;}

    public void resetSensors(){}
    public void stopMotors(){}

    public void setCheezyDrive(Gamepad gamepad){}
    public void setCheezyDrive(double Throttle, double Wheel, boolean quickTurn){}

    public void setLeftMode(){}
    public void setRightMode(){}

    public double getAngle(){return 0.0;}

    public void moveRobot(double drive, double turn){}
}
