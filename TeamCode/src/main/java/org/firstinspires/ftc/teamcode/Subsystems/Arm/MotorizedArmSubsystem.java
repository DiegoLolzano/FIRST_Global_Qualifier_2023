package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class MotorizedArmSubsystem extends SubsystemBase {
    DcMotorEx armMotor;

    public MotorizedArmSubsystem(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoder();
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setPositionPIDFCoefficients(6);
        armMotor.setTargetPositionTolerance(30);
    }

    public void setPower(double power){
        armMotor.setPower(power);
    }

    public void setArmTicks(int ticks){
        armMotor.setTargetPosition(ticks);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveArmToPos(int ticks){
        armMotor.setTargetPosition(ticks);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
    }

    public void resetArmTicks(){
        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getArmTicks(){
        return armMotor.getCurrentPosition();
    }

    public void changeToUsingEncoder(){
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isArmBusy(){
        return armMotor.isBusy();
    }

    public double getArmAmpCurrent(){
        return armMotor.getCurrent(CurrentUnit.AMPS);}

    public void resetEncoder(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isArmAtPoint(){
        return armMotor.getCurrentPosition() == armMotor.getTargetPosition();
    }
}
