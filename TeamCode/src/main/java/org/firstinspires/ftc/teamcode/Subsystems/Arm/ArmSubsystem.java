package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ArmSubsystem extends SubsystemBase {
    DcMotorEx armMotor;

    public ArmSubsystem(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoder();
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setPositionPIDFCoefficients(2.5);
        armMotor.setTargetPositionTolerance(60);
    }

    public void setPower(double power){
        armMotor.setPower(power);
    }

    public void setArmTicks(int ticks){
        armMotor.setTargetPosition(ticks);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
}
