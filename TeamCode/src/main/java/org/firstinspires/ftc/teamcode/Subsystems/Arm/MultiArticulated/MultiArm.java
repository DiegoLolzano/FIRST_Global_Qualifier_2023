package org.firstinspires.ftc.teamcode.Subsystems.Arm.MultiArticulated;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MultiArm extends SubsystemBase {
    DcMotorEx multiArmMotor;

    public MultiArm(HardwareMap hardwareMap){
        multiArmMotor = hardwareMap.get(DcMotorEx.class, "multiArmMotor");

        multiArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        multiArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        resetEncoder();
        multiArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        multiArmMotor.setPositionPIDFCoefficients(2);
        multiArmMotor.setTargetPositionTolerance(30);
    }

    public void setPower(double power){
        multiArmMotor.setPower(power);
    }

    public void setArmTicks(int ticks){
        multiArmMotor.setTargetPosition(ticks);
        multiArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetArmTicks(){
        multiArmMotor.setPower(0);
        multiArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getArmTicks(){
        return multiArmMotor.getCurrentPosition();
    }

    public void changeToUsingEncoder(){
        multiArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isArmBusy(){
        return multiArmMotor.isBusy();
    }

    public double getArmAmpCurrent(){
        return multiArmMotor.getCurrent(CurrentUnit.AMPS);}

    public void resetEncoder(){
        multiArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
