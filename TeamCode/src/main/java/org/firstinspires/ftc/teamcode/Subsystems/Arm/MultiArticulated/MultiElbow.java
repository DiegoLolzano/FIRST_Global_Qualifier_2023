package org.firstinspires.ftc.teamcode.Subsystems.Arm.MultiArticulated;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MultiElbow extends SubsystemBase {
    DcMotorEx elbowMotor;

    public MultiElbow(HardwareMap hardwareMap){
        elbowMotor = hardwareMap.get(DcMotorEx.class, "elbowMotor");

        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        resetEncoder();
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbowMotor.setPositionPIDFCoefficients(2);
        elbowMotor.setTargetPositionTolerance(30);
    }

    public void setPower(double power){
        elbowMotor.setPower(power);
    }

    public void setArmTicks(int ticks){
        elbowMotor.setTargetPosition(ticks);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetArmTicks(){
        elbowMotor.setPower(0);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getArmTicks(){
        return elbowMotor.getCurrentPosition();
    }

    public void changeToUsingEncoder(){
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isArmBusy(){
        return elbowMotor.isBusy();
    }

    public double getArmAmpCurrent(){
        return elbowMotor.getCurrent(CurrentUnit.AMPS);}

    public void resetEncoder(){
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
