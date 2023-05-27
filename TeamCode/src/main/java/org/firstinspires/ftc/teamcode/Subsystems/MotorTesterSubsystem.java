package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorTesterSubsystem extends SubsystemBase {
    DcMotorEx motor0;
    DcMotorEx motor1;
    DcMotorEx motor2;
    DcMotorEx motor3;
    public MotorTesterSubsystem(HardwareMap hardwareMap){
        motor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");

        motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);

        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotor0Power(double power){
        motor0.setPower(power);
    }

    public void setMotor1Power(double power){
        motor1.setPower(power);
    }

    public void setMotor2Power(double power){
        motor2.setPower(power);
    }

    public void setMotor3Power(double power){
        motor3.setPower(power);
    }

    public double getMotor0VeloTicks(){
        return motor0.getVelocity();
    }

    public double getMotor1VeloTicks(){
        return motor1.getVelocity();
    }

    public double getMotor2VeloTicks(){
        return motor2.getVelocity();
    }

    public double getMotor3VeloTicks(){
        return motor3.getVelocity();
    }
}
