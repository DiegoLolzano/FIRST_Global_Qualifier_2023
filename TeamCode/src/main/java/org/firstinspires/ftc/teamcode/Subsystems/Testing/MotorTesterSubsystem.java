package org.firstinspires.ftc.teamcode.Subsystems.Testing;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CerbLib.Units;

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
        motor0.getVelocity(AngleUnit.RADIANS);
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

    public double getMotor0VeloRPM(){
        return Units.radiansPerSecondToRotationsPerMinute(motor0.getVelocity(AngleUnit.RADIANS));
    }

    public double getMotor1VeloRPM(){
        return Units.radiansPerSecondToRotationsPerMinute(motor1.getVelocity(AngleUnit.RADIANS));
    }

    public double getMotor2VeloRPM(){
        return Units.radiansPerSecondToRotationsPerMinute(motor2.getVelocity(AngleUnit.RADIANS));
    }

    public double getMotor3VeloRPM(){
        return Units.radiansPerSecondToRotationsPerMinute(motor3.getVelocity(AngleUnit.RADIANS));
    }
}
