package org.firstinspires.ftc.teamcode.Subsystems.Testing;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ControlHUBTester extends SubsystemBase {
    DcMotorEx motorTest;
    DcMotorEx motorTest2;
    DcMotorEx motorTest3;
    DcMotorEx motorTest4;

    Servo servoTest;
    Servo servoTest1;
    Servo servoTest2;
    Servo servoTest3;
    Servo servoTest4;
    Servo servoTest5;

    BNO055IMU imu;

    public ControlHUBTester(HardwareMap hardwareMap){
        motorTest = hardwareMap.get(DcMotorEx.class, "motorTest");
        motorTest2 = hardwareMap.get(DcMotorEx.class, "motorTest2");
        motorTest3 = hardwareMap.get(DcMotorEx.class, "motorTest3");
        motorTest4 = hardwareMap.get(DcMotorEx.class, "motorTest4");

        servoTest = hardwareMap.get(Servo.class, "servoTest");
        servoTest1 = hardwareMap.get(Servo.class, "servoTest1");
        servoTest2 = hardwareMap.get(Servo.class, "servoTest2");
        servoTest3 = hardwareMap.get(Servo.class, "servoTest3");
        servoTest4 = hardwareMap.get(Servo.class, "servoTest4");
        servoTest5 = hardwareMap.get(Servo.class, "servoTest5");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        motorTest.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTest2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTest3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTest4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorTest.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTest2.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTest3.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTest4.setDirection(DcMotorSimple.Direction.FORWARD);

        motorTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTest2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTest3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTest4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTest2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTest3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTest4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoTest.setDirection(Servo.Direction.FORWARD);
        servoTest1.setDirection(Servo.Direction.FORWARD);
        servoTest2.setDirection(Servo.Direction.FORWARD);
        servoTest3.setDirection(Servo.Direction.FORWARD);
        servoTest4.setDirection(Servo.Direction.FORWARD);
        servoTest5.setDirection(Servo.Direction.FORWARD);

        motorTest.setPositionPIDFCoefficients(0.5);
        motorTest.setTargetPositionTolerance(20);

        motorTest2.setPositionPIDFCoefficients(0.5);
        motorTest2.setTargetPositionTolerance(20);

        motorTest3.setPositionPIDFCoefficients(0.5);
        motorTest3.setTargetPositionTolerance(20);

        motorTest4.setPositionPIDFCoefficients(0.5);
        motorTest4.setTargetPositionTolerance(20);


    }

    //Motor methods
    public void setMotorPower(double power){
        motorTest.setPower(power);
        motorTest2.setPower(power);
        motorTest3.setPower(power);
        motorTest4.setPower(power);
    }

    public void motorToPose(int ticks){
        motorTest.setTargetPosition(ticks);
        motorTest2.setTargetPosition(ticks);
        motorTest3.setTargetPosition(ticks);
        motorTest4.setTargetPosition(ticks);

        motorTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTest2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTest3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTest4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(0.5);
    }

    public void returnToUsingEncoder(){
        motorTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTest2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTest3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTest4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getMotor1CurrentPose(){
        return motorTest.getCurrentPosition();
    }

    public double getMotor2CurrentPose(){
        return motorTest2.getCurrentPosition();
    }

    public double getMotor3CurrentPose(){
        return motorTest3.getCurrentPosition();
    }

    public double getMotor4CurrentPose(){
        return motorTest4.getCurrentPosition();
    }

    public double getMotor1CurrentVelo(){
        return motorTest.getVelocity();
    }

    public double getMotor2CurrentVelo(){
        return motorTest2.getVelocity();
    }

    public double getMotor3CurrentVelo(){
        return motorTest3.getVelocity();
    }

    public double getMotor4CurrentVelo(){
        return motorTest4.getVelocity();
    }

    //Servo Methods
    public void setServoPose(double pose){
        servoTest.setPosition(pose);
        servoTest1.setPosition(pose);
        servoTest2.setPosition(pose);
        servoTest3.setPosition(pose);
        servoTest4.setPosition(pose);
        servoTest5.setPosition(pose);
    }

    public double getServoPose1(){
        return servoTest.getPosition();
    }

    public double getServoPose2(){
        return servoTest1.getPosition();
    }

    public double getServoPose3(){
        return servoTest2.getPosition();
    }

    public double getServoPose4(){
        return servoTest3.getPosition();
    }

    public double getServoPose5(){
        return servoTest4.getPosition();
    }

    public double getServoPose6(){
        return servoTest5.getPosition();
    }


    public double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}
