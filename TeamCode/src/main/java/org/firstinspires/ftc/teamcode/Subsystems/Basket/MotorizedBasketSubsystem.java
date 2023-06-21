package org.firstinspires.ftc.teamcode.Subsystems.Basket;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MotorizedBasketSubsystem extends SubsystemBase {
    DcMotorEx basketMotor;
    private double basketP = 1.0;

    public MotorizedBasketSubsystem(HardwareMap hardwareMap){
        basketMotor = hardwareMap.get(DcMotorEx.class, "basketMotor");

        basketMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        basketMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        basketMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        basketMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        basketMotor.setPositionPIDFCoefficients(basketP);
        basketMotor.setTargetPositionTolerance(20);
    }

    public void setPower(double power){
        basketMotor.setPower(power);
    }

    public void setBasketTicks(int ticks){
        basketMotor.setTargetPosition(ticks);
        basketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveBasketToPose(int ticks){
        setBasketTicks(ticks);
        setPower(0.4);
    }

    public double getBasketTicks(){
        return basketMotor.getCurrentPosition();
    }

    public boolean isBasketBusy(){
        return basketMotor.isBusy();
    }

    public boolean isBasketAtPoint(){
        return basketMotor.getCurrentPosition() == basketMotor.getTargetPosition();
    }
}
