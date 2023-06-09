package org.firstinspires.ftc.teamcode.Subsystems.Basket;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BasketSubsystem extends SubsystemBase {
    Servo basketServo;
    public BasketSubsystem(HardwareMap hardwareMap){
        basketServo = hardwareMap.get(Servo.class, "basketServo");

        basketServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setBasketPos(double position){
        basketServo.setPosition(position);
    }
}
