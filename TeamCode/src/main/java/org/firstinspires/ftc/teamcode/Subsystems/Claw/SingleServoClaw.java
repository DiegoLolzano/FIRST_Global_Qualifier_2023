package org.firstinspires.ftc.teamcode.Subsystems.Claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SingleServoClaw extends SubsystemBase {
    Servo clawServo;

    public SingleServoClaw(HardwareMap hardwareMap){
        clawServo = hardwareMap.get(Servo.class, "singleClaw"); //2

        clawServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setSingleServoPos(double position){
        clawServo.setPosition(position);
    }
}
