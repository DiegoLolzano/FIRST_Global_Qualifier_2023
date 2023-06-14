package org.firstinspires.ftc.teamcode.Subsystems.Wrist;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoedWrist extends SubsystemBase {
    private Servo wristServo;

    public ServoedWrist(HardwareMap hardwareMap){
        wristServo = hardwareMap.get(Servo.class, "wristServo");

        wristServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setWristServoPos(double position){
        wristServo.setPosition(position);
    }

    public double getServoPosition(){
        return wristServo.getPosition();
    }
}
