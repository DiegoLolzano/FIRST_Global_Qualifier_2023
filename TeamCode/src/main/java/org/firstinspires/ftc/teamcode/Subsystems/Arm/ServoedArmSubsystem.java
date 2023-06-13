package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoedArmSubsystem extends SubsystemBase {
    Servo armServo;

    public ServoedArmSubsystem(HardwareMap hardwareMap){
        armServo = hardwareMap.get(Servo.class, "servoArm"); //5

        armServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setServoArmPos(double position){
        armServo.setPosition(position);
    }
}
