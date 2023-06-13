package org.firstinspires.ftc.teamcode.Subsystems.Claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem2 extends SubsystemBase {
    Servo leftClaw2;
    Servo rightClaw2;

    public ClawSubsystem2(HardwareMap hardwareMap){
        leftClaw2 = hardwareMap.get(Servo.class, "clawLeft2"); // 3
        rightClaw2 = hardwareMap.get(Servo.class, "clawRight2");// 4

        leftClaw2.setDirection(Servo.Direction.REVERSE);
        rightClaw2.setDirection(Servo.Direction.FORWARD);
    }

    public void setLeftClaw2Pos(double position){
        leftClaw2.setPosition(position);
    }

    public void setRightClaw2Pos(double position){
        rightClaw2.setPosition(position);
    }

    public void setBotServos2Pos(double position){
        leftClaw2.setPosition(position);
        rightClaw2.setPosition(position);
    }
}
