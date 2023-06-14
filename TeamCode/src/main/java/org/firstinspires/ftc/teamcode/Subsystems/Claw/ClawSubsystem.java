package org.firstinspires.ftc.teamcode.Subsystems.Claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {
    Servo leftClaw;
    Servo rightClaw;

    public ClawSubsystem(HardwareMap hardwareMap){
        leftClaw = hardwareMap.get(Servo.class, "clawLeft"); //2
        rightClaw = hardwareMap.get(Servo.class, "clawRight"); //1

        leftClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw.setDirection(Servo.Direction.FORWARD);
    }

    public void setLeftClawPos(double position){
        leftClaw.setPosition(position);
    }

    public void setRightClawPos(double position){
        rightClaw.setPosition(position);
    }

    public void setBothClawPos(double position){
        leftClaw.setPosition(position);
        rightClaw.setPosition(position);
    }
}
