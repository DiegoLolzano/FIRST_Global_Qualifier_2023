package org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Wrist.ServoedWrist;

import java.util.concurrent.TimeUnit;

public class ManualServoControl extends CommandBase {
    private ServoedWrist m_wrist;
    private Gamepad gamepad;

    private Timing.Timer timer;

    public ManualServoControl(ServoedWrist m_wrist){
        this.m_wrist = m_wrist;

        timer = new Timing.Timer(700, TimeUnit.MILLISECONDS);

        addRequirements(m_wrist);
    }

    @Override
    public void initialize(){
        timer.start();
    }

    @Override
    public void execute(){
        double joystickValue = gamepad.left_stick_x;
        double servoPosition = (joystickValue + 1.0) / 2.0;
        m_wrist.setWristServoPos(servoPosition);
    }

    @Override
    public void end(boolean interrupted){
        timer.pause();
    }

    @Override
    public boolean isFinished(){
        return timer.done() & gamepad == null;
    }
}
