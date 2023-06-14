package org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Wrist.ServoedWrist;

import java.util.concurrent.TimeUnit;

public class ServoWristControl extends CommandBase {
    private ServoedWrist m_wrist;
    private Gamepad gamepad;
    private WristState state;
    private double servoPos = 0.0;

    private Timer timer;

    public enum WristState{
        PICK,
        LOWER_JUNCTION,
        MIDDLE_JUNCTION,
        UPPER_JUNCTION,
        TRANSPORT
    }

    public ServoWristControl(ServoedWrist m_wrist, WristState state){
        this.m_wrist = m_wrist;
        this.state = state;

        timer = new Timer(1500, TimeUnit.MILLISECONDS);

        addRequirements(m_wrist);
    }

    @Override
    public void initialize(){
        timer.start();
    }

    @Override
    public void execute(){
        switch (state){
            case PICK:
                servoPos = 0.60;
                m_wrist.setWristServoPos(servoPos);
            break;

            case TRANSPORT:
                servoPos = 0.20;
                m_wrist.setWristServoPos(servoPos);
            break;

            case LOWER_JUNCTION:
                servoPos = 0.7;
                m_wrist.setWristServoPos(servoPos);
            break;

            case MIDDLE_JUNCTION:
                servoPos = 0.55;
                m_wrist.setWristServoPos(servoPos);
            break;

            case UPPER_JUNCTION:
                servoPos = 0.35;
                m_wrist.setWristServoPos(servoPos);
            break;
        }
    }

    @Override
    public void end(boolean interrupted){
        timer.pause();
    }

    @Override
    public boolean isFinished(){
        return timer.done() & gamepad == null;
    }

    public boolean isWristOnPos(){
        return m_wrist.getServoPosition() == servoPos;
    }
}
