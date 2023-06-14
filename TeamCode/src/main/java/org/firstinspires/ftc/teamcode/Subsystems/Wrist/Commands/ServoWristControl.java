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
                m_wrist.setWristServoPos(0.60);
            break;

            case TRANSPORT:
                m_wrist.setWristServoPos(0.20);
            break;

            case LOWER_JUNCTION:
                m_wrist.setWristServoPos(0.7);
            break;

            case MIDDLE_JUNCTION:
                m_wrist.setWristServoPos(0.55);
            break;

            case UPPER_JUNCTION:
                m_wrist.setWristServoPos(0.35);
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
}
