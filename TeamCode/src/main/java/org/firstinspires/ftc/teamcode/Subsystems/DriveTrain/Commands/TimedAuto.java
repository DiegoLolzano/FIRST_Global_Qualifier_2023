package org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

import java.util.concurrent.TimeUnit;

public class TimedAuto extends CommandBase {
    private DriveTrain m_drive;
    Timer timer;

    public TimedAuto(DriveTrain m_drive){
        this.m_drive = m_drive;
        timer = new Timer(7, TimeUnit.SECONDS);

        addRequirements(m_drive);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        m_drive.setRightPower(-0.6);
        m_drive.setLeftPower(-0.6);
    }

    @Override
    public void end(boolean interrupted){
        m_drive.setLeftPower(0);
        m_drive.setRightPower(0);
    }

    @Override
    public boolean isFinished(){
        return timer.done();
    }
}
