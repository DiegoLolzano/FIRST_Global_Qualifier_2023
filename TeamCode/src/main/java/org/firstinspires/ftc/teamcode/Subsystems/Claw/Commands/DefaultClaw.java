package org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem;

import com.arcrobotics.ftclib.util.Timing.Timer;

import java.sql.Time;
import java.util.concurrent.TimeUnit;

public class DefaultClaw extends CommandBase {
    ClawSubsystem clawSubsystem;
    private ClawModes modes;
    private Gamepad gamepad;
    protected Timer timer;

    public enum ClawModes{
        OPEN,
        CLOSED
    }

    public DefaultClaw(ClawSubsystem clawSubsystem, ClawModes modes){
        this.clawSubsystem = clawSubsystem;
        this.modes = modes;

        timer = new Timer(700, TimeUnit.MILLISECONDS);

        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize(){
        timer.start();
    }

    @Override
    public void execute(){
        switch (modes){
            case OPEN:
                clawSubsystem.setLeftClawPos(0.5);
                clawSubsystem.setRightClawPos(0.5);
            break;

            case CLOSED:
                clawSubsystem.setLeftClawPos(0);
                clawSubsystem.setRightClawPos(0);
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
