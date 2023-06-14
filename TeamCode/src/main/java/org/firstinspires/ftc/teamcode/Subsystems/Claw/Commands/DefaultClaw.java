package org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem;

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

        timer = new Timer(1500, TimeUnit.SECONDS);

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
                clawSubsystem.setLeftClawPos(0.35);
                clawSubsystem.setRightClawPos(0.35);
            break;

            case CLOSED:
                clawSubsystem.setLeftClawPos(0.1);
                clawSubsystem.setRightClawPos(0.1);
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
