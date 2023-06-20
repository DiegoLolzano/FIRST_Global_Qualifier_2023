package org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Claw.SingleServoClaw;

import java.util.concurrent.TimeUnit;

public class DefaultSingleClaw extends CommandBase {
    SingleServoClaw clawSubsystem;
    private SingleClawModes singleModes;
    private Gamepad gamepad;
    protected Timing.Timer timer;

    public enum SingleClawModes{
        OPEN_SINGLE,
        CLOSED_SINGLE

    }

    public DefaultSingleClaw(SingleServoClaw clawSubsystem, SingleClawModes singleModes){
        this.clawSubsystem = clawSubsystem;
        this.singleModes = singleModes;

        timer = new Timing.Timer(1500, TimeUnit.SECONDS);

        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize(){
        timer.start();
    }

    @Override
    public void execute(){
        switch (singleModes){
            case OPEN_SINGLE:
                clawSubsystem.setSingleServoPos(0.9);
                break;

            case CLOSED_SINGLE:
                clawSubsystem.setSingleServoPos(0.5);
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
