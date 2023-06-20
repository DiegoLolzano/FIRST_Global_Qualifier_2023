package org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Claw.DoubleServoClaw;

import java.util.concurrent.TimeUnit;

public class DefaultDoubleClaw extends CommandBase {
    DoubleServoClaw clawSubsystem;
    private ClawModes modes;
    private Gamepad gamepad;
    protected Timer timer;

    public enum ClawModes{
        OPEN,
        CLOSED

    }

    public DefaultDoubleClaw(DoubleServoClaw clawSubsystem, ClawModes modes){
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
                clawSubsystem.setLeftClawPos(0.5);
                clawSubsystem.setRightClawPos(0.5);
            break;

            case CLOSED:
                clawSubsystem.setLeftClawPos(0.9);
                clawSubsystem.setRightClawPos(0.9);
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
