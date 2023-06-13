package org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem2;

import java.util.concurrent.TimeUnit;

public class DefaultClaw2 extends CommandBase {
    ClawSubsystem2 clawSubsystem2;
    private ClawModes2 modes2;
    private Gamepad gamepad;
    protected Timer timer;

    public enum ClawModes2{
        OPEN2,
        CLOSED2

    }

    public DefaultClaw2(ClawSubsystem2 clawSubsystem2, ClawModes2 modes2){
        this.clawSubsystem2 = clawSubsystem2;
        this.modes2 = modes2;

        timer = new Timer(700, TimeUnit.MILLISECONDS);

        addRequirements(clawSubsystem2);
    }

    @Override
    public void initialize(){
        timer.start();
    }

    @Override
    public void execute(){
        switch (modes2){
            case OPEN2:
                clawSubsystem2.setLeftClaw2Pos(0);
                clawSubsystem2.setRightClaw2Pos(0);
            break;

            case CLOSED2:
                clawSubsystem2.setLeftClaw2Pos(1.0);
                clawSubsystem2.setRightClaw2Pos(1.0);
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
