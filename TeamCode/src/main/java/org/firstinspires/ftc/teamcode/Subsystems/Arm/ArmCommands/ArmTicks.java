package org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmSubsystem;

import java.util.Timer;

public class ArmTicks extends CommandBase {
    private ArmSubsystem armSubsystem;
    private ArmModes modes;
    private int desiredPos = 0;
    protected Timer timer;

    public enum ArmModes{
        FLOOR,
        TRANSIT,
        SCORE
    }

    public ArmTicks(ArmSubsystem armSubsystem, ArmModes modes){
        this.armSubsystem = armSubsystem;
        this.modes = modes;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.changeToUsingEncoder();
    }

    @Override
    public void execute(){
        switch (modes){
            case FLOOR:
                desiredPos = 1895;
                armSubsystem.setArmTicks(desiredPos);
                armSubsystem.setPower(0.5);
            break;

            case TRANSIT:
                desiredPos = 181;
                armSubsystem.setArmTicks(desiredPos);
                armSubsystem.setPower(0.5);
            break;

            case SCORE:
                desiredPos = 740;
                armSubsystem.setArmTicks(desiredPos);
                armSubsystem.setPower(0.5);
        }
    }

    @Override
    public void end(boolean interrupted){
        armSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished(){
        return !armSubsystem.isArmBusy();
    }

    public String getCurrentArmMode(){
        return modes.toString();
    }
}
