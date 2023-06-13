package org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotoredArmSubsystem;

import java.util.Timer;

public class MotoredArmTicks extends CommandBase {
    private MotoredArmSubsystem armSubsystem;
    private ArmModes modes;
    private int desiredPos = 0;
    protected Timer timer;

    public enum ArmModes{
        RETRACTED,
        EXTENDED
    }

    public MotoredArmTicks(MotoredArmSubsystem armSubsystem, ArmModes modes){
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
            case RETRACTED:
                desiredPos = 0; //Adjust these values
                armSubsystem.setArmTicks(desiredPos);
                armSubsystem.setPower(0.5);
            break;

            case EXTENDED:
                desiredPos = -768; //Adjust these values
                armSubsystem.setArmTicks(desiredPos);
                armSubsystem.setPower(0.5);
            break;
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
