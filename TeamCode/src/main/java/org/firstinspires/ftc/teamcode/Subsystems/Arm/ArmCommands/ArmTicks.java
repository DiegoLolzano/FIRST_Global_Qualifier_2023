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
        RETRACTED,
        VERTICAL,
        EXTENDED,
        SKYSTONE
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
            case RETRACTED:
                desiredPos = 0; //Adjust these values
                armSubsystem.setArmTicks(desiredPos);
                armSubsystem.setPower(0.5);
            break;

            case EXTENDED:
                desiredPos = 669; //Adjust these values
                armSubsystem.setArmTicks(desiredPos);
                armSubsystem.setPower(0.5);
            break;

            case VERTICAL:
                desiredPos = 309; //Adjust these values
                armSubsystem.setArmTicks(desiredPos);
                armSubsystem.setPower(0.5);

            case SKYSTONE:
                desiredPos = 500;
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