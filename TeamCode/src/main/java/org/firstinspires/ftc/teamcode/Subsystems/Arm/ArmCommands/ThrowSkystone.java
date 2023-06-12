package org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem;

import java.util.Timer;

public class ThrowSkystone extends CommandBase {
    private ArmSubsystem armSubsystem;
    private ClawSubsystem clawSubsystem;
    private int desiredPos = 0;
    protected Timer timer;

    public ThrowSkystone(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem){
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;

        addRequirements(armSubsystem, clawSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.changeToUsingEncoder();
    }

    @Override
    public void execute(){
        desiredPos = 0;
        armSubsystem.setArmTicks(desiredPos);
        armSubsystem.setPower(0.5);

        if(armSubsystem.getArmTicks() == desiredPos){
            clawSubsystem.setLeftClawPos(1.0);
            clawSubsystem.setRightClawPos(1.0);
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

}