package org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotoredArmSubsystem;

public class ArmPower extends CommandBase {
    MotoredArmSubsystem armSubsystem;
    int desiredPos;
    Gamepad gamepad;

    public ArmPower(MotoredArmSubsystem motoredArmSubsystem, Gamepad gamepad){
        this.armSubsystem = motoredArmSubsystem;
        this.gamepad = gamepad;

        addRequirements(motoredArmSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.changeToUsingEncoder();
    }

    @Override
    public void execute(){
        if(gamepad.a){
            armSubsystem.changeToUsingEncoder();
            armSubsystem.setPower(-0.6);
        } else if(gamepad.y){
            armSubsystem.changeToUsingEncoder();
            armSubsystem.setPower(0.6);
        } else if(gamepad.dpad_up){
            desiredPos = 309;
            armSubsystem.setArmTicks(desiredPos);
            armSubsystem.setPower(0.5);
        } else if(gamepad.dpad_down){
            desiredPos = 500;
            armSubsystem.setArmTicks(desiredPos);
            armSubsystem.setPower(0.5);
        } else if(gamepad.dpad_right){
            desiredPos = 0;
            armSubsystem.setArmTicks(desiredPos);
            armSubsystem.setPower(0.5);
        } else if(gamepad.dpad_left) {
            desiredPos = 669;
            armSubsystem.setArmTicks(desiredPos);
            armSubsystem.setPower(0.5);
        }else {
            armSubsystem.setPower(0.0);
        }
    }

    @Override
    public void end(boolean interrupted){
        armSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
