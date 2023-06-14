package org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;

public class MotorizedArmPower extends CommandBase {
    MotorizedArmSubsystem armSubsystem;
    int desiredPos;
    Gamepad gamepad;

    public MotorizedArmPower(MotorizedArmSubsystem motoredArmSubsystem, Gamepad gamepad){
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
        if(gamepad.dpad_down){
            armSubsystem.changeToUsingEncoder();
            armSubsystem.setPower(-0.6);
        } else if(gamepad.dpad_up){
            armSubsystem.changeToUsingEncoder();
            armSubsystem.setPower(0.6);
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
