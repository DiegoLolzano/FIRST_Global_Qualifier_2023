package org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.ServoedArmSubsystem;

public class ServoedArm extends CommandBase {
    private ServoedArmSubsystem servoedArm;
    private ServoArmState state;

    public enum ServoArmState{
        UP,
        DOWN
    }

    public ServoedArm(ServoedArmSubsystem servoedArm, ServoArmState state){
        this.servoedArm = servoedArm;
        this.state = state;

        addRequirements(servoedArm);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        switch (state){
            case UP:
                servoedArm.setServoArmPos(0.9);
            break;

            case DOWN:
                servoedArm.setServoArmPos(0.45);
            break;
        }
    }

    @Override
    public void end(boolean interrupted){
        servoedArm.setServoArmPos(0.9);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
