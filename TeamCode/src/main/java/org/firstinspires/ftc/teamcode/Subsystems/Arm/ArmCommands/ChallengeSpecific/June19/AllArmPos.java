package org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June19;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SelectorVirtualSubsystem;

public class AllArmPos extends CommandBase {
    private MotorizedArmSubsystem m_arm;
    private ArmPoses poses;

    public enum ArmPoses{
        AUTO_PRELOAD,
        WORLD_SCORE,
        INTAKE_POSE,
        TOP,
        MID,
        LOW,
        TRANSIT,
        RETRACT
    }

    //SelectorVirtualSubsystem m_selector;

    private int levelTicks = 0;

    public AllArmPos(MotorizedArmSubsystem m_arm, ArmPoses poses){
        this.m_arm = m_arm;
        this.poses = poses;
        //this.m_selector = m_selector;
        //this.level = level;

        addRequirements(m_arm);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        switch (poses){
            case RETRACT:
                levelTicks = 0;
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case LOW:
                levelTicks = -4100;
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case MID:
                levelTicks = -3600;
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case TOP:
                levelTicks = -3200;
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case WORLD_SCORE:
                levelTicks = -1700;
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case AUTO_PRELOAD:
                levelTicks = -2300;
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case INTAKE_POSE:
                levelTicks = -4300;
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case TRANSIT:
                levelTicks = -3500;
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;
        }
    }

    @Override
    public void end(boolean interrupted){
        m_arm.setPower(0.0);
    }

    @Override
    public boolean isFinished(){
        return !m_arm.isArmBusy();
    }
}
