package org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;

public class HubArm extends CommandBase {
    private MotorizedArmSubsystem m_arm;
    private HubLevel level;
    private int levelTicks = 0;

    public enum HubLevel {
        LOWER,
        MIDDLE,
        UPPER,
        WORLD_SCORE,
        RETRACTED
    }

    public HubArm(MotorizedArmSubsystem m_arm, HubLevel level){
        this.m_arm = m_arm;
        this.level = level;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        switch (level){
            case RETRACTED:
                levelTicks = 0; //Check Values
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case LOWER:
                levelTicks = 200;
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case MIDDLE:
                levelTicks = 410; //Check Values
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case UPPER:
                levelTicks = 903; //Check Values
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case WORLD_SCORE:
                levelTicks = 1400; //Check Values
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
