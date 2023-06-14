package org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;

public class JunctionArm extends CommandBase {
    private MotorizedArmSubsystem m_arm;
    private JunctionLevel level;
    private int levelTicks = 0;

    public enum JunctionLevel {
        RETRACTED,
        LOWER,
        MIDDLE,
        UPPER
    }

    public JunctionArm(MotorizedArmSubsystem m_arm, JunctionLevel level){
        this.m_arm = m_arm;
        this.level = level;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        switch (level){
            case RETRACTED:
                levelTicks = 0;
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case LOWER:
                levelTicks = 410; //Check Values
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case MIDDLE:
                levelTicks = 903; //Check Values
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case UPPER:
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
