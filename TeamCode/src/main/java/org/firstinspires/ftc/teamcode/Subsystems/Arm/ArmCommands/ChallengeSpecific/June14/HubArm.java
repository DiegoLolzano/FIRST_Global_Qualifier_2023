package org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SelectorVirtualSubsystem;

public class HubArm extends CommandBase {
    private MotorizedArmSubsystem m_arm;

    SelectorVirtualSubsystem m_selector;

    private int levelTicks = 0;

    public HubArm(MotorizedArmSubsystem m_arm, SelectorVirtualSubsystem m_selector){
        this.m_arm = m_arm;
        this.m_selector = m_selector;
        //this.level = level;

        addRequirements(m_arm);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        switch (m_selector.getLevelToScore()){
            case "LOWER":
                levelTicks = 200; //
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case "MIDDLE":
                levelTicks = -1593; //Check Values //
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case "TOP":
                levelTicks = -2752; //Check Values //
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
