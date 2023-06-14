package org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June13;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SelectorVirtualSubsystem;

import java.util.ArrayList;

public class ConeArm extends CommandBase {
    private MotorizedArmSubsystem m_arm;
    private SelectorVirtualSubsystem m_selector;
    private int levelTicks = 0;

    public ConeArm(MotorizedArmSubsystem m_arm, SelectorVirtualSubsystem m_selector){
        this.m_arm = m_arm;
        this.m_selector = m_selector;

        addRequirements(m_arm);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        switch (m_selector.getLevelToPickUp()){
            case "TOP":
                levelTicks = 302;
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
            break;

            case "LEVEL4":
                levelTicks = 0;
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
                break;

            case "LEVEL3":
                levelTicks = 100; //Check Values
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
                break;

            case "LEVEL2":
                levelTicks = 300; //Check Values
                m_arm.setArmTicks(levelTicks);
                m_arm.setPower(0.5);
                break;

            case "BOTTOM":
                levelTicks = 600; //Check Values
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