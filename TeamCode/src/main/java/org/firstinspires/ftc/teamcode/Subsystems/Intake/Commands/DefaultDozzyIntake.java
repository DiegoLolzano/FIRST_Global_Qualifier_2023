package org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.DozzyIntake;

public class DefaultDozzyIntake extends CommandBase {
    private DozzyIntake m_intake;
    private Gamepad gamepad;
    private IntakeMode intakeMode;

    public enum IntakeMode{
        IN,
        OUT,
        STOP
    }

    public DefaultDozzyIntake(DozzyIntake m_intake, IntakeMode intakeMode){
        this.m_intake = m_intake;
        this.intakeMode = intakeMode;

        addRequirements(m_intake);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        switch (intakeMode){
            case STOP:
                m_intake.setIntakePower(0.0);
            break;

            case IN:
                m_intake.setIntakePower(-1.0);
            break;

            case OUT:
                m_intake.setIntakePower(1.0);
            break;
        }
    }

    @Override
    public void end(boolean isInterrupted){}

    @Override
    public boolean isFinished(){
        return m_intake.isIntakeBusy() & gamepad == null;
    }
}
