package org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;

public class DefaultIntake extends CommandBase {
    private IntakeSubsystem m_intake;
    private IntakeMode intakeMode;

    public enum IntakeMode{
        IN,
        OUT,
        OFF
    }

    public DefaultIntake(IntakeSubsystem m_intake, IntakeMode intakeMode){
        this.m_intake = m_intake;
        this.intakeMode = intakeMode;

        addRequirements(m_intake);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        switch (intakeMode){
            case OFF:
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
        return false;
    }
}
