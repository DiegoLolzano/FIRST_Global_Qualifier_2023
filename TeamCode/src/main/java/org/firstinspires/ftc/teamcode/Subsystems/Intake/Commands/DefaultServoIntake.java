package org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.ContinuousServoIntake;

public class DefaultServoIntake extends CommandBase {
    private ContinuousServoIntake m_intake;
    private IntakeMode mode;

    public enum IntakeMode{
        IN_SERVO,
        OUT_SERVO,
        STOP_SERVO
    }

    public DefaultServoIntake(ContinuousServoIntake m_intake, IntakeMode mode){
        this.m_intake = m_intake;
        this.mode = mode;

        addRequirements(m_intake);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        switch (mode){
            case IN_SERVO:
                m_intake.setServoIntakePower(-1.0);
            break;

            case OUT_SERVO:
                m_intake.setServoIntakePower(1.0);
            break;

            case STOP_SERVO:
                m_intake.setServoIntakePower(0.0);
            break;
        }
    }

    @Override
    public void end(boolean interrupted){
        m_intake.setServoIntakePower(0.0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
