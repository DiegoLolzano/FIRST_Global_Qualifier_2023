package org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Testing.ControlHUBTester;

public class FULLTEST extends CommandBase {
    private ControlHUBTester m_tester;
    private TestBeds beds;

    public enum TestBeds{
        MOTOR_TO_POS,
        MOTOR_POWER,
        MOTOR_STOP,
        SERVO_POS0,
        SERVO_POS1
    }

    public FULLTEST(ControlHUBTester m_tester, TestBeds beds){
        this.m_tester = m_tester;
        this.beds = beds;

        addRequirements(m_tester);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        switch (beds){
            case MOTOR_STOP:
                m_tester.returnToUsingEncoder();
                m_tester.setMotorPower(0.0);
            break;

            case MOTOR_POWER:
                m_tester.returnToUsingEncoder();
                m_tester.setMotorPower(0.5);
            break;

            case MOTOR_TO_POS:
                m_tester.motorToPose(500);
            break;

            case SERVO_POS0:
                m_tester.setServoPose(0.0);
            break;

            case SERVO_POS1:
                m_tester.setServoPose(1.0);
            break;
        }
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
