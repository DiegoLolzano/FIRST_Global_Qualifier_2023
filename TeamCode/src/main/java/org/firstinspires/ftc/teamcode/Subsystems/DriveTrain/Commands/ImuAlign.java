package org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

public class ImuAlign extends CommandBase {
    private DriveTrain m_drive;
    private PIDController angularController;

    double desiredAngle;

    public ImuAlign(DriveTrain m_drive, double desiredAngle){
        this.m_drive = m_drive;
        this.desiredAngle = desiredAngle;

        angularController = new PIDController(1.5, 0, 0);

        angularController.setTolerance(0.05);

        addRequirements(m_drive);
    }

    @Override
    public void initialize(){
        angularController.reset();

    }

    @Override
    public void execute(){
        angularController.setSetPoint(desiredAngle);

        m_drive.setCheesyishDrive(0.0,
                angularController.calculate(m_drive.getAngle()), true);
    }

    @Override
    public void end(boolean interrupted){
        m_drive.setLeftPower(0);
        m_drive.setRightPower(0);
    }

    @Override
    public boolean isFinished(){
        return angularController.atSetPoint();
    }
}
