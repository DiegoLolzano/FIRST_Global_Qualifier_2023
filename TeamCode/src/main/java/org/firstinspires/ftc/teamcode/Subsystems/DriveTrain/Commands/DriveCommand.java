package org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

public class DriveCommand extends CommandBase {
    private DriveTrain m_drive;
    private Gamepad gamepad;
    private boolean isReversed;

    public DriveCommand(DriveTrain m_drive, Gamepad gamepad){
        this.m_drive = m_drive;
        this.gamepad = gamepad;
        isReversed = false;

        addRequirements(m_drive);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        m_drive.setCheesyishDrive(gamepad);

        if(gamepad.right_stick_button){
            toggleFront();
        }

        if(!isReversed){
            m_drive.invertMotors(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        } else {
            m_drive.invertMotors(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);
        }
    }

    @Override
    public void end(boolean interrupted){
        m_drive.setLeftPower(0);
        m_drive.setRightPower(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    public void toggleFront(){
        isReversed = !isReversed;
    }
}
