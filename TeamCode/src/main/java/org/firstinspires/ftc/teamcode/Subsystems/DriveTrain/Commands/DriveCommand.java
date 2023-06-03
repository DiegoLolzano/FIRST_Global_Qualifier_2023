package org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

public class DriveCommand extends CommandBase {
    private DriveTrain m_drive;
    private Gamepad gamepad;

    public DriveCommand(DriveTrain m_drive, Gamepad gamepad){
        this.m_drive = m_drive;
        this.gamepad = gamepad;

        addRequirements(m_drive);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if(gamepad.right_trigger > 0.2)m_drive.setCheesyishDrive(0, -0.6, true);
        if(gamepad.left_trigger > 0.2)m_drive.setCheesyishDrive(0, 0.6, true);

        m_drive.setCheesyishDrive(
                gamepad.left_bumper? gamepad.left_stick_y * 0.1: gamepad.left_stick_y,
                gamepad.left_bumper? -gamepad.right_stick_x * 0.1: -gamepad.right_stick_x,
                true);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
