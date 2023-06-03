package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@TeleOp
public class TeleDrive extends CommandOpMode {
    DriveTrain m_drive;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);

        m_drive.setDefaultCommand(new DriveCommand(m_drive, gamepad1));
    }
}
