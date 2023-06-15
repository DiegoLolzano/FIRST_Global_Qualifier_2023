package org.firstinspires.ftc.teamcode.OpMode.TeleOp.DebugTele;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@TeleOp
public class OnlyDriveTele extends CommandOpMode {
    DriveTrain m_drive;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);

        register(m_drive);

        m_drive.setDefaultCommand(new DriveCommand(m_drive, gamepad1));

        schedule(new RunCommand(() -> {
            telemetry.addData("left inches",
                    DriveConstants.encoderTicksToInches(m_drive.getLeftCurrentPos()));

            telemetry.addData("right inches",
                    DriveConstants.encoderTicksToInches(m_drive.getRightCurrentPos()));

            telemetry.update();
        }));
    }
}
