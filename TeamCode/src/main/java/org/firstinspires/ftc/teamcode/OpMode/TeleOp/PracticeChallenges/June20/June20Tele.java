package org.firstinspires.ftc.teamcode.OpMode.TeleOp.PracticeChallenges.June20;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@TeleOp
public class June20Tele extends CommandOpMode {
    DriveTrain m_drive;
    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);

        register(m_drive);

        /*new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.A).whenPressed(null);

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.B).whenPressed(null);

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.Y).whenPressed(null);


        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.X).whenPressed(null);*/

        schedule(new RunCommand(() -> {
            //In case of running any telemetry (Driver HUB Dashboard)
        }));
    }
}
