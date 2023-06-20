package org.firstinspires.ftc.teamcode.OpMode.TeleOp.PracticeChallenges.June20;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class CalibrationTele20 extends CommandOpMode {
    //Add subystems to calibrate

    FtcDashboard dashboard;
    TelemetryPacket packet;

    //Add variables in a public static form to live tune
    @Override
    public void initialize() {

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry();

        //Remove comments to add commands
        /*new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.A).whenPressed(null);

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.B).whenPressed(null);

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.Y).whenPressed(null);


        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.X).whenPressed(null);*/

        schedule(new RunCommand(() -> {
            //Add the subsystem info
            //packet.put();

            dashboard.sendTelemetryPacket(packet);
        }));
    }
}
