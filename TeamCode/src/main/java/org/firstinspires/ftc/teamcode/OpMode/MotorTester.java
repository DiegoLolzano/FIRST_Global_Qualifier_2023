package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MotorTesterSubsystem;

@TeleOp(group = "Tester OpModes")
public class MotorTester extends CommandOpMode {
    MotorTesterSubsystem motorTester;

    @Override
    public void initialize() {
        motorTester = new MotorTesterSubsystem(hardwareMap);

        register(motorTester);

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A)
                .whileHeld(() -> motorTester.setMotor0Power(1))
                .whenReleased(() -> motorTester.setMotor0Power(0));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.X)
                .whileHeld(() -> motorTester.setMotor1Power(1))
                .whenReleased(() -> motorTester.setMotor1Power(0));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.Y)
                .whileHeld(() -> motorTester.setMotor2Power(1))
                .whenReleased(() -> motorTester.setMotor2Power(0));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.B)
                .whileHeld(() -> motorTester.setMotor3Power(1))
                .whenReleased(() -> motorTester.setMotor3Power(0));

        schedule(new RunCommand(() -> {
            telemetry.addData("Motor0 Velo", motorTester.getMotor0VeloTicks());
            telemetry.addData("Motor1 Velo", motorTester.getMotor1VeloTicks());
            telemetry.addData("Motor2 Velo", motorTester.getMotor2VeloTicks());
            telemetry.addData("Motor3 Velo", motorTester.getMotor3VeloTicks());
            telemetry.update();
        }));
    }
}
