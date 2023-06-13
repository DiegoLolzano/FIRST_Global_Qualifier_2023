package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.OPEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.CLOSED;

import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw2.ClawModes2.OPEN2;
import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw2.ClawModes2.CLOSED2;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmServo.ServoArmState.UP;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmServo.ServoArmState.DOWN;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmTicks.ArmModes.EXTENDED;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmTicks.ArmModes.RETRACTED;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmPower;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmServo;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmTicks;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotoredArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ServoedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem2;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw2;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@TeleOp
public class TamalTeleOp extends CommandOpMode {
    DriveTrain m_drive;
    MotoredArmSubsystem motoredArm;
    ServoedArmSubsystem servoedArm;
    ClawSubsystem motoredArmClaw;
    ClawSubsystem2 servoedArmClaw;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        motoredArm = new MotoredArmSubsystem(hardwareMap);
        servoedArm = new ServoedArmSubsystem(hardwareMap);
        motoredArmClaw = new ClawSubsystem(hardwareMap);
        servoedArmClaw = new ClawSubsystem2(hardwareMap);

        register(m_drive, motoredArm, servoedArm, motoredArmClaw, servoedArmClaw);

        m_drive.setDefaultCommand(new DriveCommand(m_drive, gamepad1));

        //motoredArm.setDefaultCommand(new ArmPower(motoredArm, gamepad2));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.A).whenPressed(new ArmTicks(motoredArm, EXTENDED));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.Y).whenPressed(new ArmTicks(motoredArm, RETRACTED));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.LEFT_BUMPER).whenPressed(new DefaultClaw2(servoedArmClaw, OPEN2));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new DefaultClaw2(servoedArmClaw, CLOSED2));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.A).whenPressed(new ArmServo(servoedArm, DOWN));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.Y).whenPressed(new ArmServo(servoedArm, UP));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.LEFT_BUMPER).whenPressed(new DefaultClaw(motoredArmClaw, OPEN));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new DefaultClaw(motoredArmClaw, CLOSED));

        schedule(new RunCommand(() -> {
            telemetry.addData("Arm Ticks", motoredArm.getArmTicks());
            telemetry.update();
        }));
    }
}
