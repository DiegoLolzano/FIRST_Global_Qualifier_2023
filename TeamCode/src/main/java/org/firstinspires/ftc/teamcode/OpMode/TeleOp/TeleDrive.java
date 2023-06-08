package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmTicks.ArmModes.SCORE;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmTicks.ArmModes.TRANSIT;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmTicks.ArmModes.FLOOR;

import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.CLOSED;
import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.OPEN;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmPower;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmTicks;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;


@TeleOp
public class TeleDrive extends CommandOpMode {
    DriveTrain m_drive;
    ArmSubsystem m_arm;
    ClawSubsystem m_claw;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_arm = new ArmSubsystem(hardwareMap);
        m_claw = new ClawSubsystem(hardwareMap);

        register(m_drive);

        m_drive.setDefaultCommand(new DriveCommand(m_drive, gamepad1));

        m_arm.setDefaultCommand(new ArmPower(m_arm, gamepad2));

        /*new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.DPAD_UP).whenPressed(new ArmTicks(m_arm, SCORE));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.DPAD_RIGHT).whenPressed(new ArmTicks(m_arm, TRANSIT)
                .alongWith(new DefaultClaw(m_claw, OPEN)));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.DPAD_DOWN).whenPressed(new ArmTicks(m_arm, FLOOR));*/

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new DefaultClaw(m_claw, CLOSED));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.LEFT_BUMPER).whenPressed(new DefaultClaw(m_claw, OPEN));

        schedule(new RunCommand(() -> {
            telemetry.addData("Ticks Arm", m_arm.getArmTicks());
            //Outtake = 655
            //Floor = 1895
            //Transit = 181
            telemetry.update();
        }));

    }
}
