package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.Basket.Commands.DefaultBasket.BasketModes.TRANSIT;
import static org.firstinspires.ftc.teamcode.Subsystems.Basket.Commands.DefaultBasket.BasketModes.INDEX;
import static org.firstinspires.ftc.teamcode.Subsystems.Basket.Commands.DefaultBasket.BasketModes.SCORE;

import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.OPEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.CLOSED;

import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultIntake.IntakeMode.OFF;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultIntake.IntakeMode.IN;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultIntake.IntakeMode.OUT;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmPower;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotoredArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Basket.BasketSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Basket.Commands.DefaultBasket;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;


@TeleOp
public class TeleDrive extends CommandOpMode {
    DriveTrain m_drive;
    MotoredArmSubsystem m_arm;
    ClawSubsystem m_claw;
    BasketSubsystem m_basket;
    IntakeSubsystem m_intake;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_arm = new MotoredArmSubsystem(hardwareMap);
        m_claw = new ClawSubsystem(hardwareMap);
        m_basket = new BasketSubsystem(hardwareMap);
        m_intake = new IntakeSubsystem(hardwareMap);

        register(m_drive);

        m_drive.setDefaultCommand(new DriveCommand(m_drive, gamepad1));

        m_arm.setDefaultCommand(new ArmPower(m_arm, gamepad2));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.A).whileHeld(new DefaultIntake(m_intake, IN))
                .whenReleased(new DefaultIntake(m_intake, OFF));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.B).whileHeld(new DefaultIntake(m_intake, OUT))
                .whenReleased(new DefaultIntake(m_intake, OFF));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.LEFT_BUMPER).whenPressed(new DefaultClaw(m_claw, OPEN));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new DefaultClaw(m_claw, CLOSED));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.X).whenPressed(new DefaultBasket(m_basket, SCORE));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new DefaultBasket(m_basket, TRANSIT));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.B).whenPressed(new DefaultBasket(m_basket, INDEX));

        schedule(new RunCommand(() -> {
            telemetry.addData("Ticks Arm", m_arm.getArmTicks());
            telemetry.addData("Left Drive Ticks", m_drive.getLeftCurrentPos());
            telemetry.addData("Right Drive Ticks", m_drive.getRightCurrentPos());
            //Outtake = 655
            //Floor = 1895
            //Transit = 181
            telemetry.update();
        }));

    }
}
