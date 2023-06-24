package org.firstinspires.ftc.teamcode.OpMode.TeleOp.PracticeChallenges.June20;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.Indexer.Commands.DefaultIndexer.IntakeMode.STOP_INDEXER;

import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultDozzyIntake.IntakeMode.IN;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultDozzyIntake.IntakeMode.OUT;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultDozzyIntake.IntakeMode.STOP;

import static org.firstinspires.ftc.teamcode.Subsystems.Basket.Commands.DefaultBasket.BasketModes.SCORE;
import static org.firstinspires.ftc.teamcode.Subsystems.Basket.Commands.DefaultBasket.BasketModes.INDEX;

import static org.firstinspires.ftc.teamcode.Subsystems.Indexer.Commands.DefaultIndexer.IntakeMode.OUT_INDEXER;
import static org.firstinspires.ftc.teamcode.Subsystems.Indexer.Commands.DefaultIndexer.IntakeMode.IN_INDEXER;

import org.firstinspires.ftc.teamcode.Subsystems.Basket.BasketSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Basket.Commands.DefaultBasket;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.Commands.DefaultIndexer;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultDozzyIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.DozzyIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.IndexerSubsystem;

@TeleOp
public class June20Tele extends CommandOpMode {
    DriveTrain m_drive;
    IndexerSubsystem m_indexer;
    BasketSubsystem m_basket;
    DozzyIntake m_intake;

    /*
    * Ports
    * leftDrive 0
    * rightDrive 1
    *
    * intakeMotor 2
    * indexerMotor 3
    * basketServo 0
    *
    */

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_basket = new BasketSubsystem(hardwareMap);
        m_indexer = new IndexerSubsystem(hardwareMap);
        m_intake = new DozzyIntake(hardwareMap);

        register(m_drive);

        m_drive.setDefaultCommand(new DriveCommand(m_drive, gamepad1));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.LEFT_BUMPER).whenPressed(new DefaultBasket(m_basket, INDEX));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new DefaultBasket(m_basket, SCORE));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.A).whenPressed(
                        new DefaultDozzyIntake(m_intake, IN)
                                .alongWith(new DefaultIndexer(m_indexer, IN_INDEXER)))
                .whenReleased(new DefaultDozzyIntake(m_intake, STOP)
                        .alongWith(new DefaultIndexer(m_indexer, STOP_INDEXER)));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.B).whileHeld(
                        new DefaultDozzyIntake(m_intake, OUT)
                                .alongWith(new DefaultIndexer(m_indexer, OUT_INDEXER)))
                .whenReleased(new DefaultDozzyIntake(m_intake, STOP)
                        .alongWith(new DefaultIndexer(m_indexer, STOP_INDEXER)));

        schedule(new RunCommand(() -> {
            //In case of running any telemetry (Driver HUB Dashboard)
            telemetry.addData("Left speed", m_drive.getLeftVelocity());
            telemetry.addData("Right speed", m_drive.getRightVelocity());
        }));

        /*
        * Diego Lozano was here.
        * We'll get 'em next time <3
        */
    }
}
