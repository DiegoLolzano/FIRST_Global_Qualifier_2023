package org.firstinspires.ftc.teamcode.OpMode.TeleOp.PracticeChallenges.June20;

import static org.firstinspires.ftc.teamcode.Subsystems.Basket.Commands.DefaultBasket.BasketModes.INDEX;
import static org.firstinspires.ftc.teamcode.Subsystems.Basket.Commands.DefaultBasket.BasketModes.SCORE;
import static org.firstinspires.ftc.teamcode.Subsystems.Indexer.Commands.DefaultIndexer.IntakeMode.IN_INDEXER;
import static org.firstinspires.ftc.teamcode.Subsystems.Indexer.Commands.DefaultIndexer.IntakeMode.OUT_INDEXER;
import static org.firstinspires.ftc.teamcode.Subsystems.Indexer.Commands.DefaultIndexer.IntakeMode.STOP_INDEXER;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultDozzyIntake.IntakeMode.IN;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultDozzyIntake.IntakeMode.OUT;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultDozzyIntake.IntakeMode.STOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Basket.BasketSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.DozzyIntake;

@Config
@TeleOp
public class CalibrationTele20 extends CommandOpMode {
    //Add subystems to calibrate
    IndexerSubsystem m_indexer;
    BasketSubsystem m_basket;
    DozzyIntake m_intake;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    public static double intakePower = 0.0;
    public static double indexerPower = 0.0;
    public static double basketPose = 0.0;

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
        m_basket = new BasketSubsystem(hardwareMap);
        m_indexer = new IndexerSubsystem(hardwareMap);
        m_intake = new DozzyIntake(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry();

        register(m_basket, m_indexer, m_intake);

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.A).whenPressed(
                        new InstantCommand(() -> m_basket.setBasketPos(basketPose)));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.X).whileHeld(
                        new InstantCommand(() -> m_intake.setIntakePower(intakePower))
                                .alongWith(
                                        new InstantCommand(() -> m_indexer.setIndexerPower(indexerPower))))
                .whenReleased(new InstantCommand(() -> m_intake.setIntakePower(0.0))
                        .alongWith(
                                new InstantCommand(() -> m_indexer.setIndexerPower(0.0))));

        schedule(new RunCommand(() -> {
            //In case of running any telemetry (Driver HUB Dashboard)
        }));
    }
}
