package org.firstinspires.ftc.teamcode.OpMode.TeleOp.PracticeChallenges.June19;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June19.AllArmPos.ArmPoses.INTAKE_POSE;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June19.AllArmPos.ArmPoses.LOW;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June19.AllArmPos.ArmPoses.MID;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June19.AllArmPos.ArmPoses.TOP;
import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.CLOSED;
import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.OPEN;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June19.AllArmPos.ArmPoses.RETRACT;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June19.AllArmPos.ArmPoses.WORLD_SCORE;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14.HubArm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June19.AllArmPos;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.MotorizedArmTicks;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ServoedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.SingleServoClaw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.SelectorVirtualSubsystem;

@TeleOp
@Config
public class June19Tele extends CommandOpMode {
    DriveTrain m_drive;
    MotorizedArmSubsystem m_motorizedArm;
    ClawSubsystem m_doubleClaw;
    SelectorVirtualSubsystem m_selector;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    public static int ArmTicksPos = 0;
    public static double singleClawPos = 0.0;
    public static double doubleClawPos = 0.0;
    public static double servoArmPos = 0.0;

    /*
    * 0 leftDrive
    * 1 rightDrive
    * 2 armMotor
    *
    * 0 brazo
    * 1 singleClaw
    * 2 leftServo
    * 3 rightServo
    */
    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_motorizedArm = new MotorizedArmSubsystem(hardwareMap);
        m_doubleClaw = new ClawSubsystem(hardwareMap);
        m_selector = new SelectorVirtualSubsystem(gamepad2);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        register(m_drive, m_motorizedArm, m_doubleClaw);

        m_drive.setDefaultCommand(new DriveCommand(m_drive, gamepad1));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.DPAD_DOWN).whenPressed(
                        new AllArmPos(m_motorizedArm, INTAKE_POSE));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new AllArmPos(m_motorizedArm, LOW));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new AllArmPos(m_motorizedArm, MID));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.DPAD_UP).whenPressed(
                new AllArmPos(m_motorizedArm, TOP));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> m_selector.updateSelectionUp()));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> m_selector.updateSelectionDown()));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                        new DefaultClaw(m_doubleClaw, OPEN));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new DefaultClaw(m_doubleClaw, CLOSED));


        //TELEMETRY
        schedule(new RunCommand(() -> {
            packet.put("Robot Angle", m_drive.getAngle());
            packet.put("Arm Ticks", m_motorizedArm.getArmTicks());
            packet.put("Selected Level", m_selector.getLevelToScore());
            telemetry.addData("Selected Level", m_selector.getLevelToScore());

            dashboard.sendTelemetryPacket(packet);
        }));

    }
}
