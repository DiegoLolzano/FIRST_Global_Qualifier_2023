package org.firstinspires.ftc.teamcode.OpMode.TeleOp.PracticeChallenges.June13;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.JunctionArm.JunctionLevel.LOWER;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.JunctionArm.JunctionLevel.MIDDLE;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.JunctionArm.JunctionLevel.RETRACTED;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.JunctionArm.JunctionLevel.UPPER;
import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.CLOSED;
import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.OPEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands.ServoWristControl.WristState.LOWER_JUNCTION;
import static org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands.ServoWristControl.WristState.MIDDLE_JUNCTION;
import static org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands.ServoWristControl.WristState.PICK;
import static org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands.ServoWristControl.WristState.TRANSPORT;
import static org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands.ServoWristControl.WristState.UPPER_JUNCTION;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.ConeArm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.JunctionArm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.SelectorVirtualSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands.ServoWristControl;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist.ServoedWrist;

@TeleOp(name="TeleOp de Calibracion", group="Reto Junio 13")
public class CalibrationTele extends CommandOpMode {
    DriveTrain m_drive;
    MotorizedArmSubsystem m_arm;
    ClawSubsystem m_claw;
    ServoedWrist m_wrist;
    SelectorVirtualSubsystem m_selector;

    /*
     * System Map
     * leftDrive Motor Port:  1
     * rightDrive Motor Port: 0
     * armMotor Motor Port:   2
     *
     * clawLeft Servo Port:   1
     * clawRight Servo Port: 0
     * wristServo Servo Port: 2
     */



    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_arm = new MotorizedArmSubsystem(hardwareMap);
        m_claw = new ClawSubsystem(hardwareMap);
        m_wrist = new ServoedWrist(hardwareMap);
        m_selector = new SelectorVirtualSubsystem(gamepad1);

        m_drive.setDefaultCommand(new DriveCommand(m_drive, gamepad1));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.LEFT_BUMPER).whenPressed(new DefaultClaw(m_claw, OPEN));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new DefaultClaw(m_claw, CLOSED));


        schedule(new RunCommand(() -> {
            telemetry.addData("Arm Ticks", m_arm.getArmTicks());
            telemetry.addData("Servo Pos", m_wrist.getServoPosition());
            telemetry.addData("Current Selected Level", m_selector.getLevelToPickUp());
            telemetry.update();
        }));
    }
}