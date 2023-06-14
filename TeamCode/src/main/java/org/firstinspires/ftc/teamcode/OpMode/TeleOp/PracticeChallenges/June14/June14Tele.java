package org.firstinspires.ftc.teamcode.OpMode.TeleOp.PracticeChallenges.June14;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultIntake.IntakeMode.STOP;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultIntake.IntakeMode.IN;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultIntake.IntakeMode.OUT;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14.HubArm.HubLevel.LOWER;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14.HubArm.HubLevel.MIDDLE;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14.HubArm.HubLevel.UPPER;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14.HubArm.HubLevel.RETRACTED;


import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14.HubArm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;

@TeleOp (name="Tele de Junio 14", group="Reto Junio 14")
public class June14Tele extends CommandOpMode {
    DriveTrain m_drive;
    IntakeSubsystem m_intake;
    MotorizedArmSubsystem m_arm;

    /*
     * System Map
     * leftDrive Motor Port:  1
     * rightDrive Motor Port: 0
     * armMotor Motor Port:   2
     *
     * intakeMotor Motor Port: 3;
     */

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap); //16:1
        m_intake = new IntakeSubsystem(hardwareMap);
        m_arm = new MotorizedArmSubsystem(hardwareMap);

        register(m_drive, m_arm, m_intake);

        m_drive.setDefaultCommand(new DriveCommand(m_drive, gamepad1));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.LEFT_BUMPER).whileHeld(new DefaultIntake(m_intake, IN))
                .whenReleased(new DefaultIntake(m_intake, STOP));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.RIGHT_BUMPER).whileHeld(new DefaultIntake(m_intake, OUT))
                .whenReleased(new DefaultIntake(m_intake, STOP));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.DPAD_UP).whenPressed(new HubArm(m_arm, UPPER));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.DPAD_RIGHT).whenPressed(new HubArm(m_arm, MIDDLE));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.DPAD_LEFT).whenPressed(new HubArm(m_arm, LOWER));

        new GamepadButton(new GamepadEx(gamepad2),
                GamepadKeys.Button.DPAD_DOWN).whenPressed(new HubArm(m_arm, RETRACTED));

        schedule(new RunCommand(() -> {
            telemetry.addData("Arm Encoder", m_arm.getArmTicks());
            telemetry.update();
        }));
    }
}
