package org.firstinspires.ftc.teamcode.OpMode.Autos.PracticeChallenges.June9;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import  static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.OFF;
import  static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.STRAIGHT;
import  static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.CENTRAL_TURN;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.MotorizedArmTicks.ArmModes.RETRACTED;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.MotorizedArmTicks.ArmModes.EXTENDED;


import org.firstinspires.ftc.teamcode.CerbLib.PathAlgorithm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.MotorizedArmTicks;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ServoedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem2;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Autonomous (name="Pieza y Estacionada Tamal", group="Reto Junio 9")
public class ScoreAndParkTamal extends CommandOpMode {
    DriveTrain m_drive;
    ServoedArmSubsystem servoedArm;
    ClawSubsystem2 servoedClaw;
    MotorizedArmSubsystem motoredArm;

    PathAlgorithm pathAlgorithm;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        servoedArm = new ServoedArmSubsystem(hardwareMap);
        servoedClaw = new ClawSubsystem2(hardwareMap);
        motoredArm = new MotorizedArmSubsystem(hardwareMap);

        pathAlgorithm = new PathAlgorithm(m_drive);

        register(m_drive, motoredArm);

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> servoedClaw.setBotServos2Pos(1.0)),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 87),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, OFF, 0),
                new InstantCommand(() -> servoedArm.setServoArmPos(0.45)),
                new WaitCommand(1000),
                new InstantCommand(() -> servoedClaw.setBotServos2Pos(0.0)),
                new InstantCommand(() -> servoedArm.setServoArmPos(0.9)),
                //new ArmServo(servoedArm, DOWN),
                //new DefaultClaw2(servoedClaw, OPEN2),
                //new ParallelCommandGroup(new ArmServo(servoedArm, UP)),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, -10),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 100),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 100),
                new MotorizedArmTicks(motoredArm, EXTENDED),
                new WaitCommand(8),
                new MotorizedArmTicks(motoredArm, RETRACTED)
        ));

    }
}
