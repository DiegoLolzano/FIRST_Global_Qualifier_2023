package org.firstinspires.ftc.teamcode.OpMode.Autos.PracticeChallenges.June13;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.OFF;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.STRAIGHT;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.CENTRAL_TURN;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.SIDE_TURN;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.JunctionArm.JunctionLevel.RETRACTED;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.JunctionArm.JunctionLevel.LOWER;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.JunctionArm.JunctionLevel.MIDDLE;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.JunctionArm.JunctionLevel.UPPER;

import static org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands.ServoWristControl.WristState.PICK;
import static org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands.ServoWristControl.WristState.LOWER_JUNCTION;
import static org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands.ServoWristControl.WristState.UPPER_JUNCTION;
import static org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands.ServoWristControl.WristState.MIDDLE_JUNCTION;
import static org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands.ServoWristControl.WristState.TRANSPORT;

import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.OPEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.CLOSED;

import org.firstinspires.ftc.teamcode.CerbLib.PathAlgorithm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.JunctionArm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist.Commands.ServoWristControl;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist.ServoedWrist;

@Autonomous (name="Autonomo de pieza y cruce", group="Reto Junio 13")
public class ScoreAndCrossAuto extends CommandOpMode {
    DriveTrain m_drive;
    MotorizedArmSubsystem m_arm;
    ClawSubsystem m_claw;
    ServoedWrist m_wrist;

    PathAlgorithm pathAlgorithm;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_arm = new MotorizedArmSubsystem(hardwareMap);
        m_claw = new ClawSubsystem(hardwareMap);
        m_wrist = new ServoedWrist(hardwareMap);

        pathAlgorithm = new PathAlgorithm(m_drive);

        schedule(new SequentialCommandGroup(
                new DefaultClaw(m_claw, CLOSED),
                new ParallelCommandGroup(
                        new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 45),
                        new JunctionArm(m_arm, UPPER)),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, OFF, 0),
                new WaitCommand(2000),
                new ServoWristControl(m_wrist, UPPER_JUNCTION),
                new WaitCommand(500),
                new DefaultClaw(m_claw, OPEN),
                new WaitCommand(2000),
                new ParallelCommandGroup(
                        new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, -40),
                        new JunctionArm(m_arm, RETRACTED)),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, OFF, 0),
                new WaitCommand(2000),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, -90),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, OFF, 0),
                new WaitCommand(2000),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 125),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, OFF, 0)
        ));

        /* BACKUP AUTO CODE
        * schedule(new SequentialCommandGroup(
        *         new InstantCommand(() -> m_claw.setBothClawPos(1.0)),
        *         new InstantCommand(() -> m_wrist.setWristPos(1.0)),
        *         new ParallelCommandGroup(
        *                 new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 45),
        *                 new JunctionArm(m_arm, UPPER)),
        *         new PathAlgorithmCommand(m_drive, pathAlgorithm, OFF, 0),
        *         new WaitCommand(2000),
        *         new ServoWristControl(m_wrist, SCORE),
        *         new WaitCommand(500),
        *        new DefaultClaw(m_claw, OPEN),
        *         new WaitCommand(2000),
        *         new ParallelCommandGroup(
        *                 new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, -40),
        *                 new JunctionArm(m_arm, RETRACTED)),
        *         new PathAlgorithmCommand(m_drive, pathAlgorithm, OFF, 0),
        *         new WaitCommand(2000),
        *         new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, -90),
        *         new PathAlgorithmCommand(m_drive, pathAlgorithm, OFF, 0),
        *         new WaitCommand(2000),
        *         new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 125),
        *         new PathAlgorithmCommand(m_drive, pathAlgorithm, OFF, 0)
        * ));
        */
    }
}
