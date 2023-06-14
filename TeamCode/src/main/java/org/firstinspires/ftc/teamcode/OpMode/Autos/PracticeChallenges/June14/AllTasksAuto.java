package org.firstinspires.ftc.teamcode.OpMode.Autos.PracticeChallenges.June14;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.STRAIGHT;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.POWAAAAAH;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.CENTRAL_TURN;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.SIDE_TURN;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.OFF;

import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultIntake.IntakeMode.STOP;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultIntake.IntakeMode.IN;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultIntake.IntakeMode.OUT;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14.HubArm.HubLevel.LOWER;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14.HubArm.HubLevel.MIDDLE;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14.HubArm.HubLevel.UPPER;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14.HubArm.HubLevel.WORLD_SCORE;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14.HubArm.HubLevel.RETRACTED;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.CerbLib.PathAlgorithm;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June14.HubArm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.DefaultIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;

@Autonomous (name="Autonomo de TODO", group="Reto Junio 14")
public class AllTasksAuto extends CommandOpMode {
    DriveTrain m_drive;
    MotorizedArmSubsystem m_arm;
    IntakeSubsystem m_intake;

    PathAlgorithm pathAlgorithm;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_arm = new MotorizedArmSubsystem(hardwareMap);
        m_intake = new IntakeSubsystem(hardwareMap);

        pathAlgorithm = new PathAlgorithm(m_drive);

        register(m_drive, m_arm, m_intake);

        schedule(new SequentialCommandGroup(
                //World and align with cubes and balls
                new ParallelCommandGroup(
                        new DefaultIntake(m_intake, IN),
                        new HubArm(m_arm, WORLD_SCORE),
                        new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 37.5)),
                 new WaitCommand(1000),
                 new DefaultIntake(m_intake, OUT),
                 new WaitCommand(1000),
                 new DefaultIntake(m_intake, STOP),
                 new ParallelCommandGroup(
                         new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, -37.5),
                         new HubArm(m_arm, RETRACTED)),
                 //Check
                 // robot orientation
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 45),
                new ParallelCommandGroup(
                        new DefaultIntake(m_intake, IN),
                        new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 50)),
                //Check turn
                //direction
                //Cycle 1
                new ParallelCommandGroup(
                        new DefaultIntake(m_intake, STOP),
                        new PathAlgorithmCommand(m_drive, pathAlgorithm, POWAAAAAH, -80)),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 180),
                new HubArm(m_arm, UPPER),
                new DefaultIntake(m_intake, OUT),
                new WaitCommand(1000),
                new ParallelCommandGroup(
                        new DefaultIntake(m_intake, STOP),
                        new HubArm(m_arm, RETRACTED)),
                //Cycle 2
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 180),
                new ParallelCommandGroup(
                        new DefaultIntake(m_intake, IN),
                        new PathAlgorithmCommand(m_drive, pathAlgorithm, POWAAAAAH, 80)),
                new ParallelCommandGroup(
                        new DefaultIntake(m_intake, STOP),
                        new PathAlgorithmCommand(m_drive, pathAlgorithm, POWAAAAAH, -80)),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 180),
                new HubArm(m_arm, UPPER),
                new DefaultIntake(m_intake, OUT),
                new WaitCommand(1000),
                new ParallelCommandGroup(
                        new DefaultIntake(m_intake, STOP),
                        new HubArm(m_arm, RETRACTED)),
                //Finish Cycle and park on barrier
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 90),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 32)
        ));

        /* BACK UP AUTO

        schedule(new SequentialCommandGroup(
                //World and align with cubes and balls
                new ParallelCommandGroup(new InstantCommand(() -> m_intake.setIntakePower(-1.0)),
                        new HubArm(m_arm, WORLD_SCORE),
                        new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 37.5)),
                new WaitCommand(1000),
                new InstantCommand(() -> m_intake.setIntakePower(1.0)),
                new WaitCommand(1000),
                new InstantCommand(() -> m_intake.setIntakePower(0.0)),
                new ParallelCommandGroup(
                        new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, -37.5),
                        new HubArm(m_arm, RETRACTED)),
                //Check
                // robot orientation
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 45),
                new ParallelCommandGroup(
                        new InstantCommand(() -> m_intake.setIntakePower(-1.0)),
                        new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 50)),
                //Check turn
                //direction
                //Cycle 1
                new ParallelCommandGroup(
                        new InstantCommand(() -> m_intake.setIntakePower(0.0)),
                        new PathAlgorithmCommand(m_drive, pathAlgorithm, POWAAAAAH, -80)),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 180),
                new HubArm(m_arm, UPPER),
                new InstantCommand(() -> m_intake.setIntakePower(1.0)),
                new WaitCommand(1000),
                new ParallelCommandGroup(
                        new InstantCommand(() -> m_intake.setIntakePower(0.0)),
                        new HubArm(m_arm, RETRACTED)),
                //Cycle 2
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 180),
                new ParallelCommandGroup(
                        new InstantCommand(() -> m_intake.setIntakePower(-1.0)),
                        new PathAlgorithmCommand(m_drive, pathAlgorithm, POWAAAAAH, 80)),
                new ParallelCommandGroup(
                        new InstantCommand(() -> m_intake.setIntakePower(0.0)),
                        new PathAlgorithmCommand(m_drive, pathAlgorithm, POWAAAAAH, -80)),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 180),
                new HubArm(m_arm, UPPER),
                new InstantCommand(() -> m_intake.setIntakePower(1.0)),
                new WaitCommand(1000),
                new ParallelCommandGroup(
                        new InstantCommand(() -> m_intake.setIntakePower(0.0)),
                        new HubArm(m_arm, RETRACTED)),
                //Finish Cycle and park on barrier
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 90),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 32)
        )); */
    }
}
