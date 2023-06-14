package org.firstinspires.ftc.teamcode.OpMode.Autos.PracticeChallenges.June9;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.STRAIGHT;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.SIDE_TURN;

import static org.firstinspires.ftc.teamcode.CerbLib.PathAlgorithm.DriveSides.LEFT;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.MotorizedArmTicks.ArmModes.EXTENDED;

import static  org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.CLOSED;

import org.firstinspires.ftc.teamcode.CerbLib.PathAlgorithm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.MotorizedArmTicks;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Autonomous (name="Pieza y Estacionada", group="Reto Junio 9")
public class ScoreAndPark extends CommandOpMode {
    DriveTrain m_drive;
    MotorizedArmSubsystem m_arm;
    ClawSubsystem m_claw;
    PathAlgorithm pathAlgorithm;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_arm = new MotorizedArmSubsystem(hardwareMap);
        m_claw = new ClawSubsystem(hardwareMap);
        pathAlgorithm = new PathAlgorithm(m_drive);

        register(m_drive, m_arm, m_claw);

        schedule(new SequentialCommandGroup(
                new DefaultClaw(m_claw, CLOSED),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 60),
                new DefaultClaw(m_claw, DefaultClaw.ClawModes.OPEN),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, SIDE_TURN, 90, LEFT),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, -60),
                new MotorizedArmTicks(m_arm, EXTENDED)
        ));

    }
}
