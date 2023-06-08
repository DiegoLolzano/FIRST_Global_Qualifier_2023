package org.firstinspires.ftc.teamcode.OpMode.Autos.PracticeChallenges;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.STRAIGHT;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.CENTRAL_TURN;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.SIDE_TURN;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.OFF;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmTicks.ArmModes.SCORE;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmTicks.ArmModes.TRANSIT;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmTicks.ArmModes.FLOOR;

import org.firstinspires.ftc.teamcode.CerbLib.PathAlgorithm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmTicks;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Autonomous
public class RedParkAuto extends CommandOpMode {
    DriveTrain m_drive;
    PathAlgorithm pathAlgorithm;
    ArmSubsystem m_arm;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_arm = new ArmSubsystem(hardwareMap);
        pathAlgorithm = new PathAlgorithm(m_drive);

        register(m_drive);

        schedule(new SequentialCommandGroup(
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, -60),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 90),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, -60),
                new ArmTicks(m_arm, FLOOR)));
    }
}
