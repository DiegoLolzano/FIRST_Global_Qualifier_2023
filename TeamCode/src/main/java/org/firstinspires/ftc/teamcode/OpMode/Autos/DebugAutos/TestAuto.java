package org.firstinspires.ftc.teamcode.OpMode.Autos.DebugAutos;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.STRAIGHT;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.CENTRAL_TURN;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.SIDE_TURN;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.OFF;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CerbLib.PathAlgorithm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Autonomous (name="Autonomo de Prueba", group="Autos De Prueba")
public class TestAuto extends CommandOpMode {
    DriveTrain m_drive;
    PathAlgorithm pathAlgorithm;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        pathAlgorithm = new PathAlgorithm(m_drive);

        register(m_drive);

        schedule(new SequentialCommandGroup(
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, -30),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, -60),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, -45),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 90),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, -50)
        ));
    }
}
