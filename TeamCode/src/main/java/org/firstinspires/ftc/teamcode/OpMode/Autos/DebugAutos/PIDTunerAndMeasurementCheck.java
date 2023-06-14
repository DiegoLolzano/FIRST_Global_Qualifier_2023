package org.firstinspires.ftc.teamcode.OpMode.Autos.DebugAutos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.STRAIGHT;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.POWAAAAAH;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.CENTRAL_TURN;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.SIDE_TURN;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.OFF;

import org.firstinspires.ftc.teamcode.CerbLib.PathAlgorithm;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Autonomous (name="Autonomo de Medicion", group="Autos De Prueba")
public class PIDTunerAndMeasurementCheck extends CommandOpMode {
    DriveTrain m_drive;

    PathAlgorithm pathAlgorithm;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);

        pathAlgorithm = new PathAlgorithm(m_drive);

        register(m_drive);

        schedule(new SequentialCommandGroup(
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 30),
                new WaitCommand(5000),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 90)
        ));

        schedule(new RunCommand(() -> {
            telemetry.addData("Inches Left",
                    DriveConstants.encoderTicksToInches(m_drive.getLeftCurrentPos()));

            telemetry.addData("Inches Right",
                    DriveConstants.encoderTicksToInches(m_drive.getRightCurrentPos()));

            telemetry.addData("Ticks Left",
                    m_drive.getRightCurrentPos());

            telemetry.addData("Ticks Right",
                    m_drive.getRightCurrentPos());


            telemetry.update();
        }));
    }
}
