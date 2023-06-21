package org.firstinspires.ftc.teamcode.OpMode.Autos.PracticeChallenges.June20;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand.IMUAlgorithmModes.STRAIGHT_IMU;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand.IMUAlgorithmModes.TURN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CerbLib.IMUPathAlgorithm;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.DozzyIntake;

@Config
@Autonomous
public class TwoBallsAndParkRED extends CommandOpMode {
    DriveTrain m_drive;
    DozzyIntake m_intake;
    IndexerSubsystem m_indexer;

    IMUPathAlgorithm pathAlgorithm;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    public static double crashBall = -110;
    public static double backUpForNextBall = 35;
    public static double turnToNextBall = 83; //263
    public static double goToNextBall = -70;
    public static double turnToBall = -7; //187
    public static double hitNextBall = -55;
    public static double goIntake = 95;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_intake = new DozzyIntake(hardwareMap);
        m_indexer = new IndexerSubsystem(hardwareMap);

        pathAlgorithm = new IMUPathAlgorithm(m_drive);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry();

        register(m_drive);

        schedule(new SequentialCommandGroup(
                //60 delante
                //20 atras
                //giro de unos 20 grados
                //30 delante
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT_IMU, 0.5, crashBall, 0),
                new WaitCommand(750),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT_IMU, 0.5, backUpForNextBall, 0),
                new WaitCommand(750),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, TURN, 0.5, turnToNextBall),
                new WaitCommand(750),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT_IMU, 0.5, goToNextBall, turnToNextBall),
                new WaitCommand(750),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, TURN, 0.5, turnToBall),
                new WaitCommand(750),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT_IMU, 0.5, hitNextBall, turnToBall),
                new WaitCommand(100),
                new InstantCommand(() -> m_intake.setIntakePower(1.0))
                        .alongWith(new InstantCommand(() -> m_indexer.setIndexerPower(-1.0))), //Intake Balls
                new WaitCommand(750),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT_IMU, 0.5, goIntake, turnToBall)
        ));

        schedule(new RunCommand(() -> {
            packet.put("Left inches", DriveConstants.encoderTicksToInches(m_drive.getLeftCurrentPos()));
            packet.put("Right inches", DriveConstants.encoderTicksToInches(m_drive.getRightCurrentPos()));
            packet.put("Robot Angle", m_drive.getAngle());

            dashboard.sendTelemetryPacket(packet);
        }));
    }
}
