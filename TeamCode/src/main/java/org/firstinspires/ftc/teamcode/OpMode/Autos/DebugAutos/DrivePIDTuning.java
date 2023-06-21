package org.firstinspires.ftc.teamcode.OpMode.Autos.DebugAutos;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand.IMUAlgorithmModes.STRAIGHT_IMU;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand.IMUAlgorithmModes.TURN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CerbLib.IMUPathAlgorithm;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Config
@Autonomous (name="Tuneo de PID Delante", group="PID Tuning")
public class DrivePIDTuning extends CommandOpMode {
    DriveTrain m_drive;
    IMUPathAlgorithm pathAlgorithm;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    //Add public Static variables for live tuning
    public static double driveSpeed = 0.5;
    public static double distance = 50;
    public static double heading = 0;

    @Override
    public void initialize() {

        m_drive = new DriveTrain(hardwareMap);
        pathAlgorithm = new IMUPathAlgorithm(m_drive);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        register(m_drive);

        schedule(new SequentialCommandGroup(
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT_IMU,
                        driveSpeed, distance, heading)
        ));

        schedule(new RunCommand(() -> {
            packet.put("Robot Angle", m_drive.getAngle());
            packet.put("Left inches", DriveConstants.encoderTicksToInches(m_drive.getLeftCurrentPos()));
            packet.put("Right inches", DriveConstants.encoderTicksToInches(m_drive.getRightCurrentPos()));

            dashboard.sendTelemetryPacket(packet);
        }));
    }
}