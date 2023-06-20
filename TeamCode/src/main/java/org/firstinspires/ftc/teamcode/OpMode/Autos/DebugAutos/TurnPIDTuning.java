package org.firstinspires.ftc.teamcode.OpMode.Autos.DebugAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand.IMUAlgorithmModes.STRAIGHT_IMU;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand.IMUAlgorithmModes.TURN;

import org.firstinspires.ftc.teamcode.CerbLib.IMUPathAlgorithm;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Autonomous (name="Tuneo de PID Giro", group="PID Tuning")
public class TurnPIDTuning extends CommandOpMode {
    DriveTrain m_drive;
    IMUPathAlgorithm pathAlgorithm;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    //Add public Static variables for live tuning
    public static double driveSpeed = 0.5;
    public static double heading = 90;

    @Override
    public void initialize() {

        m_drive = new DriveTrain(hardwareMap);
        pathAlgorithm = new IMUPathAlgorithm(m_drive);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        register(m_drive);

        schedule(new SequentialCommandGroup(
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, TURN, driveSpeed, heading)
        ));

        schedule(new RunCommand(() -> {
            packet.put("Robot Angle", m_drive.getAngle());
            packet.put("Left inches", DriveConstants.encoderTicksToInches(m_drive.getLeftCurrentPos()));
            packet.put("Right inches", DriveConstants.encoderTicksToInches(m_drive.getRightCurrentPos()));

            dashboard.sendTelemetryPacket(packet);
        }));
    }
}
