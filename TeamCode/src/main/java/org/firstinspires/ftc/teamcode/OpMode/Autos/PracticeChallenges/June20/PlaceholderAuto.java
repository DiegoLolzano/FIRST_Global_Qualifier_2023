package org.firstinspires.ftc.teamcode.OpMode.Autos.PracticeChallenges.June20;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CerbLib.IMUPathAlgorithm;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Autonomous
public class PlaceholderAuto extends CommandOpMode {
    DriveTrain m_drive;

    IMUPathAlgorithm pathAlgorithm;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);

        pathAlgorithm = new IMUPathAlgorithm(m_drive);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry();

        register(m_drive);

        schedule(new SequentialCommandGroup(

        ));

        schedule(new RunCommand(() -> {
            packet.put("Left inches", DriveConstants.encoderTicksToInches(m_drive.getLeftCurrentPos()));
            packet.put("Right inches", DriveConstants.encoderTicksToInches(m_drive.getRightCurrentPos()));
            packet.put("Robot Angle", m_drive.getAngle());

            dashboard.sendTelemetryPacket(packet);
        }));
    }
}
