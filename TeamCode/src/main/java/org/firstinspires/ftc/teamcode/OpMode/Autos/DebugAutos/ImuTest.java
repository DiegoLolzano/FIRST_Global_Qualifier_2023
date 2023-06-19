package org.firstinspires.ftc.teamcode.OpMode.Autos.DebugAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.STRAIGHT;

import org.firstinspires.ftc.teamcode.CerbLib.PathAlgorithm;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.ImuAlign;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Autonomous
public class ImuTest extends CommandOpMode {
    DriveTrain m_drive;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    PathAlgorithm pathAlgorithm;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);

        pathAlgorithm = new PathAlgorithm(m_drive);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> m_drive.resetImu()),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 40),
                new ImuAlign(m_drive, 90),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 40),
                new ImuAlign(m_drive, 180),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 40),
                new ImuAlign(m_drive, 270),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 40),
                new ImuAlign(m_drive, 0.0),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 40)
        ));

        schedule(new RunCommand(() -> {
           /* telemetry.addData("Arm Ticks", m_arm.getArmTicks());
            telemetry.addData("Servo Pos", m_wrist.getServoPosition());
            telemetry.addData("Current Selected Level", m_selector.getLevelToPickUp());
            telemetry.update(); */

            packet.put("Left inches", DriveConstants.encoderTicksToInches(m_drive.getLeftCurrentPos()));
            packet.put("Right inches", DriveConstants.encoderTicksToInches(m_drive.getRightCurrentPos()));
            packet.put("Robot Angle", m_drive.getAngle());
            //zpacket.put("Servo Angel", m_wrist.getServoPosition());

            dashboard.sendTelemetryPacket(packet);
        }));

    }
}
