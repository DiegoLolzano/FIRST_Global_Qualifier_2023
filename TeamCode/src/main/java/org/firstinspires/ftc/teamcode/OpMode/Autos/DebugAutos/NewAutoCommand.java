package org.firstinspires.ftc.teamcode.OpMode.Autos.DebugAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CerbLib.IMUPathAlgorithm;
import org.firstinspires.ftc.teamcode.CerbLib.PathAlgorithm;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand.IMUAlgorithmModes.TURN;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand.IMUAlgorithmModes.STRAIGHT_IMU;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.STRAIGHT;

@Autonomous
public class NewAutoCommand extends CommandOpMode {
    DriveTrain m_drive;

    IMUPathAlgorithm pathAlgorithm;

    PathAlgorithm normalPathAlgorithm;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);

        pathAlgorithm = new IMUPathAlgorithm(m_drive);

        normalPathAlgorithm = new PathAlgorithm(m_drive);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        register(m_drive);

        schedule(new SequentialCommandGroup(
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT_IMU, 0.5, -50, 0),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, TURN, 0.5, 90),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT_IMU, 0.2, -20, 90)
                //new PathAlgorithmCommand(m_drive, normalPathAlgorithm, STRAIGHT, -50)
                //new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 0.5, -50, 0)
                //new InstantCommand(() -> m_drive.resetImu()),
                //new WaitCommand(1500),
                //new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 0.3,
                //10, 0.0),
                //new WaitCommand(1500),
                //new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, TURN, 0.5, 90)
                //new WaitCommand(1500),
                //new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 0.8,
                        //10, 90.0)
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
