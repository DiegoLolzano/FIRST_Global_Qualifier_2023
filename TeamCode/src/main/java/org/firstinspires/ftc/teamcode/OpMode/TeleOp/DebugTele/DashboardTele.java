package org.firstinspires.ftc.teamcode.OpMode.TeleOp.DebugTele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ChallengeSpecific.June19.AllArmPos;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

public class DashboardTele extends CommandOpMode {
    DriveTrain m_drive;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void initialize() {
        dashboard = FtcDashboard.getInstance();
        m_drive = new DriveTrain(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        schedule(new RunCommand(() -> {
            //telemetry.addData("leftTicks", m_drive.getLeftCurrentPos());
            //telemetry.addData("rightTicks", m_drive.getRightCurrentPos());
            packet.put("leftTicks", m_drive.getLeftCurrentPos());
            packet.put("rightTicks", m_drive.getRightCurrentPos());
            dashboard.sendTelemetryPacket(packet);
        }));

    }
}
