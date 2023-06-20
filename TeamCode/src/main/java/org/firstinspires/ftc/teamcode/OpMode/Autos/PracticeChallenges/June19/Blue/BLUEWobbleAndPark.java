package org.firstinspires.ftc.teamcode.OpMode.Autos.PracticeChallenges.June19.Blue;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand.IMUAlgorithmModes.STRAIGHT_IMU;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand.IMUAlgorithmModes.TURN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CerbLib.IMUPathAlgorithm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ServoedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.SingleServoClaw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Autonomous
public class BLUEWobbleAndPark extends CommandOpMode {
    DriveTrain m_drive;
    MotorizedArmSubsystem m_motorizedArm;
    ServoedArmSubsystem m_servoedArm;
    ClawSubsystem m_doubleClaw;
    SingleServoClaw m_singleClaw;
    IMUPathAlgorithm pathAlgorithm;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_motorizedArm = new MotorizedArmSubsystem(hardwareMap);
        m_servoedArm = new ServoedArmSubsystem(hardwareMap);
        m_doubleClaw = new ClawSubsystem(hardwareMap);
        m_singleClaw = new SingleServoClaw(hardwareMap);

        pathAlgorithm = new IMUPathAlgorithm(m_drive);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        register(m_drive, m_motorizedArm, m_servoedArm, m_doubleClaw, m_singleClaw);

        schedule(new SequentialCommandGroup(
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT_IMU,
                        0.8, 70, 0),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, TURN,
                        0.15, -90),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT_IMU,
                        0.8, -30, 90)
        ));
    }
}
