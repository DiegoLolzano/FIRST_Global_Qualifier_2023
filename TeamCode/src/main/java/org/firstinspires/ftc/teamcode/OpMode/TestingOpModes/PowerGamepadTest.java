package org.firstinspires.ftc.teamcode.OpMode.TestingOpModes;

import static org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands.FULLTEST.TestBeds.MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands.FULLTEST.TestBeds.MOTOR_STOP;
import static org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands.FULLTEST.TestBeds.MOTOR_TO_POS;
import static org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands.FULLTEST.TestBeds.SERVO_POS0;
import static org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands.FULLTEST.TestBeds.SERVO_POS1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands.FULLTEST;
import org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands.MotorGamePad;
import org.firstinspires.ftc.teamcode.Subsystems.Testing.ControlHUBTester;

@TeleOp
public class PowerGamepadTest extends CommandOpMode {
    ControlHUBTester m_tester;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void initialize() {
        m_tester = new ControlHUBTester(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry();

        m_tester.setDefaultCommand(new MotorGamePad(m_tester, gamepad1));

        schedule(new RunCommand(() -> {
            packet.put("Motor1 Pos", m_tester.getMotor1CurrentPose());
            packet.put("Motor1 Velo", m_tester.getMotor1CurrentVelo());

            packet.put("Motor2 Pos", m_tester.getMotor2CurrentPose());
            packet.put("Motor2 Velo", m_tester.getMotor2CurrentVelo());

            packet.put("Motor3 Pos", m_tester.getMotor3CurrentPose());
            packet.put("Motor3 Velo", m_tester.getMotor3CurrentVelo());

            packet.put("Motor4 Pos", m_tester.getMotor4CurrentPose());
            packet.put("Motor4 Velo", m_tester.getMotor4CurrentVelo());

            dashboard.sendTelemetryPacket(packet);
        }));

    }
}
