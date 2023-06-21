package org.firstinspires.ftc.teamcode.OpMode.TestingOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands.FULLTEST.TestBeds.MOTOR_STOP;
import static org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands.FULLTEST.TestBeds.MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands.FULLTEST.TestBeds.MOTOR_TO_POS;
import static org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands.FULLTEST.TestBeds.SERVO_POS0;
import static org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands.FULLTEST.TestBeds.SERVO_POS1;

import org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands.FULLTEST;
import org.firstinspires.ftc.teamcode.Subsystems.Testing.ControlHUBTester;

@TeleOp
public class ControlHubTest extends CommandOpMode {
    ControlHUBTester m_tester;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void initialize() {
        m_tester = new ControlHUBTester(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry();

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new FULLTEST(m_tester, MOTOR_STOP));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.A).whenPressed(new FULLTEST(m_tester, MOTOR_POWER));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.B).whenPressed(new FULLTEST(m_tester, MOTOR_TO_POS));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.X).whenPressed(new FULLTEST(m_tester, SERVO_POS0));

        new GamepadButton(new GamepadEx(gamepad1),
                GamepadKeys.Button.Y).whenPressed(new FULLTEST(m_tester, SERVO_POS1));

        schedule(new RunCommand(() -> {
            packet.put("Motor1 Pos", m_tester.getMotor1CurrentPose());
            packet.put("Motor1 Velo", m_tester.getMotor1CurrentVelo());

            packet.put("Motor2 Pos", m_tester.getMotor2CurrentPose());
            packet.put("Motor2 Velo", m_tester.getMotor2CurrentVelo());

            packet.put("Motor3 Pos", m_tester.getMotor3CurrentPose());
            packet.put("Motor3 Velo", m_tester.getMotor3CurrentVelo());

            packet.put("Motor4 Pos", m_tester.getMotor4CurrentPose());
            packet.put("Motor4 Velo", m_tester.getMotor4CurrentVelo());

            packet.put("Get Servo1 Pos", m_tester.getServoPose1());
            packet.put("Get Servo2 Pos", m_tester.getServoPose2());
            packet.put("Get Servo3 Pos", m_tester.getServoPose3());
            packet.put("Get Servo4 Pos", m_tester.getServoPose4());
            packet.put("Get Servo5 Pos", m_tester.getServoPose5());
            packet.put("Get Servo6 Pos", m_tester.getServoPose6());

            packet.put("IMU Angle", m_tester.getAngle());

            dashboard.sendTelemetryPacket(packet);
        }));

    }
}
