package org.firstinspires.ftc.teamcode.OpMode.Autos.PracticeChallenges.June19.Red;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand.IMUAlgorithmModes.STRAIGHT_IMU;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand.IMUAlgorithmModes.TURN;

import org.firstinspires.ftc.teamcode.CerbLib.IMUPathAlgorithm;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ServoedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.DoubleServoClaw;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.SingleServoClaw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.IMUPathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Autonomous
public class REDWobbleAndPark extends CommandOpMode {
    DriveTrain m_drive;
    MotorizedArmSubsystem m_motorizedArm;
    ServoedArmSubsystem m_servoedArm;
    DoubleServoClaw m_doubleClaw;
    SingleServoClaw m_singleClaw;
    IMUPathAlgorithm pathAlgorithm;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_motorizedArm = new MotorizedArmSubsystem(hardwareMap);
        m_servoedArm = new ServoedArmSubsystem(hardwareMap);
        m_doubleClaw = new DoubleServoClaw(hardwareMap);
        m_singleClaw = new SingleServoClaw(hardwareMap);

        pathAlgorithm = new IMUPathAlgorithm(m_drive);

        register(m_drive, m_motorizedArm, m_servoedArm, m_doubleClaw, m_singleClaw);

        //33
        //66
        //50
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> m_servoedArm.setServoArmPos(0.70)),
                new InstantCommand(() -> m_singleClaw.setSingleServoPos(0.9)),
                new WaitCommand(1500),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT_IMU,
                        0.8, -15, 0),
                new WaitCommand(1000),
                new InstantCommand(() -> m_singleClaw.setSingleServoPos(0.5)),
                new InstantCommand(() -> m_servoedArm.setServoArmPos(0.33)),
                new WaitCommand(2500),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT_IMU,
                        0.8, -40, 0),
                new WaitCommand(2500),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, TURN,
                        0.5, 120),
                new WaitCommand(2500),
                new IMUPathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT_IMU,
                        0.8, -15, 90)
        ));

        schedule(new RunCommand(() -> {
            telemetry.addData("left Inches", DriveConstants.encoderTicksToInches(m_drive.getLeftCurrentPos()));
            telemetry.addData("right Inches", DriveConstants.encoderTicksToInches(m_drive.getRightCurrentPos()));
            telemetry.addData("Get Angle", m_drive.getAngle());
        }));
    }
}
