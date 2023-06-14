package org.firstinspires.ftc.teamcode.OpMode.Autos.PracticeChallenges.June13;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.OFF;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.STRAIGHT;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.SIDE_TURN;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand.AlgorithmModes.CENTRAL_TURN;

import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.OPEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw.ClawModes.CLOSED;

import org.firstinspires.ftc.teamcode.CerbLib.PathAlgorithm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Claw.Commands.DefaultClaw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.PathAlgorithmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Autonomous (name="Autonomo Seguro", group="Reto Junio 13")
public class SecureAuto extends CommandOpMode {
    DriveTrain m_drive;
    ClawSubsystem m_claw;

    PathAlgorithm pathAlgorithm;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_claw = new ClawSubsystem(hardwareMap);

        pathAlgorithm = new PathAlgorithm(m_drive);

        /*schedule(new SequentialCommandGroup(
                new DefaultClaw(m_claw, CLOSED),
                new WaitCommand(200),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, 110),
                new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 45)
        ));*/

        //BACKUP AUTO CODE
         schedule(new SequentialCommandGroup(
                 new InstantCommand(() -> m_claw.setBothClawPos(0.1)),
                 new WaitCommand(2500),
                 new PathAlgorithmCommand(m_drive, pathAlgorithm, STRAIGHT, -150),
                 new PathAlgorithmCommand(m_drive, pathAlgorithm, CENTRAL_TURN, 45)
        ));

    }
}
