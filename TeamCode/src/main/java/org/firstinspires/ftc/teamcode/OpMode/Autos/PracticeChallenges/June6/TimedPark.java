package org.firstinspires.ftc.teamcode.OpMode.Autos.PracticeChallenges.June6;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmTicks.ArmModes.EXTENDED;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.ArmTicks;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotoredArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.TimedAuto;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Autonomous
public class TimedPark extends CommandOpMode {
    DriveTrain m_drive;
    MotoredArmSubsystem m_arm;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_arm = new MotoredArmSubsystem(hardwareMap);

        register(m_drive, m_arm);
        schedule(new SequentialCommandGroup(new TimedAuto(m_drive),
                new ArmTicks(m_arm, EXTENDED)));
    }
}
