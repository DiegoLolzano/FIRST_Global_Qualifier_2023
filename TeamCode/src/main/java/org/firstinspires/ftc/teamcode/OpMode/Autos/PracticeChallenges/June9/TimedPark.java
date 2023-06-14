package org.firstinspires.ftc.teamcode.OpMode.Autos.PracticeChallenges.June9;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.MotorizedArmTicks.ArmModes.EXTENDED;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmCommands.MotorizedArmTicks;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MotorizedArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands.TimedAuto;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.DriveTrain;

@Autonomous (name="Autonomo por Tiempos", group="Reto Junio 9")
public class TimedPark extends CommandOpMode {
    DriveTrain m_drive;
    MotorizedArmSubsystem m_arm;

    @Override
    public void initialize() {
        m_drive = new DriveTrain(hardwareMap);
        m_arm = new MotorizedArmSubsystem(hardwareMap);

        register(m_drive, m_arm);
        schedule(new SequentialCommandGroup(new TimedAuto(m_drive),
                new MotorizedArmTicks(m_arm, EXTENDED)));
    }
}
