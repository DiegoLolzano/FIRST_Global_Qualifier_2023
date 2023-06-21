package org.firstinspires.ftc.teamcode.Subsystems.Testing.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Testing.ControlHUBTester;

public class MotorGamePad extends CommandBase {
    ControlHUBTester m_tester;
    Gamepad gamepad;

    public MotorGamePad(ControlHUBTester m_tester, Gamepad gamepad){
        this.m_tester = m_tester;
        this.gamepad = gamepad;

        addRequirements(m_tester);
    }

    @Override
    public void initialize(){
        m_tester.returnToUsingEncoder();
    }

    @Override
    public void execute(){
        m_tester.setMotorPower(gamepad.left_stick_y);
    }

    @Override
    public void end(boolean interrupted){
        m_tester.setMotorPower(0.0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
