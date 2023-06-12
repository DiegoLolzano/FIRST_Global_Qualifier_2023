package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    DcMotorEx intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap){
       intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

       intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setIntakePower(double power){
        intakeMotor.setPower(power);
    }
}
