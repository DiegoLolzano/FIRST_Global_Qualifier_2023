package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DozzyIntake extends SubsystemBase {
    DcMotorEx intakeMotor;

    public DozzyIntake(HardwareMap hardwareMap){
       intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

       intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setIntakePower(double power){
        intakeMotor.setPower(power);
    }

    public boolean isIntakeBusy(){
        return intakeMotor.isBusy();
    }
}
