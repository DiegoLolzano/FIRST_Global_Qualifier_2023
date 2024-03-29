package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ContinuousServoIntake extends SubsystemBase {
    CRServo intakeServo;

    public ContinuousServoIntake(HardwareMap hardwareMap){
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setServoIntakePower(double power){
        intakeServo.setPower(power);
    }
}
