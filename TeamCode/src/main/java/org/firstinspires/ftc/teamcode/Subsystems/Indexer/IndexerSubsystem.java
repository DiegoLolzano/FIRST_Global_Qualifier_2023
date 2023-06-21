package org.firstinspires.ftc.teamcode.Subsystems.Indexer;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IndexerSubsystem extends SubsystemBase {
    DcMotorEx indexerMotor;

    public IndexerSubsystem(HardwareMap hardwareMap){
       indexerMotor = hardwareMap.get(DcMotorEx.class, "indexerMotor");

       indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setIndexerPower(double power){
        indexerMotor.setPower(power);
    }

    public boolean isIndexerBusy(){
        return indexerMotor.isBusy();
    }
}
