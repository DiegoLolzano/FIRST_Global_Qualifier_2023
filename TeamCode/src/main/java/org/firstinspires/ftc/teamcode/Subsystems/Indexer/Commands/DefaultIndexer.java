package org.firstinspires.ftc.teamcode.Subsystems.Indexer.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Indexer.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.DozzyIntake;

public class DefaultIndexer extends CommandBase {
    private IndexerSubsystem m_intake;
    private Gamepad gamepad;
    private IntakeMode intakeMode;

    public enum IntakeMode{
        IN_INDEXER,
        OUT_INDEXER,
        STOP_INDEXER
    }

    public DefaultIndexer(IndexerSubsystem m_intake, IntakeMode intakeMode){
        this.m_intake = m_intake;
        this.intakeMode = intakeMode;

        addRequirements(m_intake);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        switch (intakeMode){
            case STOP_INDEXER:
                m_intake.setIndexerPower(0.0);
            break;

            case IN_INDEXER:
                m_intake.setIndexerPower(-1.0);
            break;

            case OUT_INDEXER:
                m_intake.setIndexerPower(1.0);
            break;
        }
    }

    @Override
    public void end(boolean isInterrupted){}

    @Override
    public boolean isFinished(){
        return m_intake.isIndexerBusy() & gamepad == null;
    }
}
