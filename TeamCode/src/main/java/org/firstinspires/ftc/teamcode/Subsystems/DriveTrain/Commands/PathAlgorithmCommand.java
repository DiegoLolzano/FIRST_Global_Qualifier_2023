package org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.CerbLib.GamepadState;
import org.firstinspires.ftc.teamcode.CerbLib.PathAlgorithm;
import org.firstinspires.ftc.teamcode.CerbLib.PathAlgorithm.DriveSides;
import org.firstinspires.ftc.teamcode.CerbLib.TankDrive;

import java.util.concurrent.TimeUnit;

public class PathAlgorithmCommand extends CommandBase {
    private PathAlgorithm pathAlgorithm;
    private AlgorithmModes algorithmModes;
    private double unit;
    private DriveSides driveSides;
    protected Timer timer;
    private GamepadState activateTimerJustOnce;

    public enum AlgorithmModes{
        STRAIGHT,
        CENTRAL_TURN,
        SIDE_TURN,
        POWAAAAAH,
        OFF
    }

    public PathAlgorithmCommand(TankDrive m_drive, PathAlgorithm pathAlgorithm,
                                AlgorithmModes algorithmModes, double unit){
        this.pathAlgorithm = pathAlgorithm;
        this.algorithmModes = algorithmModes;
        this.unit = unit;

        timer = new Timer(100, TimeUnit.MILLISECONDS);
        activateTimerJustOnce = new GamepadState();

        addRequirements(m_drive);
    }
    public  PathAlgorithmCommand(TankDrive m_drive, PathAlgorithm pathAlgorithm,
                                 AlgorithmModes algorithmMode, double unit, DriveSides driveSides){
        this(m_drive, pathAlgorithm, algorithmMode, unit);
        this.driveSides = driveSides;
    }

    @Override
    public void initialize(){
        pathAlgorithm.UpdateLastPose();
    }

    @Override
    public void execute(){
        switch (algorithmModes){
            case STRAIGHT:
                pathAlgorithm.straight(unit);
            break;

            case POWAAAAAH:
                pathAlgorithm.powaaah(unit);
            break;

            case CENTRAL_TURN:
                pathAlgorithm.centralTurn(unit);
            break;

            case SIDE_TURN:
                pathAlgorithm.sideTurn(unit, driveSides);
            break;

            case OFF:
                pathAlgorithm.getTankDrive().stopMotors();
            break;
        }
        if((!pathAlgorithm.getTankDrive().isLeftBusy() &&
                !pathAlgorithm.getTankDrive().isRightBusy())){
            activateTimerJustOnce.justOncePress(true, () -> timer.start());
        }
    }

    @Override
    public void end(boolean interrupted){
        timer.pause();
    }

    @Override
    public boolean isFinished(){
        return Thread.currentThread().isInterrupted() || timer.done();
    }
}
