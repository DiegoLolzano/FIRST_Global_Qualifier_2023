package org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.CerbLib.GamepadState;
import org.firstinspires.ftc.teamcode.CerbLib.IMUPathAlgorithm;
import org.firstinspires.ftc.teamcode.CerbLib.TankDrive;

import java.util.concurrent.TimeUnit;

public class IMUPathAlgorithmCommand extends CommandBase {
    private IMUPathAlgorithm pathAlgorithm;
    private IMUAlgorithmModes modes;

    protected Timer timer;
    private GamepadState activateTimerJustOnce;

    private double maxDriveSpeed;
    private double maxTurnSpeed;
    private double distance;
    private double heading;

    public enum IMUAlgorithmModes{
        STRAIGHT_IMU,
        TURN
    }

    public IMUPathAlgorithmCommand(TankDrive tankDrive, IMUPathAlgorithm pathAlgorithm,
                                   IMUAlgorithmModes modes, double maxDriveSpeed,
                                   double distance, double heading){
        this.pathAlgorithm = pathAlgorithm;
        this.modes = modes;

        this.maxDriveSpeed = maxDriveSpeed;
        this.distance = distance;
        this.heading = heading;

        timer = new Timer(100, TimeUnit.MILLISECONDS);
        activateTimerJustOnce = new GamepadState();

        addRequirements(tankDrive);
    }

    public IMUPathAlgorithmCommand(TankDrive tankDrive, IMUPathAlgorithm pathAlgorithm,
                                   IMUAlgorithmModes modes, double maxTurnSpeed, double heading){
        this.pathAlgorithm = pathAlgorithm;
        this.modes = modes;

        this.maxTurnSpeed = maxTurnSpeed;
        this.heading = heading;

        timer = new Timer(100, TimeUnit.MILLISECONDS);
        activateTimerJustOnce = new GamepadState();

        addRequirements(tankDrive);
    }

    @Override
    public void initialize(){
        pathAlgorithm.UpdateLastPose();
    }

    @Override
    public void execute(){
        switch (modes){
            case STRAIGHT_IMU:
                pathAlgorithm.driveStraight(maxDriveSpeed, distance, heading);
            break;

            case TURN:
                pathAlgorithm.turnToHeading(maxTurnSpeed, heading);
        }

        if((!pathAlgorithm.getTankDrive().isLeftBusy() &&
                !pathAlgorithm.getTankDrive().isRightBusy())){
            activateTimerJustOnce.justOncePress(true, () -> timer.start());
        }
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return Thread.currentThread().isInterrupted() || timer.done();
    }
}
