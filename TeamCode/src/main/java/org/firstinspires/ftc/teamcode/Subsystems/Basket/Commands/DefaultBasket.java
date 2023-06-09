package org.firstinspires.ftc.teamcode.Subsystems.Basket.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Basket.BasketSubsystem;

import com.arcrobotics.ftclib.util.Timing.Timer;

import java.util.concurrent.TimeUnit;

public class DefaultBasket extends CommandBase {
    BasketSubsystem basketSubsystem;
    private BasketModes modes;
    private Gamepad gamepad;
    protected Timer timer;

    public enum BasketModes{
        INDEX,
        TRANSIT,
        SCORE

    }

    public DefaultBasket(BasketSubsystem basketSubsystem, BasketModes modes){
        this.basketSubsystem = basketSubsystem;
        this.modes = modes;

        timer = new Timer(700, TimeUnit.MILLISECONDS);

        addRequirements(basketSubsystem);
    }

    @Override
    public void initialize(){
        timer.start();
    }

    @Override
    public void execute(){
        switch (modes){
            case INDEX:
                basketSubsystem.setBasketPos(0);
            break;

            case TRANSIT:
                basketSubsystem.setBasketPos(0.5);
            break;

            case SCORE:
                basketSubsystem.setBasketPos(1.0);
        }
    }

    @Override
    public void end(boolean interrupted){
        timer.pause();
    }

    @Override
    public boolean isFinished(){
        return timer.done() & gamepad == null;
    }
}
