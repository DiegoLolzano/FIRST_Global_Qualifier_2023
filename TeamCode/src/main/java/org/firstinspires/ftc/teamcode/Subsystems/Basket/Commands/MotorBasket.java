package org.firstinspires.ftc.teamcode.Subsystems.Basket.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Basket.MotorizedBasketSubsystem;

public class MotorBasket extends CommandBase {
    private MotorizedBasketSubsystem m_basket;
    private BasketPose pose;
    private int basketTicks = 0;

    public enum BasketPose {
        MOTOR_INDEX,
        MOTOR_TRANSIT,
        MOTOR_SCORE
    }

    public MotorBasket(MotorizedBasketSubsystem m_basket, BasketPose pose){
        this.m_basket = m_basket;
        this.pose = pose;

        addRequirements(m_basket);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        switch (pose){
            case MOTOR_INDEX:
                basketTicks = 0;
                m_basket.setBasketTicks(basketTicks);
                m_basket.setPower(0.4);
            break;

            case MOTOR_TRANSIT:
                basketTicks = -51;
                m_basket.setBasketTicks(basketTicks);
                m_basket.setPower(0.4);
            break;

            case MOTOR_SCORE:
                basketTicks = -96;
                m_basket.setBasketTicks(basketTicks);
                m_basket.setPower(0.4);
            break;
        }
    }

    @Override
    public void end(boolean interrupted){
        m_basket.setPower(0.0);
    }

    @Override
    public boolean isFinished(){
        return !m_basket.isBasketBusy() || m_basket.isBasketAtPoint();
    }
}
