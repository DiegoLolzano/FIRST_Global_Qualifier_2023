package org.firstinspires.ftc.teamcode.Subsystems.Arm.MultiArticulated.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.MultiArticulated.MultiArm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.MultiArticulated.MultiElbow;

public class MultiPositionControl extends CommandBase {
    private MultiArm multiArm;
    private MultiElbow multiElbow;
    private MultiState state;
    private int armTicks = 0;
    private int elbowTicks = 0;

    public enum MultiState{
        RETRACTED,
        LOWER,
        MIDDLE,     //Referring to 2023 PowerPlay Junctions
        UPPER
    }

    public MultiPositionControl(MultiArm multiArm, MultiElbow multiElbow, MultiState state){
        this.multiArm = multiArm;
        this.multiElbow = multiElbow;
        this.state = state;

        addRequirements(multiArm, multiElbow);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        switch (state){
            case RETRACTED:
                 armTicks = 0;
                 elbowTicks = 0;

                 multiArm.setArmTicks(armTicks);
                 multiElbow.setArmTicks(elbowTicks);

                 multiArm.setPower(0.5);
                 multiElbow.setPower(0.5);
            break;

            case LOWER:
                armTicks = 100; //Check Values
                elbowTicks = 50;//Check Values

                multiArm.setArmTicks(armTicks);
                multiElbow.setArmTicks(elbowTicks);

                multiArm.setPower(0.5);
                multiElbow.setPower(0.5);
            break;

            case MIDDLE:
                armTicks = 200; //Check Values
                elbowTicks = 150;//Check Values

                multiArm.setArmTicks(armTicks);
                multiElbow.setArmTicks(elbowTicks);

                multiArm.setPower(0.5);
                multiElbow.setPower(0.5);
            break;

            case UPPER:
                armTicks = 400; //Check Values
                elbowTicks = 300;//Check Values

                multiArm.setArmTicks(armTicks);
                multiElbow.setArmTicks(elbowTicks);

                multiArm.setPower(0.5);
                multiElbow.setPower(0.5);
            break;
        }
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
