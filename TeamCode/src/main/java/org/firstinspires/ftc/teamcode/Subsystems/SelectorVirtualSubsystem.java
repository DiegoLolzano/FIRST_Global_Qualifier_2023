package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;

public class SelectorVirtualSubsystem extends SubsystemBase {
    private int selectedLevel = 0;
    private ArrayList<String> coneLevel = new ArrayList<String>();
    private Gamepad gamepad;

    public SelectorVirtualSubsystem(Gamepad gamepad){
        this.gamepad = gamepad;

        coneLevel.add("TOP");
        coneLevel.add("MIDDLE");
        coneLevel.add("LOWER");
    }

    public void updateSelectionUp(){
        selectedLevel--;

        if(selectedLevel < 0){
            selectedLevel = coneLevel.size() - 1;
        }
    }

    public void updateSelectionDown(){
        selectedLevel++;
        if(selectedLevel >= coneLevel.size()){
            selectedLevel = 0;
        }
    }

    public String getLevelToScore(){
        return coneLevel.get(selectedLevel);
    }
}
