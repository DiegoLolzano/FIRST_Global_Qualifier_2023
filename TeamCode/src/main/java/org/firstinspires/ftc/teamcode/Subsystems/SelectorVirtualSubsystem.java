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

        coneLevel.add("Top");
        coneLevel.add("Level 4");
        coneLevel.add("Level 3");
        coneLevel.add("Level 2");
        coneLevel.add("Bottom");
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

    public String getLevelToPickUp(){
        return coneLevel.get(selectedLevel);
    }
}
