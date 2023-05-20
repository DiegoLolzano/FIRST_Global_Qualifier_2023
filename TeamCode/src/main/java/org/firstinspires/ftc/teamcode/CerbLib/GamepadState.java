package org.firstinspires.ftc.teamcode.CerbLib;
import androidx.annotation.NonNull;

public class GamepadState {
    private boolean justOnce = false;
    public GamepadState(){}

    public void justOncePress(boolean button, @NonNull Runnable method2Run){
        if(button && !justOnce){
            method2Run.run();
            justOnce = true;
        }
        if(!button)justOnce = false;
    }
}
