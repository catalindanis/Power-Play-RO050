package org.firstinspires.ftc.teamcode.drive.Systems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class KeyStatus {
    public boolean isPressed = false;
    //public ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public void setOn() {
        isPressed = true;
        //elapsedTime.reset();
    }

    public void setOff() {
        isPressed = false;
    }
}
