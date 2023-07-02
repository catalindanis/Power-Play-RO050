package org.firstinspires.ftc.teamcode.drive.Systems;

import org.firstinspires.ftc.teamcode.drive.ConstantValues.Powers;

public class SpeedSystem {

    public void next(){
        if(Powers.drivePower == 0.4)
            Powers.drivePower = 1.0;
        else Powers.drivePower = 0.4;
    }

}
