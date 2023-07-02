package org.firstinspires.ftc.teamcode.drive.opmode.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.teamcode.drive.Config.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.LiftPosition;
import org.firstinspires.ftc.teamcode.drive.Systems.KeyStatus;

import java.security.Key;
import java.util.ArrayList;

@TeleOp(name = "Config", group = "MecanumBot")
public class Config extends LinearOpMode {

    ArrayList<String> menu = new ArrayList<>();
    int currentSelectedIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        KeyStatus dpad_up = new KeyStatus();
        KeyStatus dpad_down = new KeyStatus();
        KeyStatus cross = new KeyStatus();
        KeyStatus dpad_right = new KeyStatus();
        KeyStatus dpad_left = new KeyStatus();

        boolean inMenu = true;
        String selectedOption = "";
        double currentValue = 0;

        initMenu();

        gamepad1.rumble(1,1,250);
        gamepad2.rumble(1,1,250);

        telemetry.speak("READY!");
        telemetry.addData("READY","press start to play!");
        telemetry.update();

        waitForStart();

        robot.opMode = this;

        while (opModeIsActive()) {
            if(inMenu) {
                for (int i = 0; i < menu.size(); i++)
                    telemetry.addData(i == currentSelectedIndex ? "->" : "", menu.get(i));

                if (gamepad1.dpad_down && currentSelectedIndex < menu.size() - 1) {
                    if (!dpad_down.isPressed) {
                        dpad_down.setOn();
                        currentSelectedIndex++;
                    }
                } else dpad_down.setOff();

                if (gamepad1.dpad_up && currentSelectedIndex > 0) {
                    if (!dpad_up.isPressed) {
                        dpad_up.setOn();
                        currentSelectedIndex--;
                    }
                } else dpad_up.setOff();

                if (gamepad1.cross) {
                    if (!cross.isPressed) {
                        cross.setOn();
                        switch (menu.get(currentSelectedIndex)){
                            case "LIFT LOW":
                                currentValue = LiftPosition.LIFT_LOW;
                                break;
                            case "LIFT MEDIUM":
                                currentValue = LiftPosition.LIFT_MEDIUM;
                                break;
                            case "LIFT UP":
                                currentValue = LiftPosition.LIFT_UP;
                                break;
                            case "LIFT DOWN":
                                currentValue = LiftPosition.LIFT_DOWN;
                                break;
                            case "LIFT UP AUTO":
                                currentValue = LiftPosition.LIFT_UP_AUTO;
                                break;
                            case "ARM ROTATOR":
                                break;
                            case "CLAW":
                                break;
                            case "CLAW ROTATOR":
                                break;
                        }
                        inMenu = false;
                    }
                } else cross.setOff();
            }
            else{
                if(menu.get(currentSelectedIndex).contains("LIFT")){
                    telemetry.addData(menu.get(currentSelectedIndex),"");
                    telemetry.addData("TICKS ", currentValue);
                    telemetry.addData("","");
                    telemetry.addData("","press TRIANGLE to SAVE!");
                    telemetry.addData("","press CIRCLE to CANCEL!");

                    if(gamepad1.dpad_left){
                        if(!dpad_left.isPressed){
                            dpad_left.setOn();
                            currentValue -= 10;
                        }
                    }
                    else dpad_left.setOff();

                    if(gamepad1.dpad_right){
                        if(!dpad_right.isPressed){
                            dpad_right.setOn();
                            currentValue += 10;
                        }
                    }
                    else dpad_right.setOff();
                }
                else{

                }

                if(gamepad1.circle){
                    inMenu = true;
                }
            }

            telemetry.update();
        }

        telemetry.speak("STOPPED!");
    }

    public void initMenu(){
        menu.add("LIFT LOW");
        menu.add("LIFT MEDIUM");
        menu.add("LIFT UP");
        menu.add("LIFT DOWN");
        menu.add("LIFT UP AUTO");
        menu.add("ARM ROTATOR ");
        menu.add("CLAW");
        menu.add("CLAW ROTATOR");
    }

}

//⠀⠀⠀⠀⠀⠀⠀⣠⣤⣤⣤⣤⣤⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⠀⠀⠀⢰⡿⠋⠁⠀⠀⠈⠉⠙⠻⣷⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⠀⠀⢀⣿⠇⠀⢀⣴⣶⡾⠿⠿⠿⢿⣿⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⣀⣀⣸⡿⠀⠀⢸⣿⣇⠀⠀⠀⠀⠀⠀⠙⣷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⣾⡟⠛⣿⡇⠀⠀⢸⣿⣿⣷⣤⣤⣤⣤⣶⣶⣿⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⢀⣿⠀⢀⣿⡇⠀⠀⠀⠻⢿⣿⣿⣿⣿⣿⠿⣿⡏⠀⠀⠀⠀⢴⣶⣶⣿⣿⣿⣆ <- aici iti doresc sa stai <3
//⢸⣿⠀⢸⣿⡇⠀⠀⠀⠀⠀⠈⠉⠁⠀⠀⠀⣿⡇⣀⣠⣴⣾⣮⣝⠿⠿⠿⣻⡟
//⢸⣿⠀⠘⣿⡇⠀⠀⠀⠀⠀⠀⠀⣠⣶⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠁⠉⠀
//⠸⣿⠀⠀⣿⡇⠀⠀⠀⠀⠀⣠⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠟⠉⠀⠀⠀⠀
//⠀⠻⣷⣶⣿⣇⠀⠀⠀⢠⣼⣿⣿⣿⣿⣿⣿⣿⣛⣛⣻⠉⠁⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⠀⠀⢸⣿⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⠀⠀⢸⣿⣀⣀⣀⣼⡿⢿⣿⣿⣿⣿⣿⡿⣿⣿⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⠀⠀⠀⠙⠛⠛⠛⠋⠁⠀⠙⠻⠿⠟⠋⠑⠛⠋⠀
