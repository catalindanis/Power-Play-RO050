package org.firstinspires.ftc.teamcode.drive.opmode.TeleOP;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Config.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ClawPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ArmRotatorPosition;
import org.firstinspires.ftc.teamcode.drive.Systems.KeyStatus;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.LiftPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.Powers;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ClawRotatorPosition;
import org.firstinspires.ftc.teamcode.drive.Systems.SpeedSystem;
import org.firstinspires.ftc.teamcode.drive.Systems.System;

import java.security.Key;

@TeleOp(name = "TeleOP", group = "MecanumBot")
public class TeleOP extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        robot.init();

        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        System system = new System(robot);
        KeyStatus dpadUp = new KeyStatus();
        KeyStatus dpadDown = new KeyStatus();
        KeyStatus leftStickButton = new KeyStatus();
        KeyStatus leftBumper = new KeyStatus();
        KeyStatus rightBumper = new KeyStatus();
        KeyStatus triangle = new KeyStatus();
        KeyStatus circle = new KeyStatus();
        KeyStatus cross = new KeyStatus();
        KeyStatus triangle2 = new KeyStatus();
        KeyStatus rightBumper1 = new KeyStatus();
        SpeedSystem speedSystem = new SpeedSystem();

        //variabile pentru relative
        double x,y,direction,ipotenuse,rotate,strafe,forward,lR,rF,lF,rR;



        //gamepad1.rumble(1,1,250);
        //gamepad2.rumble(1,1,250);

        //telemetry.speak("READY!");
        telemetry.addData("READY","press start to play!");
        telemetry.update();

        waitForStart();

        robot.opMode = this;

        elapsedTime.reset();

        new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    //ridicare lift manuala
                    if ((gamepad2.right_trigger > 0) && robot.getLiftCurrentTicks() >= LiftPosition.LIFT_UP) {
                        robot.liftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                        if (robot.getLiftCurrentTicks() <= LiftPosition.LIFT_UP + 250)
                            Powers.liftSpeedReductor = 0.25;
                        else Powers.liftSpeedReductor = 1.0;

                        robot.liftLifter.setPower((-gamepad2.right_trigger) * Powers.liftSpeedReductor);
                        robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);

                        new Thread(new Runnable() {
                            @Override
                            public void run() {
//                                if(robot.equals(robot.armRotator.getPosition(),ArmRotatorPosition.DOWN,0.01)) {
//                                    robot.armRotator.setPosition(ArmRotatorPosition.HALF);
//                                }
//                                if(robot.equals(robot.armRotator.getPosition(),ArmRotatorPosition.HALF,0.01)) {
                                    robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
                                //}
                            }
                        }).start();

                        if (robot.claw.getPosition() == ClawPosition.CLOSED)
                            robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
                    } else if ((gamepad2.left_trigger > 0) && robot.getLiftCurrentTicks() <= LiftPosition.LIFT_DOWN) {
                        robot.liftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        if (robot.getLiftCurrentTicks() >= LiftPosition.LIFT_DOWN - 1500)
                            Powers.liftSpeedReductor = 0.05;
                        else if (robot.getLiftCurrentTicks() >= LiftPosition.LIFT_DOWN - 1100)
                            Powers.liftSpeedReductor = 0.01;
                        else Powers.liftSpeedReductor = 1.0;

                        robot.liftLifter.setPower((gamepad2.left_trigger) * Powers.liftSpeedReductor);

                        if (robot.claw.getPosition() == ClawPosition.OPENED)
                            robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
                    } else if (robot.liftLifter.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                        if (gamepad2.right_stick_y != 0)
                            robot.liftLifter.setPower(gamepad2.right_stick_y);
                        else {
//                            if(robot.getLiftCurrentTicks() <= -2300)
//                                robot.liftLifter.setPower(-0.01);
//                            else {
                                robot.liftLifter.setPower(-0.01);
                                //robot.liftLifter.setPower(0);
                                robot.liftLifter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                            //}
                        }
                    }
                }
            }
        }).start();

        new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()){
                    //deschidere || inchidere cleste
                    if (gamepad1.right_bumper)
                        robot.claw.setPosition(ClawPosition.CLOSED);

                    if (gamepad1.left_bumper) {
                        robot.claw.setPosition(ClawPosition.WIDE_OPENED);
                        if(robot.getLiftCurrentTicks() <= -2000) {
                            new Thread(new Runnable() {
                                @Override
                                public void run() {
                                    sleep(250);
                                    robot.liftUp(LiftPosition.LIFT_LOW);
                                    robot.armRotator.setPosition(ArmRotatorPosition.HALF);
                                    sleep(200);
                                    robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
                                }
                            }).start();
                        }
                    }

//                    if(gamepad1.right_trigger > 0.3)
//                        robot.claw.setPosition(ClawPosition.CLOSED);
//                    if(gamepad1.left_trigger > 0.3)
//                        robot.claw.setPosition(ClawPosition.WIDE_OPENED);

                    if(gamepad1.triangle) {
                        if(!triangle2.isPressed) {
                            triangle2.setOn();
                            robot.claw.setPosition(ClawPosition.BEACON);
                            new Thread(new Runnable() {
                                @Override
                                public void run() {
                                    sleep(300);
                                    robot.claw.setPosition(ClawPosition.CLOSED);
                                    robot.armRotator.setPosition(ArmRotatorPosition.HALF);
                                }
                            }).start();
                        }
                    }
                    else triangle2.setOff();
                }
            }
        }).start();

        new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    //sistem armRotator si clawRotator
                    if (gamepad2.dpad_up) {
                        if (!dpadUp.isPressed) {
                            dpadUp.setOn();
                            system.next();
                        }
                    } else dpadUp.setOff();

                    if (gamepad2.dpad_down) {
                        if (!dpadDown.isPressed) {
                            dpadDown.setOn();
                            system.previous();
                        }
                    } else dpadDown.setOff();
                }
            }
        }).start();

        new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    //ridicare automata pentru low
                    if (gamepad2.cross) {
                        if(!cross.isPressed) {
                            cross.setOn();
                            robot.runUsingEncoders(LiftPosition.LIFT_LOW);
                            new Thread(new Runnable() {
                                @Override
                                public void run() {
                                    sleep(100);
                                    robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
//                                    sleep(350);
                                    robot.armRotator.setPosition(ArmRotatorPosition.LOW);
                                }
                            }).start();
                        }
                    }
                    else cross.setOff();
                    //ridicare automata pentru medium
                    if (gamepad2.circle) {
                        if(!circle.isPressed) {
                            circle.setOn();
                            robot.runUsingEncoders(LiftPosition.LIFT_MEDIUM);
                            new Thread(new Runnable() {
                                @Override
                                public void run() {
                                    sleep(350);
                                    robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
                                    //sleep(300);
                                    robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
                                }
                            }).start();
                        }
                    }
                    else circle.setOff();
                    //ridicare automata pentru high
                    if (gamepad2.triangle) {
                        if(!triangle.isPressed) {
                            triangle.setOn();
                            robot.runUsingEncoders(LiftPosition.LIFT_UP);
                            new Thread(new Runnable() {
                                @Override
                                public void run() {
                                    sleep(350);
                                    robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
                                    //sleep(300);
                                    robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
                                }
                            }).start();
                        }
                    }
                    else triangle.setOff();
                }
            }
        }).start();

        new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    //inchidere automata cleste cand vede con
//                    if (robot.distanceSensor.getDistance(DistanceUnit.CM) <= 8.2 &&
//                            robot.distanceSensor.getDistance(DistanceUnit.CM) >= 7 &&
//                            robot.getLiftCurrentTicks() >= -300 &&
//                            robot.equals(robot.armRotator.getPosition(), ArmRotatorPosition.DOWN, 0.01)
//                            && robot.distanceSensorEnabled && robot.equals(robot.claw.getPosition(),ClawPosition.WIDE_OPENED,0.02)) {
//                        robot.claw.setPosition(ClawPosition.CLOSED);
//
//                    }

                    //deschidere automata cleste cand vede batul
                    if (robot.distanceSensor.getDistance(DistanceUnit.CM) <= 7 &&
                            robot.getLiftCurrentTicks() <= -2600 && robot.distanceSensorEnabled &&
                                robot.equals(robot.armRotator.getPosition(),ArmRotatorPosition.HIGH,0.01)) {
                        robot.claw.setPosition(ClawPosition.WIDE_OPENED);
                        robot.liftUp(LiftPosition.LIFT_LOW);
                        new Thread(new Runnable() {
                            @Override
                            public void run() {
                                sleep(250);
                                robot.liftUp(LiftPosition.LIFT_LOW);
                                robot.armRotator.setPosition(ArmRotatorPosition.HALF);
                                sleep(200);
                                robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
                            }
                        }).start();
                    }

                    if (gamepad1.square) {
                        robot.setExternalHeading(Math.toRadians(90));
                    }

                    //schimbare viteza drive
                    if (gamepad2.left_stick_button) {
                        if (!leftStickButton.isPressed) {
                            leftStickButton.setOn();
                            speedSystem.next();
                        }
                    } else leftStickButton.setOff();

                    //activare / dezactivare senzor
                    if (gamepad2.left_bumper && gamepad2.right_bumper) {
                        if (!leftBumper.isPressed && !rightBumper.isPressed) {
                            leftBumper.setOn();
                            rightBumper.setOn();
                            robot.distanceSensorEnabled = robot.distanceSensorEnabled ? false : true;
                            robot.forcedDistanceSensorDisabled = robot.distanceSensorEnabled ? false : true;
                        }
                    } else {
                        leftBumper.setOff();
                        rightBumper.setOff();
                    }

//                    if(robot.equals(elapsedTime.time(),90,1)){
//                        robot.distanceSensorEnabled = false;
//                        robot.forcedDistanceSensorDisabled = true;
//                    }

                    //resetare ticks lift
                    if (gamepad2.square)
                        robot.resetLiftTicks();

//                    if(!robot.forceddistanceSensorDisabled) {
//                        if (robot.equals(robot.armRotator.getPosition(), ArmRotatorPosition.HALF, 0.01)) {
//                            robot.distanceSensorEnabled = false;
//                        }
//                        else robot.distanceSensorEnabled = true;
//                    }
                }
            }
        }).start();

        //mesaje afisate
        new Thread(new Runnable() {
            @Override
            public void run() {
                int remainingMinutes = 0;
                int remainingSeconds = 0;
                while(opModeIsActive()){
                    telemetry.addData("STANGA FATA ",robot.leftFront.getCurrentPosition());
                    telemetry.addData("DREAPTA FATA ",robot.rightFront.getCurrentPosition());
                    telemetry.addData("DREAPTA SPATE ",robot.rightRear.getCurrentPosition());
                    telemetry.addData("STANGA SPATE ",robot.leftRear.getCurrentPosition());
                    telemetry.update();
                    //telemetry.addData("RIGHT BUMPER STATUS ",rightBumperG1.isPressed);
//                    if(elapsedTime.time() <= 121) {
//                        remainingMinutes = ((int) elapsedTime.time() >= 60) ? 0 : 1;
//                        remainingSeconds = 60 - ((int) elapsedTime.time() - ((remainingMinutes == 0) ? 60 : 0));
//                    }
//                    telemetry.addData("TIME LEFT ", remainingMinutes + ":" + ((remainingSeconds < 10) ? "0"+remainingSeconds : remainingSeconds));
//                    telemetry.addData("","");
//                    telemetry.addData("DISTANCE SENSOR ", robot.distanceSensorEnabled ? "ACTIVATED" : "DEACTIVATED");
//                    telemetry.addData("FORCED DISTANCE SENSOR ", robot.forcedDistanceSensorDisabled ? "ACTIVATED" : "DEACTIVATED");
//                    try {
//                        telemetry.addData("DISTANCE ", robot.distanceSensor.getDistance(DistanceUnit.CM));
//                    }catch (Exception exception){
//                        telemetry.addData("DISTANCE ", exception.getMessage());
//                    }
//                    telemetry.addData("DRIVE MOTOR SPEED ", Powers.drivePower);
//                    telemetry.addData("LIFT CURRENT TICKS ", robot.getLiftCurrentTicks());
//                    telemetry.addData("LIFT ERROR ", robot.ticksError);
//                    telemetry.addData("CLAW ", robot.equals(robot.claw.getPosition(),ClawPosition.CLOSED,0.01) ? "CLOSED" : "OPENED");
//                    telemetry.addData("CLAW ROTATOR ", robot.equals(robot.clawRotator.getPosition(),ClawRotatorPosition.FACE_GROUND,0.01) ? "FACE GROUND" : "FACE TOP");
//                    telemetry.addData("ARM ROTATOR ", system.getArmRotatorPosition());
//                    telemetry.update();
                }
            }
        }).start();

        while (opModeIsActive()) {
            Pose2d poseEstimate = robot.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(Math.PI -poseEstimate.getHeading());

            //deplasare drept in fata cand e apasat dpad up
            if(gamepad1.dpad_up)
                robot.setWeightedDrivePower(new Pose2d(
                        -0.5*(Powers.drivePower*Powers.driveSpeedReductor),
                        input.getY()*Powers.drivePower*Powers.driveSpeedReductor,
                        -gamepad1.right_stick_x*Powers.drivePower*Powers.driveSpeedReductor
                ));
                //deplasare drept in spate cand e apasat dpad down
            else if(gamepad1.dpad_down)
                robot.setWeightedDrivePower(new Pose2d(
                        +0.55*(Powers.drivePower*Powers.driveSpeedReductor),
                        input.getY()*Powers.drivePower*Powers.driveSpeedReductor,
                        -gamepad1.right_stick_x*Powers.drivePower*Powers.driveSpeedReductor
                ));
                //deplasarea din joystick
            else {
                robot.setWeightedDrivePower(new Pose2d(
                        input.getX() * Powers.drivePower * Powers.driveSpeedReductor,
                        input.getY() * Powers.drivePower * Powers.driveSpeedReductor,
                        -gamepad1.right_stick_x * Powers.drivePower * Powers.driveSpeedReductor
                ));
            }

            robot.update();

            //brake la motoare
            if(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 /*&& rx == 0*/ && gamepad1.right_stick_x == 0
                    && !gamepad1.dpad_up && !gamepad1.dpad_down){
                robot.stopDriving();
            }

//            double x = -gamepad1.left_stick_x;
//            double y = -gamepad1.left_stick_y;
//
//            double direction = Math.atan2(x, y) - Math.toRadians(robot.getAngle()) + 3*Math.PI/2;
//            double ipotenuse = Math.sqrt(x * x + y * y);
//            double rotate = gamepad1.right_stick_x * 0.50;
//            double strafe = Math.sin(direction) * ipotenuse;
//            double forward = Math.cos(direction) * ipotenuse;
//
//            double lR = -Powers.drivePower*Powers.driveSpeedReductor * robot.SQRT(-strafe - forward - rotate);
//            double rF = Powers.drivePower*Powers.driveSpeedReductor * robot.SQRT(strafe + forward - rotate);
//            double lF = -Powers.drivePower*Powers.driveSpeedReductor * robot.SQRT(strafe - forward - rotate);
//            double rR = Powers.drivePower*Powers.driveSpeedReductor * robot.SQRT(-strafe + forward - rotate);
//
//            //reductie drive Power cand bratul e jos si nu are con
            if(robot.equals(robot.armRotator.getPosition(), ArmRotatorPosition.DOWN, 0.01) /*&&
                    (robot.distanceSensor.getDistance(DistanceUnit.CM) > 8.5)*/)
                Powers.driveSpeedReductor = 0.5;
            else Powers.driveSpeedReductor = 1.0;

        }

        telemetry.speak("STOPPED!");

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