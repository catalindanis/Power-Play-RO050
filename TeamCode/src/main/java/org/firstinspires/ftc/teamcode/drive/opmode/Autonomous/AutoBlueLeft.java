package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.drive.OpenCVCameraDetection.DetectarePozitie.ParkingPosition.CENTER;

import android.icu.text.Transliterator;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.sun.tools.javac.tree.JCTree;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.ComputedValues.StickAngles;
import org.firstinspires.ftc.teamcode.drive.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Config.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ArmRotatorPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ClawPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ClawRotatorPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.LiftPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.Positions;
import org.firstinspires.ftc.teamcode.drive.OpenCVCameraDetection.DetectarePozitie;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.net.SocketException;

@Autonomous(name = "AutoBlueLeft", group = "Auto")
public class AutoBlueLeft extends LinearOpMode {

    FtcDashboard dashboard;
    private DetectarePozitie sleeveDetection;
    private OpenCvCamera camera;
    private StickAngles angle = null;

    private String webcamName = "Webcam 1";

    private SampleMecanumDrive robot;

    String ParkCase;

    int corectieUp,corectieDown;

    TrajectorySequence GoesForPreCone;
    TrajectorySequence PlacesPreCone;
    TrajectorySequence AlignForCone;
    TrajectorySequence GoesForCone;
    TrajectorySequence AlignForPlace;
    TrajectorySequence PlacesCone;

    double takeX = 0;
    double takeY = 0;
    double placeX = 0;
    double placeY = 0;
    double waitTime = 0;
    double secondWaitTime = 0;
    double angleCorrection = 0;

    Pose2d currentPosition = new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new DetectarePozitie();
        camera.setPipeline(sleeveDetection);

        dashboard = FtcDashboard.getInstance();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                dashboard.startCameraStream(camera, 120);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        robot = new SampleMecanumDrive(hardwareMap);

        angle = new StickAngles();

        robot.setPoseEstimate(new Pose2d(35.18152727949673, 62.5678946778844));
        robot.setExternalHeading(Math.toRadians(0));

        try {

            robot.claw.setPosition(ClawPosition.CLOSED);
            sleep(1000);
            robot.initAutonomous();

            while (!isStarted()) {
                telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
                telemetry.update();
                ParkCase = sleeveDetection.getPosition().toString();
            }

        }catch (Exception exception){
            camera.stopStreaming();
            dashboard.stopCameraStream();
            Log.v("EROARE ",exception.getMessage());
        }

        waitForStart();

        if(isStarted())
        {
            camera.stopStreaming();
            dashboard.stopCameraStream();
            try {
                Autonomous();
            } catch (java.net.SocketException exception) {
                Log.v("EROARE",exception.getMessage());
            }
            robot.armRotator.setPosition(ArmRotatorPosition.HALF);
        }
    }

    private void Autonomous() throws java.net.SocketException{

        GoesForPreCone = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    robot.claw.setPosition(ClawPosition.CLOSED);
                    robot.armRotator.setPosition(ArmRotatorPosition.UP);
                })
                .strafeRight(30)
                .addTemporalMarker(1,()-> {
                    robot.liftUp(LiftPosition.LIFT_UP_AUTO);
                })
                .splineToConstantHeading(new Vector2d(45, 10.5),Math.toRadians(-10),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(1.5, () -> robot.followTrajectorySequenceAsync(PlacesPreCone))
                .build();

        PlacesPreCone = robot.trajectorySequenceBuilder(GoesForPreCone.end())
                .addDisplacementMarker(()->{
                    robot.liftUp(LiftPosition.LIFT_UP_AUTO);
                })
                .addTemporalMarker(1.1,() -> {
                    robot.armRotator.setPosition(ArmRotatorPosition.HIGH_AUTO);
                })
                .lineToLinearHeading(new Pose2d(42,9.5, Math.toRadians(24)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .waitSeconds(0.1)
                .addTemporalMarker(1.3,()-> {
                    robot.claw.setPosition(ClawPosition.INTERMEDIATE);
                })
                .build();
//        PlacesPreCone = robot.trajectorySequenceBuilder(GoesForPreCone.end())
//                .addDisplacementMarker(()->{
//                    robot.liftUp(LiftPosition.LIFT_UP_AUTO + CorectieUp + 200);
//                })
//                .addTemporalMarker(1.1+waitTime,() -> {
//                    robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
//                })
//                .lineToLinearHeading(new Pose2d(42.7,-4.1, Math.toRadians(-10)),
//                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
//                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(25))
//                .waitSeconds(0.7)
//                .addTemporalMarker(1.6+waitTime,()-> {
//                    robot.claw.setPosition(ClawPosition.INTERMEDIATE);
//                })
//                .build();

        robot.followTrajectorySequence(GoesForPreCone);

//        for(int i=1;i<=5;++i) {
//            robot.updatePoseEstimate();
//            switch (i){
//                case 1 :
//                    takeX = 59;
//                    takeY = +12;
//                    placeX = 42.4;
//                    placeY = +10;
//                    corectieUp = 0;
//                    corectieDown = 730;
//                    currentPosition = PlacesPreCone.end();
//                    break;
//                case 2 :
//                    takeX = 58.5;
//                    takeY = +12;
//                    placeX = 42.4;
//                    placeY = +10;
//                    corectieUp = 0;
//                    corectieDown = 600;
//                    currentPosition = PlacesCone.end();
//                    break;
//                case 3 :
//                    takeX = 57.8;
//                    takeY = 12.3;
//                    corectieUp = 0;
//                    corectieDown = 440;
//                    currentPosition = PlacesCone.end();
//                    break;
//                case 4 :
//                    takeX = 57.3;
//                    takeY = 12.3;
//                    corectieUp = 0;
//                    corectieDown = 300;
//                    currentPosition = PlacesCone.end();
//                    break;
//                case 5 :
//                    takeX = 56.5;
//                    takeY = 12.3;
//                    corectieUp = 0;
//                    corectieDown = 0;
//                    currentPosition = PlacesCone.end();
//                    break;
//            }
//            //60 ticks ... 0.2 X
//
//            AlignForCone = robot.trajectorySequenceBuilder(currentPosition)
//                    .addDisplacementMarker(() -> {
//                        robot.liftUp(LiftPosition.LIFT_DOWN - corectieDown);
//                        robot.armRotator.setPosition((ArmRotatorPosition.HALF));
//                        robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
//                        robot.claw.setPosition(ClawPosition.WIDE_OPENED);
//                    })
//                    .addTemporalMarker(0.3,()->{
//                        robot.armRotator.setPosition(ArmRotatorPosition.DOWN);
//                    })
//                    .splineToLinearHeading(new Pose2d(44.7,11,0),Math.toRadians(-70),
//                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
//                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(25))
//                    .addTemporalMarker(1.4,() -> {
//                        robot.followTrajectorySequenceAsync(GoesForCone);
//                    })
////                    .addTemporalMarker(1.4, () -> {
////                        robot.claw.setPosition(ClawPosition.CLOSED);
////                    })
////                    .addTemporalMarker(2.3, () -> {
////                        robot.armRotator.setPosition(ArmRotatorPosition.HALF);
////                    })
////                    .addTemporalMarker(2.4+waitTime, () -> {
////                        //robot.armRotator.setPosition((ArmRotatorPosition.HALF));
////                        robot.liftUp(LiftPosition.LIFT_UP_AUTO + CorectieUp);
////                    })
////                    .addTemporalMarker(2.4+waitTime, () -> {
////                        //robot.followTrajectorySequenceAsync(PlacesFirstCone);
////                    })
//                    .build();
//
//            GoesForCone = robot.trajectorySequenceBuilder(AlignForCone.end())
//                    .lineToConstantHeading(new Vector2d(takeX,takeY),
//                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
//                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(30))
//                    .addTemporalMarker(1.4,() -> {
//                        robot.claw.setPosition(ClawPosition.CLOSED);
//                    })
//                    .waitSeconds(0.3)
//                    .addTemporalMarker(2,() -> {
//                        robot.followTrajectorySequenceAsync(AlignForPlace);
//                    })
//                    .build();
//
//            AlignForPlace = robot.trajectorySequenceBuilder(GoesForCone.end())
//                    .addDisplacementMarker(() -> {
//                        robot.armRotator.setPosition(ArmRotatorPosition.HALF);
//                    })
//                    .addTemporalMarker(0.8, () -> {
//                        robot.liftUp(LiftPosition.LIFT_UP_AUTO + corectieUp);
//                        robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
//                    })
//                    .lineToConstantHeading(new Vector2d(56,10),
//                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
//                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(40))
//                    .addTemporalMarker(1,() -> {
//                        robot.followTrajectorySequenceAsync(PlacesCone);
//                    })
//                    .build();
//
//            PlacesCone = robot.trajectorySequenceBuilder(AlignForPlace.end())
//                    .addDisplacementMarker(() -> {
//                        robot.armRotator.setPosition(ArmRotatorPosition.HALF);
//                    })
//                    .addTemporalMarker(0.8, () -> {
//                        robot.liftUp(LiftPosition.LIFT_UP_AUTO + corectieUp);
//                        robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
//                    })
//                    //angle.AlphaRedAngleHigh(42,-9.5)
////                    .splineTo(new Vector2d(42,-9.5),Math.toRadians(-24),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
////                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
////                            SampleMecanumDrive.getAccelerationConstraint(30))
//                    .splineToLinearHeading(new Pose2d(placeX,placeY,Math.toRadians(32)),Math.toRadians(70),
//                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
//                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(30))
//                    .addTemporalMarker(1.1, () -> {
//                        robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
//                    })
//                    .addTemporalMarker(1.5, () -> {
//                        robot.claw.setPosition(ClawPosition.INTERMEDIATE);
//                    })
//                    .build();
//
//            robot.followTrajectorySequence(AlignForCone);
//        }

/*
        GoesForPreCone = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    robot.claw.setPosition(ClawPosition.CLOSED);
                    robot.armRotator.setPosition(ArmRotatorPosition.UP);
                })
                .strafeLeft(30)
                .addTemporalMarker(1.2,()-> {
                    robot.liftUp(LiftPosition.LIFT_UP_AUTO+CorectieUp);
                })
                .splineToConstantHeading(new Vector2d(45.5, -10.5),Math.toRadians(10),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(1.5, () -> robot.followTrajectorySequenceAsync(PlacesPreCone))
                .build();

        PlacesPreCone = robot.trajectorySequenceBuilder(GoesForPreCone.end())
                .addDisplacementMarker(()->{
                    robot.liftUp(LiftPosition.LIFT_UP_AUTO + CorectieUp + 200);
                })
                .addTemporalMarker(1.1+waitTime,() -> {
                    robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
                })
                .lineToLinearHeading(new Pose2d(45.4,-6.1, Math.toRadians(-8)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .waitSeconds(0.7)
                .addTemporalMarker(1.6+waitTime,()-> {
                    robot.claw.setPosition(ClawPosition.INTERMEDIATE);
                })
                .build();

        robot.followTrajectorySequence(GoesForPreCone);

        for(int i=1;i<=5;++i) {
            robot.updatePoseEstimate();
            switch(i)
            {
                case 1:
                    CorectieUp = -60;
                    CorectieDown = 750;
                    angleCorrection = (i+2)/2;
                    takeX = 61;
                    waitTime = 0.6;
                    break;
                case 2:
                    takeX = 61;
                    takeY = -10.6;
                    CorectieDown=630;
                    CorectieUp = -60;
                    waitTime = 0.6;
                    angleCorrection = i/2;
                    break;
                case 3:
                    takeX = 60.6;
                    takeY = -11.2;
                    CorectieDown=430;
                    CorectieUp = -60;
                    waitTime = 0.6;
                    angleCorrection = i/2;
                    break;
                case 4:
                    takeX = 59.1;
                    takeY = -11.2;
                    CorectieUp = -60;
                    CorectieDown=180;
                    waitTime = 0.6;
                    angleCorrection = i/2;
                    break;
                case 5:
                    takeX = 57.2;
                    takeY = -11.2;
                    CorectieUp = -20;
                    CorectieDown=0;
                    waitTime = 0.6;
                    angleCorrection = i/2;
                    break;
            }
            //TODO gandit aici la PlacesPreCone.end() (el defapt nu ramane la placesprecone)
            GoesForFirstCone = robot.trajectorySequenceBuilder(PlacesPreCone.end())
                    .addDisplacementMarker(() -> {
                        robot.liftUp(LiftPosition.LIFT_DOWN - CorectieDown);
                        robot.armRotator.setPosition((ArmRotatorPosition.HALF));
                        robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
                        robot.claw.setPosition(ClawPosition.WIDE_OPENED);
                    })
                    .addTemporalMarker(0.3,()->{
                        robot.armRotator.setPosition(ArmRotatorPosition.DOWN);
                    })
                    .lineToLinearHeading(new Pose2d(takeX, takeY,
                                    angle.AlphaRedAngleCons(robot.getPoseEstimate().getX(),robot.getPoseEstimate().getY())),
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(18))
                    .addTemporalMarker(1.4, () -> {
                        robot.claw.setPosition(ClawPosition.CLOSED);
                    })
                    .addTemporalMarker(2.3, () -> {
                        robot.armRotator.setPosition(ArmRotatorPosition.HALF);
                    })
                    .addTemporalMarker(2.4+waitTime, () -> {
                        //robot.armRotator.setPosition((ArmRotatorPosition.HALF));
                        robot.liftUp(LiftPosition.LIFT_UP_AUTO + CorectieUp);
                    })
                    .addTemporalMarker(2.4+waitTime, () -> {
                        robot.followTrajectorySequenceAsync(PlacesFirstCone);
                    })
                    .build();

            PlacesFirstCone = robot.trajectorySequenceBuilder(GoesForFirstCone.end())
                    .addDisplacementMarker(() -> {
                        robot.updatePoseEstimate();
                        robot.liftUp(LiftPosition.LIFT_UP_AUTO + CorectieUp);
                    })
                    .waitSeconds(waitTime)
                    .addTemporalMarker(0.5, () -> {
                        robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
                    })
                    .lineToLinearHeading(new Pose2d(46, -8.1,
                                    angle.AlphaRedAngleHigh(robot.getPoseEstimate().getX(),robot.getPoseEstimate().getY())),
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(18))
                    .addTemporalMarker(1.1+waitTime, () -> {
                        robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
                    })
                    .addTemporalMarker(1.6+waitTime, () -> {
                        robot.claw.setPosition(ClawPosition.INTERMEDIATE);
                    })
                    .build();

            robot.followTrajectorySequence(GoesForFirstCone);
        }
        park();

 */
    }


        private void park ()
    {
        robot.armRotator.setPosition(ArmRotatorPosition.UP);
        switch (ParkCase) {
            case "LEFT":
                TrajectorySequence ParkL = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .addTemporalMarker(0, () -> {
                            robot.liftUp(LiftPosition.LIFT_DOWN);
                            robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
                        })
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(58,+10,Math.toRadians(270)))
                        .build();

                robot.followTrajectorySequence(ParkL);
                robot.updatePoseEstimate();
                break;
            case "CENTER":
                TrajectorySequence ParkC = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .addTemporalMarker(0, () -> {
                            robot.liftUp(LiftPosition.LIFT_DOWN);
                            robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
                        })
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(38,+10,Math.toRadians(270)))
                        .build();

                robot.followTrajectorySequence(ParkC);
                robot.updatePoseEstimate();
                break;
            case "RIGHT":
                TrajectorySequence ParkR = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .addTemporalMarker(0, () -> {
                            robot.liftUp(LiftPosition.LIFT_DOWN);
                            robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
                        })
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(13,6,Math.toRadians(270)))
                        .build();

                robot.followTrajectorySequence(ParkR);
                robot.updatePoseEstimate();
                break;

        }
    }

}