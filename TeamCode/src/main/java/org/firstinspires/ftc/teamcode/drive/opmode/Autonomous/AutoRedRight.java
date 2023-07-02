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
import org.opencv.core.Core;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.net.SocketException;

@Autonomous(name = "AutoRedRight", group = "Auto")
public class AutoRedRight extends LinearOpMode {

    FtcDashboard dashboard;
    private DetectarePozitie sleeveDetection;
    private OpenCvCamera camera;
    private StickAngles angle = null;

    private String webcamName = "Webcam 1";

    private SampleMecanumDrive robot;

    String ParkCase;

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
    int corectieUp = 0,corectieDown = 0;

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

        robot.setPoseEstimate(new Pose2d(35.18152727949673, -62.5678946778844,Math.toRadians(0)));

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
                .lineToConstantHeading(new Vector2d(35,-25.5),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .strafeLeft(30)
                .addTemporalMarker(1.3,()-> {
                    robot.liftUp(LiftPosition.LIFT_UP_AUTO);
                })
                .lineToConstantHeading(new Vector2d(45, -10.5),
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
                .lineToLinearHeading(new Pose2d(42,-6.5, Math.toRadians(-21)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .waitSeconds(0.3)
                .addTemporalMarker(1.1,()-> {
                    robot.claw.setPosition(ClawPosition.INTERMEDIATE);
                })
                .build();

        robot.followTrajectorySequence(GoesForPreCone);

        for(int i=1;i<=5;++i) {

            switch (i){
                case 1 :
                    takeX = 59.9;
                    takeY = -11.5;
                    placeX = 43.5;
                    placeY = -9.5;
                    //corectieUp = -50;
                    corectieDown = 630;
                    currentPosition = PlacesPreCone.end();
                    break;
                case 2 :
                    takeX = 59.5;
                    corectieDown = 500;
                    currentPosition = PlacesCone.end();
                    break;
                case 3 :
                    takeX = 58.3;
                    //takeY = -14;
                    placeY = -10.5;
                    corectieDown = 320;
                    currentPosition = PlacesCone.end();
                    break;
                case 4 :
                    takeX = 57.2;
                    corectieDown = 210;
                    //corectieUp = 50;
                    angleCorrection = 0.7;
                    currentPosition = PlacesCone.end();
                    break;
                case 5 :
                    takeX = 56.7;
                    corectieDown = 0;
                    waitTime = 0.2;
                    angleCorrection = 1;
                    currentPosition = PlacesCone.end();
                    break;
            }

            AlignForCone = robot.trajectorySequenceBuilder(currentPosition)
                    .addDisplacementMarker(() -> {
                        robot.liftUp(LiftPosition.LIFT_DOWN - corectieDown);
                        robot.armRotator.setPosition((ArmRotatorPosition.HALF));
                        robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
                        robot.claw.setPosition(ClawPosition.WIDE_OPENED);
                    })
                    .addTemporalMarker(0.3,()->{
                        robot.armRotator.setPosition(ArmRotatorPosition.DOWN);
                    })
                    .lineToLinearHeading(new Pose2d(44.7,-10,Math.toRadians(0)),
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(25))
                    .addTemporalMarker(0.9,() -> {
                        robot.followTrajectorySequenceAsync(GoesForCone);
                    })
                    .build();

            GoesForCone = robot.trajectorySequenceBuilder(AlignForCone.end())
                    .lineToConstantHeading(new Vector2d(takeX,takeY),
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .addTemporalMarker(1.1,() -> {
                        robot.claw.setPosition(ClawPosition.CLOSED);
                    })
                    .waitSeconds(0.3)
                    .addTemporalMarker(1.2,() -> {
                        robot.followTrajectorySequenceAsync(AlignForPlace);
                    })
                    .build();

            AlignForPlace = robot.trajectorySequenceBuilder(GoesForCone.end())
                    .addDisplacementMarker(() -> {
                        robot.armRotator.setPosition(ArmRotatorPosition.HALF);
                    })
                    .addTemporalMarker(0.8, () -> {
                        robot.liftUp(LiftPosition.LIFT_UP_AUTO + corectieUp);
                        robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
                    })
                    .lineToConstantHeading(new Vector2d(53.5,-9),
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .addTemporalMarker(0.9,() -> {
                        robot.followTrajectorySequenceAsync(PlacesCone);
                    })
                    .build();

            PlacesCone = robot.trajectorySequenceBuilder(AlignForPlace.end())
                    .addDisplacementMarker(() -> {
                        robot.armRotator.setPosition(ArmRotatorPosition.HALF);
                    })
                    .addTemporalMarker(0.8, () -> {
                        robot.liftUp(LiftPosition.LIFT_UP_AUTO + corectieUp);
                        robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
                    })
                    .splineToLinearHeading(new Pose2d(placeX,placeY,Math.toRadians(-31.4 - angleCorrection)),Math.toRadians(-250),
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .addTemporalMarker(0.9+waitTime, () -> {
                        robot.armRotator.setPosition(ArmRotatorPosition.HIGH_AUTO);
                    })
                    .addTemporalMarker(1.3+waitTime, () -> {
                        robot.claw.setPosition(ClawPosition.INTERMEDIATE);
                    })
                    .addTemporalMarker(1.8+waitTime, () -> {
                        robot.liftUp(LiftPosition.LIFT_DOWN - corectieDown);
                    })
                    .build();

            robot.followTrajectorySequence(AlignForCone);
        }


        //TODO INVERSAT UNGHIUL DE PARCARE PENTRU RELATIVE
        park();

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
                        .lineToSplineHeading(new Pose2d(14,-14,Math.toRadians(268)),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
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
                        .lineToSplineHeading(new Pose2d(37,-14,Math.toRadians(268)),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(30))
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
                        .lineToSplineHeading(new Pose2d(60,-14,Math.toRadians(268)),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .build();

                robot.followTrajectorySequence(ParkR);
                robot.updatePoseEstimate();
                break;
        }
    }

}