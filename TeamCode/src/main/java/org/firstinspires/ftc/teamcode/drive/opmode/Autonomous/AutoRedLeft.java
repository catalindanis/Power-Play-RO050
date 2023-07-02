package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.drive.OpenCVCameraDetection.DetectarePozitie.ParkingPosition.CENTER;

import android.icu.text.Transliterator;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "AutoRedLeft", group = "Auto")
public class AutoRedLeft extends LinearOpMode {

    FtcDashboard dashboard;
    private DetectarePozitie sleeveDetection;
    private OpenCvCamera camera;
    private StickAngles angle = null;

    private String webcamName = "Webcam 1";

    private SampleMecanumDrive robot;

    String ParkCase;

    Pose2d pozitiepl = new Pose2d();

    int CorectieUp = 30,CorectieDown = 750;

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
    double speedTime = 0;
    double angleCorrection = 0;
    int liftParkCorrection = 0;

    Pose2d currentPosition = new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new DetectarePozitie();
        camera.setPipeline(sleeveDetection);

        dashboard = FtcDashboard.getInstance();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                dashboard.startCameraStream(camera, 120);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        robot = new SampleMecanumDrive(hardwareMap);

        angle = new StickAngles();

        robot.setPoseEstimate(new Pose2d(-34.1513637177631,  -62.68015227050209,Math.toRadians(180)));
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
            robot.liftLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }

    private void Autonomous() throws java.net.SocketException {

        GoesForPreCone = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    robot.claw.setPosition(ClawPosition.CLOSED);
                    robot.armRotator.setPosition(ArmRotatorPosition.UP);
                })
                .lineToConstantHeading(new Vector2d(-37,-25.5),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .strafeRight(30)
                .addTemporalMarker(1.5,()-> {
                    robot.liftUp(LiftPosition.LIFT_UP_AUTO);
                })
                //.lineToConstantHeading(new Vector2d(-39, -10.5),
                        .lineToLinearHeading(new Pose2d(-33,-10.5,Math.toRadians(-140)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .addTemporalMarker(1.5, () -> robot.followTrajectorySequenceAsync(PlacesPreCone))
                .build();

        PlacesPreCone = robot.trajectorySequenceBuilder(GoesForPreCone.end())
                .addDisplacementMarker(()->{
                    robot.liftUp(LiftPosition.LIFT_UP_AUTO);
                })
                .addTemporalMarker(1.1,() -> {
                    robot.armRotator.setPosition(ArmRotatorPosition.HIGH_AUTO);
                })
                .lineToLinearHeading(new Pose2d(-42,-6.5, Math.toRadians(-149)),
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
                    takeX = -61;
                    takeY = -7;
                    placeX = -44.6;
                    placeY = -8.5;
                    corectieUp = -30;
                    corectieDown = 630;
                    currentPosition = PlacesPreCone.end();
                    break;
                case 2 :
                    takeX = -60.6;
                    takeY = -7.5;
                    corectieDown = 500;
                    //placeX = -46;
                    currentPosition = PlacesCone.end();
                    break;
                case 3 :
                    takeX = -60.2;
                    takeY = -7.5;
                    placeY = -7.5;
                    corectieDown = 290;
                    waitTime = 0.1;
                    speedTime = 0.1;
                    currentPosition = PlacesCone.end();
                    break;
                case 4 :
                    takeX = -59;
                    takeY = -7.5;
                    placeY = -9.5;
                    corectieDown = 210;
                    //corectieUp = 50;
                    waitTime = 0.1;
                    angleCorrection = 4;
                    currentPosition = PlacesCone.end();
                    break;
                case 5 :
                    takeX = -57.8;
                    placeY = -9.5;
                    corectieDown = 0;
                    corectieUp = -60;
                    waitTime = 0.2;
                    speedTime = 0.2;
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
                    .waitSeconds(0.1)
                    .lineToLinearHeading(new Pose2d(-49,-8,Math.toRadians(180)),
                    //.lineToLinearHeading(new Pose2d(-55,-2,Math.toRadians(220)),
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(25))
                    .addTemporalMarker(0.9,() -> {
                        robot.followTrajectorySequenceAsync(GoesForCone);
                    })
                    .build();

            GoesForCone = robot.trajectorySequenceBuilder(AlignForCone.end())
                    .lineToConstantHeading(new Vector2d(takeX,takeY),
//                    .lineToLinearHeading(new Pose2d(takeX,takeY,Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .addTemporalMarker(1.1-speedTime,() -> {
                        robot.claw.setPosition(ClawPosition.CLOSED);
                    })
                    //.waitSeconds(0.2)
                    .addTemporalMarker(1.2+waitTime,() -> {
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
                    .lineToConstantHeading(new Vector2d(-53.5,-6.5),
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
                    .splineToLinearHeading(new Pose2d(placeX,placeY,Math.toRadians(-143 + angleCorrection)),Math.toRadians(-250),
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .addTemporalMarker(0.9+waitTime,() -> {
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
                        .waitSeconds(0.4)
                        .lineToSplineHeading(new Pose2d(-65,-9,Math.toRadians(268)),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
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
                        .lineToSplineHeading(new Pose2d(-40,-9,Math.toRadians(268)),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
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
                        .waitSeconds(0.4)
                        .lineToSplineHeading(new Pose2d(-20,-9,Math.toRadians(268)),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL,
                                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .build();

                robot.followTrajectorySequence(ParkR);
                robot.updatePoseEstimate();
                break;

        }
    }

}