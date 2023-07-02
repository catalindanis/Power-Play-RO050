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

@Autonomous(name = "AutoBlueRight", group = "Auto")
public class AutoBlueRight extends LinearOpMode {

    FtcDashboard dashboard;
    private DetectarePozitie sleeveDetection;
    private OpenCvCamera camera;
    private StickAngles angle = null;

    private String webcamName = "Webcam 1";

    private SampleMecanumDrive robot;

    String ParkCase;

    int CorectieUp = -250,CorectieDown = 850;

    TrajectorySequence GoesForPreCone;
    TrajectorySequence PlacesPreCone; //async
    TrajectorySequence GoesForFirstCone;
    TrajectorySequence PlacesFirstCone; //async

    double takeX = 60.6;
    double takeY = -10.49;
    double waitTime = 0;
    double secondWaitTime = 0;
    double angleCorrection = 0;

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

        robot.setPoseEstimate(new Pose2d(35.18152727949673, -62.5678946778844));
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
                    robot.liftUp(LiftPosition.LIFT_UP_AUTO + CorectieUp + 150);
                })
                .addTemporalMarker(1.1+waitTime,() -> {
                    robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
                })
                .lineToLinearHeading(new Pose2d(45.8,-6.1, Math.toRadians(-9)),
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
            switch(i)
            {
                case 1:
                    takeY = -10.2;
                    CorectieUp = -50;
                    CorectieDown = 750;
                    angleCorrection = (i+2)/2;
                    break;
                case 2:
                    takeX = 59.8;
                    takeY = -10.6;
                    CorectieDown=600;
                    CorectieUp = 0;
                    angleCorrection = i/2;
                    break;
                case 3:
                    takeX = 59;
                    takeY = -11.2;
                    CorectieUp = 100;
                    CorectieDown=430;
                    waitTime = 0.3;
                    angleCorrection = i/2;
                    break;
                case 4:
                    takeX = 57.8;
                    takeY = -11.2;
                    CorectieUp = 100;
                    CorectieDown=160;
                    waitTime = 0.5;
                    angleCorrection = i/2;
                    break;
                case 5:
                    takeX = 57.3;
                    takeY = -11.2;
                    CorectieUp = 100;
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
                    .lineToLinearHeading(new Pose2d(takeX, takeY, angle.AlphaRedAngleCons(takeX,takeY+angleCorrection)),
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(18))
                    .addTemporalMarker(1.4, () -> {
                        robot.claw.setPosition(ClawPosition.CLOSED);
                    })
                    .addTemporalMarker(2.4+waitTime, () -> {
                        robot.armRotator.setPosition((ArmRotatorPosition.HALF));
                        robot.liftUp(LiftPosition.LIFT_UP_AUTO + CorectieUp);
                    })
                    .addTemporalMarker(2.4+waitTime, () -> {
                        robot.followTrajectorySequenceAsync(PlacesFirstCone);
                    })
                    .build();

            PlacesFirstCone = robot.trajectorySequenceBuilder(GoesForFirstCone.end())
                    .addDisplacementMarker(() -> {
                        robot.liftUp(LiftPosition.LIFT_UP_AUTO + CorectieUp);
                    })
                    .waitSeconds(waitTime)
                    .addTemporalMarker(0.1+waitTime, () -> {
                        robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
                    })
                    .lineToLinearHeading(new Pose2d(46, -8.1, angle.AlphaRedAngleHigh(46, -5.7)),
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
    }

    private void park ()
    {
        robot.armRotator.setPosition(ArmRotatorPosition.UP);
        switch (ParkCase) {
            case "RIGHT":
                TrajectorySequence ParkL = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .addTemporalMarker(0, () -> {
                            robot.liftUp(LiftPosition.LIFT_DOWN);
                            robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
                        })
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(60.7,-18,Math.toRadians(90)))
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
                        .lineToSplineHeading(new Pose2d(40,-10,Math.toRadians(90)))
                        .build();

                robot.followTrajectorySequence(ParkC);
                robot.updatePoseEstimate();
                break;
            case "LEFT":
                TrajectorySequence ParkR = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .addTemporalMarker(0, () -> {
                            robot.liftUp(LiftPosition.LIFT_DOWN);
                            robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
                        })
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(18,-10,Math.toRadians(90)))
                        .build();

                robot.followTrajectorySequence(ParkR);
                robot.updatePoseEstimate();
                break;
        }
    }

}