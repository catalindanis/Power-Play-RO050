package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import android.view.inputmethod.CorrectionInfo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.ComputedValues.StickAngles;
import org.firstinspires.ftc.teamcode.drive.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Config.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ArmRotatorPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ClawPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ClawRotatorPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.LiftPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.Positions;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;

@Disabled
@Autonomous(name="Test", group="Auto")
public class Test extends LinearOpMode {

    FtcDashboard dashboard;
    private SampleMecanumDrive robot = null;
    private StickAngles angle = null;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timp_scurs = new ElapsedTime();

    Pose2d currentPose;

    int CorectieUp = -180,CorectieDown = 850;

    TrajectorySequence GoesForPreCone;
    TrajectorySequence PlacesPreCone; //async
    TrajectorySequence GoesForFirstCone;
    TrajectorySequence PlacesFirstCone;
    TrajectorySequence GoesForSecondCone;
    TrajectorySequence PlacesSecondCone;
    TrajectorySequence GoesForThirdCone;
    TrajectorySequence PlacesThirdCone;
    TrajectorySequence GoesForFourthCone;
    TrajectorySequence PlacesFourthCone;
    TrajectorySequence GoesForFifthCone;
    TrajectorySequence PlacesFifthCone;
    double takeX = 62.4;
    double takeY = -10.49;
    double waitTime = 0;
    double secondWaitTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleMecanumDrive(hardwareMap);

        angle = new StickAngles();

        robot.setPoseEstimate(new Pose2d(35.18152727949673, -62.5678946778844));
        robot.setExternalHeading(Math.toRadians(0));

        dashboard = FtcDashboard.getInstance();

        robot.claw.setPosition(ClawPosition.CLOSED);
        sleep(1000);
        robot.initAutonomous();

        waitForStart();

        test();

    } //y 6.5

    private void test(){

        GoesForPreCone = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    robot.claw.setPosition(ClawPosition.CLOSED);
                    robot.armRotator.setPosition(ArmRotatorPosition.UP);
                })
                .strafeLeft(30)
                .addTemporalMarker(1.2,()-> {
                    robot.liftUp(LiftPosition.LIFT_UP+CorectieUp);
                })
                .splineToConstantHeading(new Vector2d(45.5, -10.5),Math.toRadians(10), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addTemporalMarker(1.5, () -> robot.followTrajectorySequenceAsync(PlacesPreCone))
                .build();


        //ASYNC SHIT
        PlacesPreCone = robot.trajectorySequenceBuilder(GoesForPreCone.end())
                .addDisplacementMarker(()->{
                    robot.liftUp(LiftPosition.LIFT_UP + CorectieUp + 150);
                })
                .addTemporalMarker(1.1+waitTime,() -> {
                    robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
                })
                .lineToLinearHeading(new Pose2d(44.5,-5.5, Math.toRadians(-12)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .waitSeconds(0.7)
                .addTemporalMarker(1.6+waitTime,()-> {
                    robot.claw.setPosition(ClawPosition.INTERMEDIATE);
                })
                .build();
       //ASYNC SHIT

        robot.followTrajectorySequence(GoesForPreCone);

        CorectieUp = -50;

        for(int i=1;i<=5;++i) {
            switch(i)
            {
                case 2:
                    takeX = 61.1;
                    takeY = -10.49;
                    CorectieDown=690;
                    CorectieUp = 30;
                    break;
                case 3:
                    takeX = 59.9;
                    takeY = -10.49;
                    CorectieDown=470;
                    break;
                case 4:
                    takeX = 57.9;
                    takeY = -10.49;
                    CorectieDown=210;
                    waitTime = 0.5;
                    //secondWaitTime = 0.1;
                    break;
                case 5:
                    takeX = 57.3;
                    takeY = -10.49;
                    CorectieDown=0;
                    //waitTime = 0.5;
                    //secondWaitTime = 0.3;
                    break;
            }

            GoesForFirstCone = robot.trajectorySequenceBuilder(PlacesPreCone.end())
                    .addDisplacementMarker(() -> {
                        robot.liftUp(LiftPosition.LIFT_DOWN - CorectieDown);
                        robot.armRotator.setPosition((ArmRotatorPosition.HALF));
                        robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
                    })
                    .addTemporalMarker(1, () -> {
                        robot.armRotator.setPosition(ArmRotatorPosition.DOWN);
                        robot.claw.setPosition(ClawPosition.WIDE_OPENED);
                    })
                    .lineToLinearHeading(new Pose2d(takeX, takeY, angle.AlphaRedAngleCons(takeX,takeY)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(18))
                    .addTemporalMarker(1.4, () -> {
                        robot.claw.setPosition(ClawPosition.CLOSED);
                    })
                    .addTemporalMarker(2.3+waitTime, () -> {
                        robot.armRotator.setPosition((ArmRotatorPosition.HALF));
                        robot.liftUp(LiftPosition.LIFT_UP + CorectieUp);
                    })
                    .addTemporalMarker(2.3+waitTime, () -> {
                        robot.followTrajectorySequenceAsync(PlacesFirstCone);
                    })
                    .build();


            // Async SHIT
            PlacesFirstCone = robot.trajectorySequenceBuilder(GoesForFirstCone.end())
                    .addDisplacementMarker(() -> {
                        robot.liftUp(LiftPosition.LIFT_UP + CorectieUp);
                    })
                    .waitSeconds(waitTime)
                    .addTemporalMarker(0.1+waitTime, () -> {
                        robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
                    })
                    .lineToLinearHeading(new Pose2d(45.6, -6.5, angle.AlphaRedAngleHigh(45, -6.5)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(18))
//                    .waitSeconds(0.1)
                    .addTemporalMarker(1.1+waitTime, () -> {
                        robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
                    })
                    .addTemporalMarker(1.6+waitTime, () -> {
                        robot.claw.setPosition(ClawPosition.INTERMEDIATE);
                    })
                    .build();

            robot.followTrajectorySequence(GoesForFirstCone);
            //ASYNC SHIT
        }

        /*

       670
       480
       270
       120
         */

      //  robot.followTrajectorySequence(PlacesFirstCone);
//
//        CorectieDown = 670;
//
//        GoesForSecondCone = robot.trajectorySequenceBuilder(PlacesFirstCone.end())
//                .addDisplacementMarker(() -> {
//                    robot.liftUp(LiftPosition.LIFT_DOWN-CorectieDown);
//                    robot.armRotator.setPosition((ArmRotatorPosition.HALF));
//                    robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
//                })
//                .addTemporalMarker(1,() -> {
//                    robot.armRotator.setPosition(ArmRotatorPosition.DOWN);
//                    robot.claw.setPosition(ClawPosition.WIDE_OPENED);
//                })
//                .lineToConstantHeading(new Vector2d(61.5,-10.49072854593326),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(15))
//                .waitSeconds(0.2)
//                .build();
//
//        robot.followTrajectorySequence(GoesForSecondCone);
//
//        robot.claw.setPosition(ClawPosition.CLOSED);
//        sleep(200);
//        robot.armRotator.setPosition((ArmRotatorPosition.HALF));
//
//        PlacesSecondCone = robot.trajectorySequenceBuilder(GoesForSecondCone.end())
//                .addDisplacementMarker(()->{
//                    robot.liftUp(LiftPosition.LIFT_UP + CorectieUp);
//                })
//                .addTemporalMarker(0.1, () -> {
//                    robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
//                })
//                .lineToLinearHeading(new Pose2d(44,-6.5,angle.RightRedAngleHigh(45,-6.5)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(15))
//                .waitSeconds(0.1)
//                .addTemporalMarker(0.7, () -> {
//                    robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
//                })
//                .addTemporalMarker(1.9, () ->{
//                    robot.claw.setPosition(ClawPosition.INTERMEDIATE);
//                })
//                .build();
//
//        robot.followTrajectorySequence(PlacesSecondCone);
//
//        CorectieDown = 480;
//
//        GoesForThirdCone = robot.trajectorySequenceBuilder(PlacesSecondCone.end())
//                .addDisplacementMarker(() -> {
//                    robot.liftUp(LiftPosition.LIFT_DOWN-CorectieDown);
//                    robot.armRotator.setPosition((ArmRotatorPosition.HALF));
//                    robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
//                })
//                .addTemporalMarker(1,() -> {
//                    robot.armRotator.setPosition(ArmRotatorPosition.DOWN);
//                    robot.claw.setPosition(ClawPosition.WIDE_OPENED);
//                })
//                .lineToConstantHeading(new Vector2d(59.9,-10.49072854593326),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(15))
//                .waitSeconds(0.2)
//                .build();
//
//        robot.followTrajectorySequence(GoesForThirdCone);
//
//        robot.claw.setPosition(ClawPosition.CLOSED);
//        sleep(200);
//        robot.armRotator.setPosition((ArmRotatorPosition.HALF));
//
//        PlacesThirdCone = robot.trajectorySequenceBuilder(GoesForThirdCone.end())
//                .addDisplacementMarker(()->{
//                    robot.liftUp(LiftPosition.LIFT_UP + CorectieUp);
//                })
//                .addTemporalMarker(0.1, () -> {
//                    robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
//                })
//                .lineToLinearHeading(new Pose2d(44,-6.5,angle.RightRedAngleHigh(45,-6.5)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(15))
//                .waitSeconds(0.1)
//                .addTemporalMarker(0.7, () -> {
//                    robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
//                })
//                .addTemporalMarker(1.9, () ->{
//                    robot.claw.setPosition(ClawPosition.INTERMEDIATE);
//                })
//                .build();
//
//        robot.followTrajectorySequence(PlacesThirdCone);
//
////        TakesFirstCone = robot.trajectorySequenceBuilder(GoesForFirstCone.end())
////                .addDisplacementMarker(()->robot.liftUp(LiftPosition.LIFT_DOWN-CorectieDown))
////                .lineToConstantHeading(new Vector2d(63,-13))
////                .addTemporalMarker(1,()->{
////                    robot.claw.setPosition(ClawPosition.CLOSED);
////                    //robot.followTrajectorySequenceAsync(PlacesFirstTakenCone);
////                })
////                .build();
////
////        PlacesFirstTakenCone = robot.trajectorySequenceBuilder(TakesFirstCone.end())
////                .addDisplacementMarker(()->robot.liftUp(LiftPosition.LIFT_UP))
////                .lineToLinearHeading(new Pose2d(48.01597903331522,-8.49072854593326,Math.toRadians(-12)))
////                .waitSeconds(0.3)
////                .addTemporalMarker(0.8,()-> {
////                    robot.claw.setPosition(ClawPosition.OPENED);
////                })
////                .build();
//        //ASYNC SHIT
//
//        CorectieDown = 270;
//
//        GoesForFourthCone = robot.trajectorySequenceBuilder(PlacesSecondCone.end())
//                .addDisplacementMarker(() -> {
//                    robot.liftUp(LiftPosition.LIFT_DOWN-CorectieDown);
//                    robot.armRotator.setPosition((ArmRotatorPosition.HALF));
//                    robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
//                })
//                .addTemporalMarker(1,() -> {
//                    robot.armRotator.setPosition(ArmRotatorPosition.DOWN);
//                    robot.claw.setPosition(ClawPosition.WIDE_OPENED);
//                })
//                .lineToConstantHeading(new Vector2d(59.4,-10.49072854593326),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(15))
//                .waitSeconds(0.2)
//                .build();
//
//        robot.followTrajectorySequence(GoesForFourthCone);
//
//        robot.claw.setPosition(ClawPosition.CLOSED);
//        sleep(200);
//        robot.armRotator.setPosition((ArmRotatorPosition.HALF));
//
//        PlacesFourthCone = robot.trajectorySequenceBuilder(GoesForThirdCone.end())
//                .addDisplacementMarker(()->{
//                    robot.liftUp(LiftPosition.LIFT_UP + CorectieUp);
//                })
//                .addTemporalMarker(0.1, () -> {
//                    robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
//                })
//                .lineToLinearHeading(new Pose2d(44,-6.5,angle.RightRedAngleHigh(45,-6.5)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(15))
//                .waitSeconds(0.1)
//                .addTemporalMarker(0.7, () -> {
//                    robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
//                })
//                .addTemporalMarker(1.9, () ->{
//                    robot.claw.setPosition(ClawPosition.INTERMEDIATE);
//                })
//                .build();
//
//        robot.followTrajectorySequence(PlacesFourthCone);
//
//        CorectieDown = 120;
//
//        GoesForFifthCone = robot.trajectorySequenceBuilder(PlacesSecondCone.end())
//                .addDisplacementMarker(() -> {
//                    robot.liftUp(LiftPosition.LIFT_DOWN-CorectieDown);
//                    robot.armRotator.setPosition((ArmRotatorPosition.HALF));
//                    robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
//                })
//                .addTemporalMarker(1,() -> {
//                    robot.armRotator.setPosition(ArmRotatorPosition.DOWN);
//                    robot.claw.setPosition(ClawPosition.WIDE_OPENED);
//                })
//                .lineToConstantHeading(new Vector2d(59.4,-10.49072854593326),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(15))
//                .waitSeconds(0.2)
//                .build();
//
//        robot.followTrajectorySequence(GoesForFifthCone);
//
//        robot.claw.setPosition(ClawPosition.CLOSED);
//        sleep(200);
//        robot.armRotator.setPosition((ArmRotatorPosition.HALF));
//
//        PlacesFifthCone = robot.trajectorySequenceBuilder(GoesForThirdCone.end())
//                .addDisplacementMarker(()->{
//                    robot.liftUp(LiftPosition.LIFT_UP + CorectieUp);
//                })
//                .addTemporalMarker(0.1, () -> {
//                    robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
//                })
//                .lineToLinearHeading(new Pose2d(44,-6.5,angle.RightRedAngleHigh(45,-6.5)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(15))
//                .waitSeconds(0.1)
//                .addTemporalMarker(0.7, () -> {
//                    robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
//                })
//                .addTemporalMarker(1.9, () ->{
//                    robot.claw.setPosition(ClawPosition.INTERMEDIATE);
//                })
//                .build();
//
//        robot.followTrajectorySequence(PlacesFifthCone);
//
//        robot.updatePoseEstimate();
//        currentPose = robot.getPoseEstimate();

    }
}