package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

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
import org.firstinspires.ftc.teamcode.drive.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Config.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ArmRotatorPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ClawPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ClawRotatorPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.LiftPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.Positions;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name="lift_test_auto", group="Auto")
public class lift_test_auto extends LinearOpMode {

    FtcDashboard dashboard;
    private SampleMecanumDrive robot = null;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timp_scurs = new ElapsedTime();

    Pose2d currentPose;

    int CorectieUp = 0,CorectieDown = 0;

    TrajectorySequence GoesForPreCone;
    TrajectorySequence PlacesPreCone; //async
    TrajectorySequence GoesForFirstCone;
    TrajectorySequence TakesFirstCone;
    TrajectorySequence PlacesFirstTakenCone;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleMecanumDrive(hardwareMap);

        robot.setPoseEstimate(new Pose2d(35.18152727949673, -62.5678946778844));
        robot.setExternalHeading(Math.toRadians(0));

        dashboard = FtcDashboard.getInstance();

        robot.initAutonomous();

        waitForStart();

        test();

    }

    private void test(){

        robot.liftUp2(LiftPosition.LIFT_UP);

        sleep(3200);

        robot.liftUp(LiftPosition.LIFT_DOWN);

        sleep(3200);

        robot.liftUp(LiftPosition.LIFT_UP_AUTO);

        sleep(3200);

        robot.liftUp(LiftPosition.LIFT_DOWN);

        sleep(3200);

        robot.liftUp(LiftPosition.LIFT_UP);

        sleep(3200);

        robot.liftUp(LiftPosition.LIFT_DOWN);

        sleep(3200);

        robot.liftUp(LiftPosition.LIFT_UP);

        sleep(3200);

        robot.liftUp(LiftPosition.LIFT_DOWN);

        sleep(3200);
        robot.liftUp(LiftPosition.LIFT_UP);

        sleep(3200);

        robot.liftUp(LiftPosition.LIFT_DOWN);

        sleep(3200);

    }
}
