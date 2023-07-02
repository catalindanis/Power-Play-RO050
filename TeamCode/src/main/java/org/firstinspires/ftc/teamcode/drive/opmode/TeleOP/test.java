package org.firstinspires.ftc.teamcode.drive.opmode.TeleOP;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Config.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.Config.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */

@Disabled
@TeleOp(name="teleoppose",group = "MecanumBot")
public class test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        //RobotUsbDevice.USBIdentifiers robotUsbDevice = new RobotUsbDevice.USBIdentifiers().createLynxIdentifiers();

        waitForStart();

        while (opModeIsActive()) {
            //telemetry.addData(robotUsbDevice.bcdDevice,"###");
            //telemetry.addData("distanta ",robot.colorSensor.getDistance(DistanceUnit.CM));
        }
    }
}