package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//import org.firstinspires.ftc.teamcode.drive.ClawPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ArmRotatorPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ClawPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ClawRotatorPosition;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

@Deprecated
public class RobotMap{

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public Servo claw;
    public Servo clawRotator;
    public Servo armRotator;
    public DcMotorEx liftLifter;
    public RevColorSensorV3 colorSensor;

    public BNO055IMU imu;

    public Orientation lastAngles = new Orientation();
    public double globalAngle;
    public double globalAngle2;

    private LinearOpMode opMode;

    public RobotMap(HardwareMap hardwareMap, LinearOpMode opMode){

        this.opMode = opMode;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        //inversare motoare
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");
        clawRotator = hardwareMap.get(Servo.class, "clawRotator");
        armRotator = hardwareMap.get(Servo.class, "armRotator");

        liftLifter = hardwareMap.get(DcMotorEx.class, "liftLifter");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "frontColorSensor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
    }

    public void init(){
        armRotator.setPosition(ArmRotatorPosition.DOWN);
        clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
        claw.setPosition(ClawPosition.CLOSED);
        //am sters (poate trebuiesc adaugate inapoi, dar nu cred)
        //liftLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //liftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoders(int ticks, double power){
        liftLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLifter.setTargetPosition(-ticks);
        liftLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if(deltaAngle < -180)
            deltaAngle += 360;
        else if(deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle +=deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    public double SQRT(double x) {
        if (x < 0) return -Math.sqrt(Math.abs(x));
        return Math.sqrt(x);
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }
}