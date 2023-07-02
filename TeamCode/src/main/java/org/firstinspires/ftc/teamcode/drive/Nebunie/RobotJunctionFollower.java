package org.firstinspires.ftc.teamcode.drive.Nebunie;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.ComputedValues.StickAngles;
import org.firstinspires.ftc.teamcode.drive.Config.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RobotJunctionFollower", group = "Auto")
public class RobotJunctionFollower extends LinearOpMode {

    FtcDashboard dashboard;
    private CameraConfigJunction sleeveDetection;
    private OpenCvCamera camera;
    private StickAngles angle = null;

    private String webcamName = "Webcam 1";

    private SampleMecanumDrive robot;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new CameraConfigJunction(telemetry);
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

        //robot.setPoseEstimate(new Pose2d(35.18152727949673, 62.5678946778844));
        //robot.setExternalHeading(Math.toRadians(0));

        //robot.claw.setPosition(ClawPosition.CLOSED);
        sleep(1000);
        //robot.initAutonomous();

        waitForStart();

        if(isStarted())
        {
            Autonomous();
        }

    }

    private void Autonomous()
    {
        double power = 0.1;
        double leftAngleCorr = 0;
        double rightAngleCorr = 0;
        while(opModeIsActive()){
            robot.setMotorPowers(power+leftAngleCorr,power+leftAngleCorr,power+rightAngleCorr,power+rightAngleCorr);

            if(sleeveDetection.getLocation() == CameraConfigJunction.Location.CENTER) {
                leftAngleCorr = 0;
                rightAngleCorr = 0;
            }
            else if(sleeveDetection.getLocation() == CameraConfigJunction.Location.RIGHT){
                rightAngleCorr = CameraConfigJunction.rightValue;
                leftAngleCorr = 0;
            }
            else if(sleeveDetection.getLocation() == CameraConfigJunction.Location.LEFT){
                rightAngleCorr = 0;
                leftAngleCorr = CameraConfigJunction.leftValue;
            }
            else if(sleeveDetection.getLocation() == CameraConfigJunction.Location.VERY_RIGHT){
                rightAngleCorr = CameraConfigJunction.veryRightValue;
                leftAngleCorr = 0;
            }
            else if(sleeveDetection.getLocation() == CameraConfigJunction.Location.VERY_LEFT){
                rightAngleCorr = 0;
                leftAngleCorr = CameraConfigJunction.veryLeftValue;
            }
            else{
                //power = 0;
                leftAngleCorr = 0;
                rightAngleCorr = 0;
            }

            telemetry.addData("Location ",sleeveDetection.getLocation());
            telemetry.update();
        }
    }


    private void park ()
    {

    }

}