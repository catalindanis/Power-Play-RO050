package org.firstinspires.ftc.teamcode.drive.OpenCVCameraDetection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.OpenCVCameraDetection.DetectarePozitie;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "Signal Sleeve Test")
public class morti extends LinearOpMode {

    FtcDashboard dashboard;
    private DetectarePozitie sleeveDetection;
    private OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";

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
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                dashboard.startCameraStream(camera, 120);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }

        waitForStart();
        switch (sleeveDetection.getPosition()) {
            case LEFT:
                telemetry.addData("left: ",1);
                telemetry.update();
                break;
            case CENTER:
                telemetry.addData("center: ",2);
                telemetry.update();
                break;
            case RIGHT:
                telemetry.addData("right: ",3);
                telemetry.update();
                break;

        }
        camera.stopStreaming();
    }
}