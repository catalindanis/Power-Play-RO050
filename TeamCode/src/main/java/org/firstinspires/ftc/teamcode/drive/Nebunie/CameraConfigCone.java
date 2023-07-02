package org.firstinspires.ftc.teamcode.drive.Nebunie;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CameraConfigCone extends OpenCvPipeline {

    Telemetry telemetry;

    Mat mat = new Mat();
    public enum Location {
        RIGHT,
        LEFT,
        CENTER,
        VERY_LEFT,
        VERY_RIGHT,
        NONE
    }

    private volatile Location location = Location.NONE;

    static final Rect VERY_LEFT_ROI = new Rect(
            new Point(0, 120),
            new Point(60, 160));
    static final Rect LEFT_ROI = new Rect(
            new Point(60, 120),
            new Point(100, 160));
    static final Rect CENTER_ROI = new Rect(
            new Point(100, 120),
            new Point(140, 160));
    static final Rect RIGHT_ROI = new Rect(
            new Point(140,120),
            new Point(180,160));
    static final Rect VERY_RIGHT_ROI = new Rect(
            new Point(180,120),
            new Point(240,160));

    static double PERCENT_CENTER_THRESHOLD = 0.15;
    static double PERCENT_ORIENTATION_TRESHOLD = 0.2;

    static double veryLeftValue = 0;
    static double leftValue = 0;
    static double centerValue = 0;
    static double rightValue = 0;
    static double veryRightValue = 0;

    public CameraConfigCone(Telemetry t)
    {
        telemetry=t;
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //Scalar lowHSV = new Scalar(168, 158, 154);
        //Scalar highHSV = new Scalar(179, 236, 255);

        Scalar lowHSV = new Scalar(20, 70, 80);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat veryleft = mat.submat(VERY_LEFT_ROI);
        Mat left = mat.submat(LEFT_ROI);
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat veryright = mat.submat(VERY_RIGHT_ROI);

        veryLeftValue = Core.sumElems(veryleft).val[0] / VERY_LEFT_ROI.area() / 255;
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double veryRightValue = Core.sumElems(veryright).val[0] / VERY_RIGHT_ROI.area() / 255;

        veryleft.release();
        left.release();
        center.release();
        right.release();
        veryright.release();

        telemetry.addData("Very left raw value", (int) Core.sumElems(veryleft).val[0]);
        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Center raw value", (int) Core.sumElems(center).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Very right raw value", (int) Core.sumElems(veryright).val[0]);
        telemetry.addData("Very left percentage", Math.round(veryLeftValue * 100) + "%");
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Very right percentage", Math.round(veryRightValue * 100) + "%");

        boolean isVeryLeft = veryLeftValue > PERCENT_ORIENTATION_TRESHOLD;
        boolean isLeft = leftValue > PERCENT_ORIENTATION_TRESHOLD;
        boolean isCenter = centerValue > PERCENT_CENTER_THRESHOLD;
        boolean isRight = rightValue > PERCENT_ORIENTATION_TRESHOLD;
        boolean isVeryRight = veryRightValue > PERCENT_ORIENTATION_TRESHOLD;

        if (isCenter) {
            location = Location.CENTER;
            telemetry.addData("CENTER", "");
        }
        else if (isRight) {
            location = Location.RIGHT;
            telemetry.addData("RIGHT", "");
        }
        else if(isLeft){
            location = Location.LEFT;
            telemetry.addData("LEFT", "");
        }
        else if(isVeryLeft){
            location = Location.VERY_LEFT;
            telemetry.addData("VERY LEFT","");
        }
        else if(isVeryRight){
            location = Location.VERY_RIGHT;
            telemetry.addData("VERY RIGHT","");
        }
        else {
            location = Location.NONE;
            telemetry.addData("NONE","");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, CENTER_ROI, location == Location.CENTER? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, VERY_RIGHT_ROI, location == Location.VERY_RIGHT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, VERY_LEFT_ROI, location == Location.VERY_LEFT? colorSkystone:colorStone);
        return mat;
    }

    public Location getLocation() {
        return location;
    }

}

