package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.util.Angle;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import javax.sound.midi.Track;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double start_x=0,start_y=0;

        StickAngles angle = new StickAngles();
        //System.out.println(angle.BetaRedAngleCons(-57.5,-5));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                //Pose2d poseEstimate = new Pose2d(36.421455251915326, -33.01402312256442, Math.toRadians(90));
                // Set bot constraints: maxVel, maxAccel, maxAgVel, maxAngAccel, track width

                .setConstraints(62.01654253906262, 60, Math.toRadians(140), Math.toRadians(90), 10)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-49,  -3, Math.toRadians(-140)))
                        .lineToLinearHeading(new Pose2d(-55,-3,Math.toRadians(200)))
                        .build()
                );





        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

class StickAngles {

    //4 cu 3 bat HIGH Alpha
    double HIGH_X_Alpha = 23.0;
    double HIGH_Y_Alpha = 0;
    //4 cu 3 bat HIGH Alpha

    //con rosu Alpha
    double Y_AlphaRed =-12;
    double X_AlphaRed = 68;
    //con rosu Alpha

    //con albastru Alpha
    double Y_AlphaBlue = 12;
    double X_AlphaBlue = 68;
    //con albastru Alpha

    //1 cu 3 bat HIGH Beta
    double HIGH_X_Beta = -23.0;
    double HIGH_Y_Beta = 0;
    //1 cu 3 bat HIGH Beta

    //con rosu Beta
    double Y_BetaRed =-12;
    double X_BetaRed = -68;
    //con rosu Beta

    //con albastru Beta
    double Y_BetaBlue = 12;
    double X_BetaBlue = -68;
    //con albastru Beta

    //high albastru
    double X_BlueHigh = 0;
    double Y_BlueHigh=23.0;
    //high albastru

    //high rosu
    double X_RedHigh = 0;
    double Y_RedHigh=-23.0;
    //high rosu


    //TODO Verificat veridicitate unghi pe celelalte cazuri

    public double AlphaBlueAngleCons(double X_ROBOT, double Y_ROBOT)
    {
        return  -Math.atan((Y_AlphaBlue - Y_ROBOT)/(X_AlphaBlue - X_ROBOT));
    }

    public double AlphaBlueAngleHigh(double X_ROBOT, double Y_ROBOT)
    {
        return Math.atan((HIGH_Y_Alpha - Y_ROBOT)/(HIGH_X_Alpha - X_ROBOT));
    }

    public double AlphaRedAngleCons(double X_ROBOT, double Y_ROBOT)
    {
        return Math.atan((Y_AlphaRed - Y_ROBOT)/(X_AlphaRed - X_ROBOT));
    }

    public double AlphaRedAngleHigh(double X_ROBOT, double Y_ROBOT)
    {
        return Math.atan((HIGH_Y_Alpha - Y_ROBOT)/(HIGH_X_Alpha - X_ROBOT));
    }

    public double BetaRedAngleCons(double X_ROBOT, double Y_ROBOT)
    {
        return -(Math.atan((Y_AlphaRed - Y_ROBOT)/(X_AlphaRed - X_ROBOT)) + Math.PI);
    }

    public double BetaRedAngleHigh(double X_ROBOT, double Y_ROBOT)
    {
        return (Math.atan((HIGH_Y_Beta - Y_ROBOT)/(HIGH_X_Beta - X_ROBOT)) + Math.PI);
    }

    public double BetaBlueAngleHigh(double X_ROBOT, double Y_ROBOT)
    {
        return Math.atan((HIGH_Y_Beta - Y_ROBOT)/(HIGH_X_Beta - X_ROBOT));
    }

    public double BetaBlueAngleCons(double X_ROBOT, double Y_ROBOT)
    {
        return  Math.atan((Y_AlphaBlue - Y_ROBOT)/(X_AlphaBlue - X_ROBOT));
    }

    //Unghi pentru betele de langa substation:

    public double HighBlueAngle(double X_ROBOT, double Y_ROBOT)
    {
        return Math.atan((Y_BlueHigh - Y_ROBOT)/(X_BlueHigh-X_ROBOT) + Math.PI);
    }

    public double HighRedAngle(double X_ROBOT, double Y_ROBOT)
    {
        return Math.atan((Y_RedHigh - Y_ROBOT)/(X_RedHigh-X_ROBOT));
    }

}
