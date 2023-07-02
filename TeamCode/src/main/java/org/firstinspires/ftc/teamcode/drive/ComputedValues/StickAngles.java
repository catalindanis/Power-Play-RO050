package org.firstinspires.ftc.teamcode.drive.ComputedValues;

import org.opencv.core.Mat;

public class StickAngles {

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
