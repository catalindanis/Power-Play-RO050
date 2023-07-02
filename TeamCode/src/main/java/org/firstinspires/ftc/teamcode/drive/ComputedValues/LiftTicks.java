package org.firstinspires.ftc.teamcode.drive.ComputedValues;

public class LiftTicks {

    //4 cu 3 bat HIGH dreapta
    double HIGH_X_Right = 0;
    double HIGH_Y_Right = 23.0;
    //4 cu 3 bat HIGH dreapta

    // SUNT DOAR 2 HIGH-uri PE CARE SCORAM IN AUTONOMIE

    //1 cu 3 bat HIGH stanga
    double HIGH_X_Left = 0;
    double HIGH_Y_Left = -23;
    //1 cu 3 bat HIGH stanga

    //inaltime bat

    double h_stick = 85; //CM

    double er = 0; //eroare in caz caplm

    //TODO Determinare eventuala eroare

    //inaltime bat

    public int CMtoTicks(double CM)
    {
        double d = CM*30.0169493;
        return (int)d;
    }

    public int RightRedTicks(double X_ROBOT, double Y_ROBOT)
    {
        double b = Math.sqrt((HIGH_X_Right - X_ROBOT)*(HIGH_X_Right - X_ROBOT) + (HIGH_Y_Right - Y_ROBOT)*(HIGH_Y_Right - Y_ROBOT));

        return -CMtoTicks(Math.sqrt(b*b + (h_stick+er)*(h_stick+er)));

        //TODO Sistem siguranta in caz ca se deopaseste nr tick-urilor maxime si eventuala miscare a robotului in cazul in care e nevoie pentru a se ajunge la pozitia corecta ( se face din nou o functie in care se calculeaza X-ul robotolui in functie de nr tick-urilor care nu ajung - cel mai apropiat X)
    }

}
