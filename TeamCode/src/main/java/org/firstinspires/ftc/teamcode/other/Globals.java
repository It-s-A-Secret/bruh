package org.firstinspires.ftc.teamcode.other;

//import static org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem.hardStoppedHighPitch;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subSystems.hIntakeSubsystem;

//import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

@Config
public class Globals {
//intake subsystem
    //intaking
   public static double closeRPM = 2400;
   public static double farRPM = 3500;

//drive to point
    public static double translationKP = 0.016;
    public static final double translationKPprecise = 0.015;
    public static final double translationKPfast = 0.02*1.2;

    public static double translationKI = 0.0;
    public static double translationKD = 0.3;
//    public static double translationKD = 0.001;
    public static double translationKF = -0.03; //static friction coefficient, overall robot vector not individual module so may not work as expected

    public static double translationKR = .5; //KR is the constant for the root of the pid
    public static double translationMaxVel = 1; //in inches per second

    public static double headingKP = 0.005 * (11.5/9.0) / 1.5 * 2;
    public static double headingKI = 0.0;
    public static double headingKD = 0.0002;
    public static double alignKP = 0.005 * (11.5/9.0) / 1.5 * 2;
    public static double alignKI = 0.0;
    public static double alignKD = 0.0002;

    public static double lateralMutliplier = 1.5;


    public static double hoodClose = 0.1;
    public static double hoodCloseAuto = 0.2;


    public static double hoodFar = 0.9;
    public static double hoodFarBlueAuto = 0.1;


    public static double gateClose = 0.75;

    public static double gateOpen = 0.5;

}
