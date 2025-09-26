package org.firstinspires.ftc.teamcode.other;

//import static org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem.hardStoppedHighPitch;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subSystems.hIntakeSubsystem;

//import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

@Config
public class Globals {
//intake subsystem
    //intaking
    public static int pitchWhenIntake = -95;
    public static int rollWhenIntake = -0;
    public static int pitchLastLeftAuto = 60;
    public static int rollLastLeftAuto = -190;
    //intake from the wall
    public static int pitchIntakeWall = -60;
    public static int rollIntakeWall = 180;
    // Right left specimens
    public static int pitchRightAutoSpecimen = pitchIntakeWall;
    public static int rollRightAutoSpecimen = -190;

    //claw poses
    public static double clawOpen = 0.35;
    public static double clawExtraOpen = .275;
    public static double clawHalfClose = .5;
    public static double clawClose = .575;


    //scoring
    //basket
    public static int pitchWhenBasket = 40;
    public static int rollWhenBasket = 0;

    public static double pitchWhenHighChamber = 0;
    public static double rollWhenHighChamber = -20;

    //autoLeft
    public static double autoPitchFrontHighChamber = 30;
    public static double rollFrontHighChamber = 0;

    public static double pitchFrontHighChamber = 300;


    //teleop high chamber
    public static double pitchTeleopHighChamber = 0;
    public static double rollTeleopHighChamber = 230;


    //autoright
    public static double pitchFrontRightHighChamber = 0;
    public static double rollFrontRightHighChamber = rollTeleopHighChamber;
    public static double pitchPlaceFrontHighRightChamber = 90;
    public static double rollPlaceFrontHighRightChamber = 260;


    //home
    public static int rollWhenArmBack = -150;
    public static int rollWhenArmHome = 200;



    //secondaryArm poses

    //wall intake
//    public static int secondaryPitchWallIntake = hardStoppedHighPitch;//over, but it guarantees that even when we skip we will still be able to pickup
    public static int secondaryYawWallIntake = 0;

    //high chamber
    public static int secondaryPitchHighChamber = 0;
    public static int secondaryYawHighChamber = 0;




//arm subsystem
    //arm home
    public static double armHomeX = 7.5;
    public static double armHomeY = 7;
    //arm fold
//    public static double armFoldY = 6.0;
    //arm back - not really back anymmore, just the spec scoring position
    public static double armBackX = 13.0;
    public static double armBackY = 20;

    public static double armFastX = 25+0.86;
    public static double armFastY = 22.75;
    //arm basket
    public static double armHighBasketX = -2;
    public static double  armHighBasketY = 42.5;
    public static double  armLowBasketY = 30;
    //arm when front high chamber
    public static double armFrontHighChamberX = 13.0;
    public static double armFrontHighChamberY = 20.5;
    public static double autoArmFrontHighChamberY = 21;
    //arm when high chamber
    public static double armRightHighChamberX = -1;
    public static double armRightHighChamberY = 28.0;
    public static double armHighChamberX = -1;
    public static double armHighChamberY = armRightHighChamberY;

    //arm when intaking from sub
    public static double armAutoSpikeX = 23;
    public static double armReadySubIntakeX = 28;
    public static double armSubIntakeY = 3;
    public static double armReadySubIntakeY = 9.0;
    //arm when close (distance) intake
    public static double armCloseIntakeX = 15;
    public static double armCloseIntakeY = 8;
    //arm when intaking form the wall
    public static double armIntakeWallX = 2;
    public static double armIntakeWallY = 14.2;
    // arm when intaking for AutoRight
    public static double armAutoRightX = 16.6;
    public static double armAutoRightY = 1.3;
    public static double armAutoPushY = -1.5;
    public static double armAutoReadyPushY = 5;

    //manual arm boolean
    public static boolean manualArm = false;
    public static boolean manualSlides = false;
    //arm auto park
    public static double armParkLeftAutoX = 13;
    public static double armParkLeftAutoY = 23;

    //endstop
    public static double endstopUp = .463;
    public static double endstopDown = 1;

    //defensePad
    public static double defensePadDown = 0;
    public static double defensePadUp = 0;

    //dt pto
    public static double dtPTOEngaged = 0.7;
    public static double dtPTODisengaged = 0.4;

    //arm when climbing
    //climbing to first rung
    public static double armAngleReady = 36.08;
    public static double armPositionToClimbX = 16.9;
    public static double armPositionToClimbY = 24.8;
    //climbing to second rung
    public static double armAngleToSecondRungX = 3.5;
    public static double armAngleToSecondRungY = 14.6;
    public static double armExtendPastSecondRungX = 6;//12;
    public static double armExtendPastSecondRungY = 27.5;
    public static double armMoveToSecondRungX = 15;
    public static double armMoveToSecondRungY = 27;
    public static double armPositionRobotToEdgeOfFirstRungX = -8;
    public static double armPositionRobotToEdgeOfFirstRungY = 18;
    //completely retracting when climbing
    public static double armCompleteRetractX = 8;
    public static double armCompleteRetractY = 3.5;

//drive to point
    public static double translationKP = 0.007;
    public static final double translationKPprecise = 0.015;
    public static final double translationKPfast = 0.02*1.2;

    public static double translationKI = 0.0;
    public static double translationKD = 0.2;
//    public static double translationKD = 0.001;
    public static double translationKF = -0.06; //static friction coefficient, overall robot vector not individual module so may not work as expected

    public static double translationKR = .5; //KR is the constant for the root of the pid
    public static double translationMaxVel = 1; //in inches per second

    public static double headingKP = 0.005 * (11.5/9.0) / 1.5 * 2;
    public static double headingKI = 0.0;
    public static double headingKD = 0.0002;
    public static double headingKR = .5;
    public static double headingMaxVel = 1; //in degrees per second
    public static double lateralMutliplier = 1.5;

    public static double testX = 0.0;
    public static double testY = 0.0;
    public static double testHeading = 0;

}
