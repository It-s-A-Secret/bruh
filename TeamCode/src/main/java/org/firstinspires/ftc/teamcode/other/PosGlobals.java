package org.firstinspires.ftc.teamcode.other;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

@Config
public class PosGlobals {
    //starting pos
    public static final double startRightX = (3 + 7/16) + (14.74/2)-1.5;//the 3&7/16 is the length a sample (bro didn't do double division :skull)
    public static final double startLeftX = -startRightX;
//    public static final double startRightX = 14.74/2 + .5;//the 3&7/16 is the length a sample
//    public static final double startLeftX = -(2*(3 + 7/16) + 14.74/2);
    public static final double startLeftY = -70.9 + (8.18898)-1.25;
    public static final double startRightY = startLeftY;
    public static Pose2d startingPosRight = new Pose2d(startRightX, startRightY, Rotation2d.fromDegrees(0));
    public static Pose2d startingPosTest = new Pose2d(63, -76, Rotation2d.fromDegrees(0));
    public static Pose2d moveOdoTestX = new Pose2d(73, -76, Rotation2d.fromDegrees(0));
    public static Pose2d moveOdoTestY = new Pose2d(73, -66, Rotation2d.fromDegrees(0));
    public static Pose2d moveOdoTestRot = new Pose2d(73, -66, Rotation2d.fromDegrees(90));





    public static Pose2d startingPosLeft = new Pose2d(startLeftX, startLeftY, Rotation2d.fromDegrees(0));
    public static Pose2d startingRedBack = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    public static Pose2d startingRedFront = new Pose2d(118, 156, Rotation2d.fromDegrees(130));

    public static Pose2d redShootFront = new Pose2d(-16.7, 36.6, Rotation2d.fromDegrees(-46));
    public static Pose2d redShootFrontStraight = new Pose2d(19, 64, Rotation2d.fromDegrees(135));



    public static Pose2d redShootBack = new Pose2d(-17, 36, Rotation2d.fromDegrees(-22));
    public static Pose2d redFirstRowReady = new Pose2d(22, -108, Rotation2d.fromDegrees(-90));
    public static Pose2d redFirstRowReadyTwo = new Pose2d(-9, 86, Rotation2d.fromDegrees(-90));

    public static Pose2d redFirstRowIntake = new Pose2d(121, -108, Rotation2d.fromDegrees(-90));
    public static Pose2d redFirstRowIntakeTwo = new Pose2d(113, 88, Rotation2d.fromDegrees(-90));

    public static Pose2d redSecondRowReady = new Pose2d(22, -33, Rotation2d.fromDegrees(-90));
    public static Pose2d redSecondRowReadyCheckpoint = new Pose2d(50, -33, Rotation2d.fromDegrees(-90));

    public static Pose2d redSecondRowReadyTwo = new Pose2d(-9, 156, Rotation2d.fromDegrees(-90));

    public static Pose2d redSecondRowIntake = new Pose2d(123, -33, Rotation2d.fromDegrees(-90));
    public static Pose2d redSecondRowIntakeTwo = new Pose2d(113, 156, Rotation2d.fromDegrees(-90));


    public static Pose2d redThirdRowReady = new Pose2d(25.6, 33, Rotation2d.fromDegrees(-90));
    public static Pose2d redThirdRowIntake = new Pose2d(105, 33, Rotation2d.fromDegrees(-90));


    public static Pose2d startingBlueBack = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static Pose2d blueShootBack = new Pose2d(17, 38, Rotation2d.fromDegrees(23));

    public static Pose2d startingBlueFront = new Pose2d(-118, 156, Rotation2d.fromDegrees(-130));

    public static Pose2d blueShootFront = new Pose2d(9, 14, Rotation2d.fromDegrees(45));


    public static Pose2d blueFirstRowReady = new Pose2d(-9, -108, Rotation2d.fromDegrees(90));
    public static Pose2d blueFirstRowReadyTwo = new Pose2d(9, 86, Rotation2d.fromDegrees(90));

    public static Pose2d blueFirstRowIntake = new Pose2d(-120, -108, Rotation2d.fromDegrees(90));
    public static Pose2d blueFirstRowIntakeTwo = new Pose2d(-113, 88, Rotation2d.fromDegrees(90));

    public static Pose2d blueSecondRowReady = new Pose2d(-9, -44, Rotation2d.fromDegrees(90));
    public static Pose2d blueSecondRowReadyTwo = new Pose2d(9, 156, Rotation2d.fromDegrees(90));

    public static Pose2d blueSecondRowIntake = new Pose2d(-120, -44, Rotation2d.fromDegrees(90));
    public static Pose2d blueSecondRowIntakeTwo = new Pose2d(-113, 156, Rotation2d.fromDegrees(90));


    public static Pose2d blueThirdRowReady = new Pose2d(-9, 26, Rotation2d.fromDegrees(90));
    public static Pose2d blueThirdRowIntake = new Pose2d(-95, 26, Rotation2d.fromDegrees(90));





    public static Pose2d startingPosLeft2 = new Pose2d(startLeftX - (3+7/16) - 23.75, startLeftY, Rotation2d.fromDegrees(0));


    //high chamber
    public static Pose2d highChamberLeft = new Pose2d(-5, -32.4, Rotation2d.fromDegrees(0));
    public static Pose2d firstHighChamberRight = new Pose2d(1, -31.5+1.0, Rotation2d.fromDegrees(0));
    public static Pose2d highChamberCheckpoint = new Pose2d(-4, -42.0, Rotation2d.fromDegrees(15));

    public static Pose2d highChamberRight = new Pose2d(7, -33.76, Rotation2d.fromDegrees(12));
    public static Pose2d highChamberSpecMech = new Pose2d(6, -30+0.5+0.5, new Rotation2d());
    public static Pose2d highChamberSafeScore = new Pose2d(6, -32, Rotation2d.fromDegrees(180));
    public static Pose2d highChamberSafeScoreCheckpoint = new Pose2d(4, -26, Rotation2d.fromDegrees(180));


    public static Pose2d highChamberSpecMechCheckpoint = new Pose2d(8, -41, new Rotation2d());



    public static Pose2d highChamberFastCheckpoint = new Pose2d(35, -58.5+2+4+2, Rotation2d.fromDegrees(20));

    public static Pose2d highChamberFast = new Pose2d(16, -45, Rotation2d.fromDegrees(20));
    public static Pose2d firstHighChamberFast = new Pose2d(16, -45.5, Rotation2d.fromDegrees(20));




    //baskets
    public static Pose2d leftBasketPose = new Pose2d(-55, -55, Rotation2d.fromDegrees(-45));
    public static Pose2d leftBasketPose2 = new Pose2d(-56.5, -55, Rotation2d.fromDegrees(-45));

    //spikemarks
    public static Pose2d leftSideRightSpike = new Pose2d(-46, -39.75, Rotation2d.fromDegrees(0));
    public static Pose2d leftSideMidSpike = new Pose2d(-56, leftSideRightSpike.getY(), Rotation2d.fromDegrees(0));
    public static Pose2d leftSideLeftSpike = new Pose2d(-59.3, -36.85, Rotation2d.fromDegrees(35));

    public static Pose2d rightSideLeftSpike = new Pose2d(32, -37, Rotation2d.fromDegrees(-37));
    public static Pose2d rightSideMiddleSpike = new Pose2d(44, -36, Rotation2d.fromDegrees(-37));
    public static Pose2d rightSideRightSpike = new Pose2d(54, -36, Rotation2d.fromDegrees(-37));

    public static Pose2d rightSideLeftSpikeFlip = new Pose2d(51.25, -53.5, Rotation2d.fromDegrees(0));
     public static Pose2d rightSideLeftSpikePush = new Pose2d(51.25-5, -53.5, Rotation2d.fromDegrees(0));

    public static Pose2d rightSideMiddleSpikeFlip = new Pose2d(60.5, -53.5, Rotation2d.fromDegrees(0));
    public static Pose2d rightSideRightSpikeFlip = new Pose2d(63, -48.25, Rotation2d.fromDegrees(-20));


    //observation zone pickup
    public static double obsZoneX = 0;
    public static double obsZoneY = 0;
    public static double obsZoneHeading = 0;

    //wallPickUp
    public static Pose2d firstWallPickUp = new Pose2d(62, -60+4-1, Rotation2d.fromDegrees(0));

    public static Pose2d wallPickUp = new Pose2d(37, -61, Rotation2d.fromDegrees(0));

    public static Pose2d firstWallPickUpSafe = new Pose2d(62, -60+3, Rotation2d.fromDegrees(180));

    public static Pose2d wallPickUpSafe = new Pose2d(37, -61, Rotation2d.fromDegrees(180));
    public static Pose2d wallPickUpSafeCheckpoint = new Pose2d(37-3, -61+21, Rotation2d.fromDegrees(180));

    public static Pose2d wallPickUpFastCheckpoint = new Pose2d(38-5, -58.5+7, Rotation2d.fromDegrees(0));



    public static Pose2d specMechPickUp = new Pose2d(44, -61.5, Rotation2d.fromDegrees(0));
    public static Pose2d specMechPickUpCheckpoint = new Pose2d(44-5-5, -61.5+20, Rotation2d.fromDegrees(0));



    //parking
    public static Pose2d leftAutoPark = new Pose2d(-24, -7.09, Rotation2d.fromDegrees(-90));
}
