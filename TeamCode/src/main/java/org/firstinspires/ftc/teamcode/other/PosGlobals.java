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
    public static Pose2d startingPosTest = new Pose2d(10, 10, Rotation2d.fromDegrees(0));
    public static Pose2d moveOdoTestX = new Pose2d(73, -76, Rotation2d.fromDegrees(0));
    public static Pose2d moveOdoTestY = new Pose2d(73, -66, Rotation2d.fromDegrees(0));
    public static Pose2d moveOdoTestRot = new Pose2d(73, -66, Rotation2d.fromDegrees(90));





    public static Pose2d BLUEFIELD = new Pose2d(55.433, -51.5, Rotation2d.fromDegrees(0));

    public static Pose2d REDFIELD = new Pose2d(55.433, 51.5, Rotation2d.fromDegrees(0));

    public static Pose2d startingPosLeft = new Pose2d(startLeftX, startLeftY, Rotation2d.fromDegrees(0));
    public static Pose2d startingRedBack = new Pose2d(178.75, 13, Rotation2d.fromDegrees(0));

    public static Pose2d startingRedFront = new Pose2d(-123.9, 84., Rotation2d.fromDegrees(-48));

    public static Pose2d redShootFront = new Pose2d(-8.5, 8.7, Rotation2d.fromDegrees(-44));


    public static Pose2d redShootBack = new Pose2d(141.22, 8, Rotation2d.fromDegrees(0));
    public static Pose2d redBackFinish = new Pose2d(141.22, 17, Rotation2d.fromDegrees(0));
    public static Pose2d redFirstRowReady = new Pose2d(82.8, 27.75, Rotation2d.fromDegrees(-90));
    public static Pose2d redFirstRowIntake = new Pose2d(84.85, 110.5, Rotation2d.fromDegrees(-90));

    public static Pose2d redSecondRowReady = new Pose2d(27.3, 35.9, Rotation2d.fromDegrees(-90));
    public static Pose2d redSecondRowReadyCheckpoint = new Pose2d(28.5, 116.5, Rotation2d.fromDegrees(-90));


    public static Pose2d redSecondRowIntake = new Pose2d(26.5, 110.5, Rotation2d.fromDegrees(-90));


    public static Pose2d redThirdRowReady = new Pose2d(-32.7, 14, Rotation2d.fromDegrees(-90));
    public static Pose2d redThirdRowIntake = new Pose2d(-35.4, 92.5, Rotation2d.fromDegrees(-90));


    public static Pose2d redOpenGateCheckpoint = new Pose2d(-19.2, 80, Rotation2d.fromDegrees(-90));

    public static Pose2d redOpenGate = new Pose2d(-19.2, 97.8, Rotation2d.fromDegrees(-90));
    public static Pose2d redOpenGateIntakeReady = new Pose2d(15.25, 97.5, Rotation2d.fromDegrees(-90));
    public static Pose2d redOpenGateIntakeReadyCheckpoint = new Pose2d(15.25, 80, Rotation2d.fromDegrees(-90));
    public static Pose2d redOpenGateIntake = new Pose2d(35.67, 109.6, Rotation2d.fromDegrees(-53.5));

    public static Pose2d redTurtleIntakeReady = new Pose2d(146.6, 1.105, Rotation2d.fromDegrees(-90));
    public static Pose2d redTurtleIntake = new Pose2d(151.7, 110.3, Rotation2d.fromDegrees(-90));
    public static Pose2d redTurtleIntakeReady2 = new Pose2d(118, 78.5, Rotation2d.fromDegrees(-90));
    public static Pose2d redTurtleIntake2 = new Pose2d(122, 114, Rotation2d.fromDegrees(-90));









    public static Pose2d startingBlueBack = new Pose2d(178.75, -13, Rotation2d.fromDegrees(0));

    public static Pose2d startingBlueFront = new Pose2d(-123.9, -84., Rotation2d.fromDegrees(48));

    public static Pose2d blueShootFront = new Pose2d(-12.5, -8.7, Rotation2d.fromDegrees(44));


    public static Pose2d blueShootBack = new Pose2d(141.22, -8, Rotation2d.fromDegrees(0));
    public static Pose2d blueFirstRowReady = new Pose2d(84.8, -27.75, Rotation2d.fromDegrees(90));
    public static Pose2d blueFirstRowIntake = new Pose2d(84.85, -110.5, Rotation2d.fromDegrees(90));

    public static Pose2d blueSecondRowReady = new Pose2d(34.3, -20.9, Rotation2d.fromDegrees(90));
    public static Pose2d blueSecondRowReadyCheckpoint = new Pose2d(34.5, -116.5, Rotation2d.fromDegrees(90));


    public static Pose2d blueSecondRowIntake = new Pose2d(34.5, -110.5, Rotation2d.fromDegrees(90));


    public static Pose2d blueThirdRowReady = new Pose2d(-32.7, -14, Rotation2d.fromDegrees(90));
    public static Pose2d blueThirdRowIntake = new Pose2d(-32.7, -90, Rotation2d.fromDegrees(90));


    public static Pose2d blueOpenGateCheckpoint = new Pose2d(-19.2, -70, Rotation2d.fromDegrees(90));

    public static Pose2d blueOpenGate = new Pose2d(-19.2, -97.8, Rotation2d.fromDegrees(90));
    public static Pose2d blueOpenGateIntakeReady = new Pose2d(15.25, -97.5, Rotation2d.fromDegrees(90));
    public static Pose2d blueOpenGateIntakeReadyCheckpoint = new Pose2d(15.25, -80, Rotation2d.fromDegrees(90));
    public static Pose2d blueOpenGateIntake = new Pose2d(35.67, -109.6, Rotation2d.fromDegrees(53.5));

    public static Pose2d blueTurtleIntakeReady = new Pose2d(146.6, -1.105, Rotation2d.fromDegrees(90));
    public static Pose2d blueTurtleIntake = new Pose2d(151.7, -110.3, Rotation2d.fromDegrees(90));
    public static Pose2d blueTurtleIntakeReady2 = new Pose2d(118, -78.5, Rotation2d.fromDegrees(90));
    public static Pose2d blueTurtleIntake2 = new Pose2d(122, -114, Rotation2d.fromDegrees(90));
    public static Pose2d blueBackFinish = new Pose2d(141.22, -17, Rotation2d.fromDegrees(0));






}
