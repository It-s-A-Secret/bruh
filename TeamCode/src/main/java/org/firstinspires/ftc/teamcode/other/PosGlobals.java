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
    public static Pose2d startingRedBack = new Pose2d(72, 6.16, Rotation2d.fromDegrees(0));

    public static Pose2d startingRedFront = new Pose2d(-49.427, 34.895, Rotation2d.fromDegrees(-49.5));

    public static Pose2d redShootFront = new Pose2d(-10.8, 8.284, Rotation2d.fromDegrees(-46));


    public static Pose2d redShootBack = new Pose2d(60.96, 8.83, Rotation2d.fromDegrees(0));
    public static Pose2d redShootBackCheck = new Pose2d(52, 6.68, Rotation2d.fromDegrees(-90));

    public static Pose2d redBackFinish = new Pose2d(54, 11, Rotation2d.fromDegrees(0));
    public static Pose2d redThirdRowReady = new Pose2d(-12.885, 7.63, Rotation2d.fromDegrees(-90));
    public static Pose2d redThirdRowIntake = new Pose2d(-12.585, 39.27, Rotation2d.fromDegrees(-90));

    public static Pose2d redSecondRowReady = new Pose2d(12.54, 7.63, Rotation2d.fromDegrees(-90));
    public static Pose2d redSecondRowReadyCheckpoint = new Pose2d(28.5, 116.5, Rotation2d.fromDegrees(-90));


    public static Pose2d redSecondRowIntake = new Pose2d(12.54, 44, Rotation2d.fromDegrees(-90));


    public static Pose2d redFirstRowReady = new Pose2d(34.5, 7.63, Rotation2d.fromDegrees(-90));
    public static Pose2d redFirstRowIntake = new Pose2d(34.5, 44, Rotation2d.fromDegrees(-90));


    public static Pose2d redOpenGateCheckpoint = new Pose2d(-4.73, 34.885, Rotation2d.fromDegrees(-90));

    public static Pose2d redOpenGate = new Pose2d(-4.73, 41.1, Rotation2d.fromDegrees(-90));
    public static Pose2d redOpenGateIntakeReady = new Pose2d(5.54, 41.1, Rotation2d.fromDegrees(-90));
    public static Pose2d redOpenGateIntakeReadyCheckpoint = new Pose2d(5.54, 12, Rotation2d.fromDegrees(-90));
    public static Pose2d redOpenGateIntake = new Pose2d(23.46, 48, Rotation2d.fromDegrees(-42));
    public static Pose2d redOpenGateIntakeFinish = new Pose2d(20.46, 48, Rotation2d.fromDegrees(-90));


    public static Pose2d redTurtleIntakeReady = new Pose2d(60, 16.6, Rotation2d.fromDegrees(-90));
    public static Pose2d redTurtleIntake = new Pose2d(60, 45, Rotation2d.fromDegrees(-90));
    public static Pose2d redTurtleIntakeReady2 = new Pose2d(122, 78.5, Rotation2d.fromDegrees(-90));
    public static Pose2d redTurtleIntake2 = new Pose2d(122, 114, Rotation2d.fromDegrees(-90));









    public static Pose2d startingBlueBack = new Pose2d(72, -6.16, Rotation2d.fromDegrees(0));

    public static Pose2d startingBlueFront = new Pose2d(-49.427, -34.895, Rotation2d.fromDegrees(50));

    public static Pose2d blueShootFront = new Pose2d(-10.8, -8.284, Rotation2d.fromDegrees(46));


    public static Pose2d blueShootBack = new Pose2d(60.96, -3.83, Rotation2d.fromDegrees(90));
    public static Pose2d blueShootBackCheck = new Pose2d(52, -6.68, Rotation2d.fromDegrees(90));

    public static Pose2d blueBackFinish = new Pose2d(54, -11, Rotation2d.fromDegrees(0));
    public static Pose2d blueThirdRowReady = new Pose2d(-10.885, -7.63, Rotation2d.fromDegrees(90));
    public static Pose2d blueThirdRowIntake = new Pose2d(-10.585, -39.27, Rotation2d.fromDegrees(90));

    public static Pose2d blueSecondRowReady = new Pose2d(14.54, -7.63, Rotation2d.fromDegrees(90));
    public static Pose2d blueSecondRowReadyCheckpoint = new Pose2d(28.5, -116.5, Rotation2d.fromDegrees(90));


    public static Pose2d blueSecondRowIntake = new Pose2d(14.54, -45.5, Rotation2d.fromDegrees(90));


    public static Pose2d blueFirstRowReady = new Pose2d(37.01, -7.63, Rotation2d.fromDegrees(90));
    public static Pose2d blueFirstRowIntake = new Pose2d(37.01, -45.5, Rotation2d.fromDegrees(90));


    public static Pose2d blueOpenGateCheckpoint = new Pose2d(-2.73, -34.885, Rotation2d.fromDegrees(90));

    public static Pose2d blueOpenGate = new Pose2d(-2.73, -41.1, Rotation2d.fromDegrees(90));
    public static Pose2d blueOpenGateIntakeReady = new Pose2d(7.54, -41.1, Rotation2d.fromDegrees(90));
    public static Pose2d blueOpenGateIntakeReadyCheckpoint = new Pose2d(7.54, -12, Rotation2d.fromDegrees(90));
    public static Pose2d blueOpenGateIntake = new Pose2d(25.46, -48, Rotation2d.fromDegrees(42));
    public static Pose2d blueOpenGateIntakeFinish = new Pose2d(22.46, -48, Rotation2d.fromDegrees(90));


    public static Pose2d blueTurtleIntakeReady = new Pose2d(65, -16.6, Rotation2d.fromDegrees(90));
    public static Pose2d blueTurtleIntake = new Pose2d(65, -46, Rotation2d.fromDegrees(90));


}
