package org.firstinspires.ftc.teamcode.subSystems;


import static org.firstinspires.ftc.teamcode.other.Globals.headingKD;
import static org.firstinspires.ftc.teamcode.other.Globals.headingKI;
import static org.firstinspires.ftc.teamcode.other.Globals.headingKP;
import static org.firstinspires.ftc.teamcode.other.Globals.lateralMutliplier;
import static org.firstinspires.ftc.teamcode.other.Globals.translationKD;
import static org.firstinspires.ftc.teamcode.other.Globals.translationKF;
import static org.firstinspires.ftc.teamcode.other.Globals.translationKI;
import static org.firstinspires.ftc.teamcode.other.Globals.translationKP;
import static org.firstinspires.ftc.teamcode.other.Globals.translationKPfast;
import static org.firstinspires.ftc.teamcode.other.Globals.translationKPprecise;
import static org.firstinspires.ftc.teamcode.other.Globals.translationKR;
import static org.firstinspires.ftc.teamcode.other.Robot.voltageCompensation;

import android.util.Log;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.other.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.runner.Drawing;

import java.util.function.BooleanSupplier;

;
@Config
public class DriveSubsystem extends SubsystemBase {

//    private static final Logger log = LoggerFactory.getLogger(DriveSubsystem.class);
    private MotorEx FR, FL, BR, BL;
    private Servo defensePad, ptoServo;

    private double y;
    private double x;
    private double power = 1;

    private Telemetry telemetry;

    //driveToPoint squid
//    private ProfiledPIDController profiledTranslationController, profiledHeadingController;
    private double errorX, errorY, rawErrorHeading, correctedErrorHeading;
    private GoBildaPinpointDriver pinpoint;
    private Pose2d currentPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private double rawVectorMagnitude;
    private double correctedVectorMagnitude;
    private double vectorTheta;
    private double headingCalculation;
    private Pose2d targetPos = new Pose2d(0,0, Rotation2d.fromDegrees(0));
//    private PIDController translationController = new PIDController(translationKP, translationKI, translationKD);
    private PIDController headingController = new PIDController(headingKP, headingKI, headingKD);
    BasicPID translationController = new BasicPID(new PIDCoefficients(translationKP, translationKI, translationKD));


    private double strafeVelocity;
    private double forwardVelocity;
    private double turnVelocity;

    private boolean autoDriveToggle = false;

    private MecanumDrive mecanumDrive;

    public Drive drive;
    public enum Drive {
        FAST,
        SLOW
    }

    ElapsedTime timer = new ElapsedTime();

    double translationalError = 0.0;



//    //constructor for auto
//    public DriveSubsystem(MotorEx FR, MotorEx FL, MotorEx BR, MotorEx BL, MecanumDrive mecanumDrive, Telemetry telemetry, GoBildaPinpointDriver pinpoint) {
//        this.FR = FR;
//        this.FL = FL;
//        this.BR = BR;
//        this.BL = BL;
//        this.mecanumDrive = mecanumDrive;
//        this.telemetry = telemetry;
//        this.pinpoint = pinpoint;
//    }

    //constructor for teleop
    public DriveSubsystem(MotorEx FR, MotorEx FL, MotorEx BR, MotorEx BL, MecanumDrive mecanumDrive, Telemetry telemtry, GoBildaPinpointDriver pinpoint) {
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
        this.mecanumDrive = mecanumDrive;
        this.ptoServo = ptoServo;
        this.telemetry = telemtry;
        this.pinpoint = pinpoint;


    }

    public void stopDrive(){
        mecanumDrive.stop();
    }

    //current arm command
    //public Command currentArmCommand = armSubsystem.getCurrentCommand();

    //drive with arc tan dead zones (teleop)
    public void teleDrive(GamepadEx driver, boolean arcTanZones, int arcTanAngleRange, double strafeSpeed, double forwardSpeed, double turnSpeed) {
        //slow mode
        if (driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            power = .3;
        } else {
            power = 1;
        }


        //arc tan dead zones
        if (arcTanZones) {
            if (Math.toDegrees(Math.atan(y / x)) > 90 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 90 + arcTanAngleRange / 2
            || Math.toDegrees(Math.atan(y / x)) < -90 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) > -90 + arcTanAngleRange / 2) {
                x = 0;

                } else if (Math.toDegrees(Math.atan(y / x)) > 0 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 0 + arcTanAngleRange / 2
            || Math.toDegrees(Math.atan(y / x)) > 180 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 180 + arcTanAngleRange / 2) {
                y = 0;
            }
        }

        //actually moving
        mecanumDrive.driveFieldCentric(strafeSpeed * power, forwardSpeed * power, turnSpeed * power, currentPos.getRotation().getDegrees());

        //read pinpoint
        readPinpoint();
    }

    /**
     * Just for backwards compatibility
     */
    public void teleDrive(BooleanSupplier slowmode, boolean arcTanZones, int arcTanAngleRange, double strafeSpeed, double forwardSpeed, double turnSpeed) {
        //slow mode
        if (slowmode.getAsBoolean()) {
            power = .3;
        } else {
            power = 1;
        }


        //arc tan dead zones
        if (arcTanZones) {
            if (Math.toDegrees(Math.atan(y / x)) > 90 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 90 + arcTanAngleRange / 2
                    || Math.toDegrees(Math.atan(y / x)) < -90 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) > -90 + arcTanAngleRange / 2) {
                x = 0;

            } else if (Math.toDegrees(Math.atan(y / x)) > 0 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 0 + arcTanAngleRange / 2
                    || Math.toDegrees(Math.atan(y / x)) > 180 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 180 + arcTanAngleRange / 2) {
                y = 0;
            }
        }

        //actually moving
        mecanumDrive.driveFieldCentric(strafeSpeed * power, forwardSpeed * power, -turnSpeed * power, currentPos.getRotation().getDegrees());

        //read pinpoint
        readPinpoint();
    }

    public void teleDriveHeadingLocked(BooleanSupplier slowmode, boolean arcTanZones, int arcTanAngleRange, double strafeSpeed, double forwardSpeed, Rotation2d desiredHeading){
        //slow mode
        if (slowmode.getAsBoolean()) {
            power = .3;
        } else {
            power = 1;
        }

        rawErrorHeading = currentPos.getRotation().getDegrees() - desiredHeading.getDegrees();
        rawErrorHeading = rawErrorHeading % 360;

        if (rawErrorHeading < -180) {
            correctedErrorHeading = rawErrorHeading + 360;
        } else if (rawErrorHeading > 180) {
            correctedErrorHeading = rawErrorHeading - 360;
        } else {
            correctedErrorHeading = rawErrorHeading;
        }

        headingCalculation = headingController.calculate(correctedErrorHeading);

        turnVelocity = -Math.sqrt(Math.abs(headingCalculation)) * Math.signum(headingCalculation);


        //arc tan dead zones
        if (arcTanZones) {
            if (Math.toDegrees(Math.atan(y / x)) > 90 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 90 + arcTanAngleRange / 2
                    || Math.toDegrees(Math.atan(y / x)) < -90 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) > -90 + arcTanAngleRange / 2) {
                x = 0;

            } else if (Math.toDegrees(Math.atan(y / x)) > 0 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 0 + arcTanAngleRange / 2
                    || Math.toDegrees(Math.atan(y / x)) > 180 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 180 + arcTanAngleRange / 2) {
                y = 0;
            }
        }

        //actually moving
        mecanumDrive.driveFieldCentric(strafeSpeed * power, forwardSpeed * power, turnVelocity, currentPos.getRotation().getDegrees());

        //read pinpoint
        readPinpoint();
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        mecanumDrive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void driveToPoint(Pose2d targetPos){
        this.targetPos = targetPos;
    }

    public void pidToRotation2d(Rotation2d targetRotation){
        driveToPoint(new Pose2d(
                this.targetPos.getTranslation(),
                targetRotation
        ));
    }

    public void autoDrive(){
        //completely temperary testing - delete if causing problems
        /*targetPos = new Pose2d(testX, testY, Rotation2d.fromDegrees(testHeading));*/

        //read pinpoint
        readPinpoint();

        //pids
//        if(profiledTranslation){
//            profiledTranslationController = new ProfiledPIDController(translationKP, translationKI, translationKD, new TrapezoidProfile.Constraints(1, 1));
//        } else {translationController = new PIDController(translationKP, translationKI, translationKD);}
//
//        if(profiledHeading){
//            profiledHeadingController = new ProfiledPIDController(headingKP, headingKI, headingKD, new TrapezoidProfile.Constraints(1, 1));
//        } else {headingController = new PIDController(headingKP, headingKI, headingKD);
//        translationController.setPID(translationKP, translationKI, translationKD);
        translationController = new BasicPID(new PIDCoefficients(translationKP, translationKI, translationKD));
        headingController.setPID(headingKP, headingKI, headingKD);

        //error calculation
        errorX = currentPos.getX() - targetPos.getX();
        errorY = currentPos.getY() - targetPos.getY();

        //angular difference correction
        rawErrorHeading = currentPos.getRotation().getDegrees() - targetPos.getRotation().getDegrees();
        rawErrorHeading = rawErrorHeading % 360;

        if (rawErrorHeading < -180) {
            correctedErrorHeading = rawErrorHeading + 360;
        } else if (rawErrorHeading > 180) {
            correctedErrorHeading = rawErrorHeading - 360;
        } else {
            correctedErrorHeading = rawErrorHeading;
        }

        //testing
        telemetry.addData("errorX", errorX);
        telemetry.addData("errorY", errorY);
        Log.i("autoError", String.valueOf(errorY));
        telemetry.addData("targetHeading", targetPos.getRotation().getDegrees());
        telemetry.addData("rawErrorHeading", rawErrorHeading);
        telemetry.addData("correctedHeading", correctedErrorHeading);

        //vector calculation
        rawVectorMagnitude = Math.hypot(errorX, errorY);
        //DO NOT USE THIS VARIABLE, this is just for a getERror method
        translationalError = Math.abs(rawVectorMagnitude);
        vectorTheta = Math.toDegrees(Math.atan2(errorY, errorX));

        //pid calculation
        headingCalculation = voltageCompensation * headingController.calculate(correctedErrorHeading);
        correctedVectorMagnitude = voltageCompensation * -Math.pow((Math.abs(translationController.calculate(rawVectorMagnitude,0))) * Math.signum(rawVectorMagnitude), translationKR);

        //Do not do static friction if error is low enough
        if(rawVectorMagnitude>0.015) {
            correctedVectorMagnitude += translationKF;
        }

        //testing
        telemetry.addData("rawVectorMagnitude", rawVectorMagnitude);
        telemetry.addData("correctedVectorMagnitude", correctedVectorMagnitude);
        telemetry.addData("vectorTheta", vectorTheta);

        //breaking vector into speed values + pid
        strafeVelocity = lateralMutliplier * (Math.cos (Math.toRadians(vectorTheta)) * correctedVectorMagnitude);
        forwardVelocity = (Math.sin (Math.toRadians(vectorTheta)) * correctedVectorMagnitude);
        turnVelocity = -Math.sqrt(Math.abs(headingCalculation)) * Math.signum(headingCalculation);

        //testing
        telemetry.addData("strafeSpeed", strafeVelocity);
        telemetry.addData("forwardSpeed", forwardVelocity);
        telemetry.addData("turnSpeed", turnVelocity);

        //actually driving
        mecanumDrive.driveFieldCentric(strafeVelocity, forwardVelocity, turnVelocity, getHeadingInDegrees(currentPos));
    }

    public void readPinpoint() {
        timer.reset();
        pinpoint.update();
        Pose2D tempPos = pinpoint.getPosition();
        if(!(Double.isNaN(tempPos.getX(DistanceUnit.INCH)) || Double.isNaN(tempPos.getY(DistanceUnit.INCH)) || Double.isNaN(tempPos.getHeading(AngleUnit.DEGREES)))){
            currentPos = new Pose2d(-tempPos.getY(DistanceUnit.INCH), tempPos.getX(DistanceUnit.INCH), Rotation2d.fromDegrees(tempPos.getHeading(AngleUnit.DEGREES)));
        }
        telemetry.addData("pinpointRefreshRate", timer.milliseconds());
        Log.i("pinpointRefreshRate", String.valueOf(timer.milliseconds()));
        Log.i("autoErrorpinpointreading?", "yes");
        telemetry.addData("xDTPos", currentPos.getX());
        telemetry.addData("yDTPos", currentPos.getY());
        telemetry.addData("dtHeading", currentPos.getRotation().getDegrees());
    }

    public Pose2d getPos(){
        return currentPos;
    }

    public Pose2d getTargetPos(){
        return targetPos;
    }

    public double getTranslationalError(){
        return rawVectorMagnitude;
    }

    public double getXError(){
        return errorX;
    }

    public double getHeadingError(){
        return correctedErrorHeading;
    }

    public void setStartingPos(Pose2d pos){
        pinpoint.setPosition( new Pose2D(DistanceUnit.INCH, pos.getY(), -pos.getX(), AngleUnit.RADIANS, pos.getHeading()));
    }

    public void resetPinpointIMU(){
        Log.i("resetPinPointIMU", "true");
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, currentPos.getX(), currentPos.getY(), AngleUnit.DEGREES, 0));
    }

    public static double getHeadingInDegrees(Pose2d pose) {
            // Convert the heading from radians to degrees
            double headingInDegrees = pose.getRotation().getDegrees();

            // Normalize the heading to 0-360 degrees
            if (headingInDegrees < 0) {
                headingInDegrees += 360;
            }

            return headingInDegrees;
    }

    public void drawBot(Pose2d pose) {
        //drawing robot on dash
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), new com.acmerobotics.roadrunner.Pose2d(pose.getX(), pose.getY(), pose.getRotation().getRadians() + Math.PI/2));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public double getAutoDriveError(){
        return translationalError;
    }

    public void enablePrecisePID(boolean enable){
        if(enable){
            translationKP=translationKPprecise;
        }
        else{
            translationKP=translationKPfast;
        }
    }


    public void setPower(double power){
        FL.set(power);
        FR.set(power);
        BL.set(power);
        BR.set(power);

    }

    @Override
    public void periodic() {
        if(autoDriveToggle){
            //autoDrive(false, false);
        }


    }

}