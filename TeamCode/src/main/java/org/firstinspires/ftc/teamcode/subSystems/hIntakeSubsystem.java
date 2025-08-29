package org.firstinspires.ftc.teamcode.subSystems;

//import static org.firstinspires.ftc.teamcode.other.Globals.armFoldX;
//import static org.firstinspires.ftc.teamcode.other.Globals.armFoldY;
//import static org.firstinspires.ftc.teamcode.other.Globals.manualArm;
import static org.firstinspires.ftc.teamcode.other.Globals.manualSlides;
import static org.firstinspires.ftc.teamcode.other.Globals.slideInX;
import static org.firstinspires.ftc.teamcode.other.Robot.voltageCompensation;

import android.util.Log;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedList;
import java.util.function.DoubleSupplier;

@Config
public class hIntakeSubsystem extends SubsystemBase {

    private DcMotor slideL, slideR;
    private MotorGroup arm;
    private Servo endStop;
    private AnalogInput armEncoder;
    private Telemetry telemetry;

//
//    //arm PIDF
//    public static double kParm = 0.17, kIarm = 0, kDarm = 0.01, kFarm = 1, kGarm = 1; //kF is gain scheduling, kG is gravity ff
//    public static double armWeakKP = 0.01;
//    public static double armAngleOffset = -256;
//
//    public static double armMinAngle = 5;
//    public static double armMaxAngle = 93;
//
//    public static double armNautilusMinAngle = 31.0; //min on the high end
//    public static double armNautilusMaxAngle = 14-0.7; //max on the low end
//
////    public static double armNautilusMaxAngle = 10; //max on the low end
//
//
//    public static double climbingArmP = .03;
//    private double armPowerCap = 1;

    private double slidePowerCap = 1;

    private double ff;
//    private PIDController armController;
//    public static double setArmTargetAngle = 0;
//    private double armPower;
//    private double rawAngle;
//    private double correctedAngle = 0;
    private InterpLUT slideKgLut = new InterpLUT();
    private InterpLUT slideKfLut = new InterpLUT();


    //slide pidf
    //IMPORTANT, slideKP needs to be changed in VisionToSampleInterpolte as well
    public static double slideKP = 0.8, slideKI = 0.0, slideKD = 0.0, slideKF = 0.07;
    private PIDController slideController;
    private final double ticksPerIn = 837/20.8; //one tick is about .04" which is about 1mm. This means that we have about 1mm of precision on the slides 738/30.5 / 1.3529
    private int slideTicks = 1;
    private double slidePower = 1;
    private double slideExtention = 9;
    public static double slideWristOffset = 9  ; //(in)

    public static double slideOutExtension = 0;
    public static double slideRetractMin = slideWristOffset+0.2 + slideOutExtension;
    public static final double slideRetractMax = 40.5;
    public static double setSlideTarget = slideRetractMin;
    private double slideError = 0;

    //arm coordinates
    private double slideTargetIn;

    private double targetX = slideInX;
//    private double targetY = armFoldY;

    //manualArm

    private double slideManualPower;



    //intakeFromWall
    private boolean wallActive;

    //slide velocity
    private LinkedList<TimeStampedPosition> positionHistory = new LinkedList<>();
    private static final long VELOCITY_TIME_FRAME_MS = 100; // Time frame in milliseconds

    //nautilus



    //last command store
    Command currentCommand;
    Command lastCommand;

    //constructor
    public hIntakeSubsystem(MotorGroup arm, DcMotor slideL, DcMotor slideR, Servo endStop, AnalogInput armEncoder, Telemetry telemetry) {
        this.arm = arm;
        this.slideL = slideL;
        this.slideR = slideR;
        this.endStop = endStop;
        this.armEncoder = armEncoder;
        this.telemetry = telemetry;
        wallActive = false;

//TODO: tune the slide gain scheduling
        //Adding each val with a key
        slideKgLut.add(-999999, .1);
        slideKgLut.add(9, .1);
        slideKgLut.add(20, 0.28);
        slideKgLut.add(30, .45);
        slideKgLut.add(40, .73);
        slideKgLut.add(9999999, .73);
        //generating final equation

        slideKgLut.createLUT();

        slideKfLut.add(-999999, 0.135);
        slideKfLut.add(7, 0.12);
        slideKfLut.add(23.9, .25);
        slideKfLut.add(41, .35);
        slideKfLut.add(99999999, .35);

        slideKfLut.createLUT();

        //nautilus lut

        //-0.02909x+0.7582
//        nautilus.add(-999999,0.81);
//        nautilus.add(-2.1,0.81);
//        nautilus.add(0.7,0.74);
//        nautilus.add(2.4,0.69);
//        nautilus.add(4.5,0.63);
//        nautilus.add(6.9,0.56);
//        nautilus.add(9.3,0.49);
//        nautilus.add(11.7,0.42);
//        nautilus.add(14.1,0.35);
//        nautilus.add(99999,0.35);
//
//
////        nautilus.add(17.1,0.26);
////        nautilus.add(19.2,0.2);
////        nautilus.add(21.4,0.13);
////        nautilus.add(999999,0.13);
//        nautilus.createLUT();
//
//        nautilusDown();
    }

    public void manualArm(double armPower, double slidePower){
//        armManualPower = -armPower;
        slideManualPower = slidePower;
    }

    public void manualSlides(double slidePower){
        slideManualPower = slidePower;
    }

//    public void setArm(double targetAngle) {
//        //this is kind of sus
//        if(getSlideTarget()<11){targetAngle = MathUtils.clamp(targetAngle, armMinAngle, armMaxAngle);}
//        //allow for intake sub to go lower when the arm is extended out
//        else{targetAngle = MathUtils.clamp(targetAngle, 0, armMaxAngle);}
//
//        if(targetAngle<armNautilusMaxAngle && !manualNautilus){
//
//            setNautilus(nautilus.get(targetAngle+0.8)-0.34);
//            nautilusUp=false;
//        }
//
//
//        setArmTargetAngle = targetAngle;
//    }

    public void setSlide(double targetInches) {
        targetInches = MathUtils.clamp(targetInches, slideRetractMin, slideRetractMax);
        setSlideTarget = targetInches;
    }

//    public void setArmCoordinates(double x, double y){
//        targetX = x;
//        targetY = y;
//
//        //inverse kinematics
//        slideTargetIn = Math.sqrt(Math.pow(x, 2) + Math.pow(y - armHeight, 2));
//        armTargetAngle = Math.toDegrees(Math.atan2((y - armHeight), x));
//
//        //write
//        setSlide(slideTargetIn);
//        //do one before the other so we can clamp arm based on slides
//        setArm(armTargetAngle);
//    }
//
//    public void setArmY(double y){
//        targetY = y;
//        setArmCoordinates(targetX, y);
//    }
//
//    public void setArmX(double x){
//        targetX = x;
//        setArmCoordinates(x, targetY);
//    }
//
//    public void setArmX(DoubleSupplier x){
//        targetX = x.getAsDouble();
//        setArmCoordinates(targetX, targetY);
//    }


//    public void setArmP(double p){
//        armController.setP(p);
//    }
//
//    public void setArmPowerCap(double cap){
//        armPowerCap = cap;
//    }

    public void setSlidePowerCap(double cap){
        slidePowerCap=cap;
    }

    //forward kinematics
//    public double getCurrentX(){
//        return slideExtention * Math.cos(Math.toRadians(correctedAngle));
//    }
//    public double getCurrentY(){
//        return slideExtention * Math.sin(Math.toRadians(correctedAngle)) + armHeight;
//    }
//
//    public double getArmAngle(){
//        return correctedAngle;
//    }

    public double getSlideExtention(){
        return slideExtention;
    }

    public double getSlideError(){
        return slideError;
    }

    public void setSlideP(double p){
        slideKP = p;
    }

    public double getSlideX(){
        return slideExtention;
    }

//    public double getArmTarget(){
//        return setArmTargetAngle;
//    }

    public double getSlideTarget(){
        return setSlideTarget;
    }

    public double getTargetX(){
        return targetX;
    }

//    public double getTargetY(){
//        return targetY;
//    }

    //return intakeWall state
    public boolean getWallState(){
        return wallActive;
    }
    //toggle wall state
    public void toggleWallState(){
        wallActive = !wallActive;
    }


    //slide velocity
    public double getSlideVelocity() {

        long currentTime = System.currentTimeMillis();

        // Add the new position with its timestamp
        positionHistory.add(new TimeStampedPosition(getSlideExtention(), currentTime));

        // Remove old entries beyond the time frame
        while (!positionHistory.isEmpty() &&
                currentTime - positionHistory.getFirst().getTimestamp() > VELOCITY_TIME_FRAME_MS) {
            positionHistory.removeFirst();
        }


        if (positionHistory.size() < 2) {
            // Not enough data to calculate velocity
            return 0.0;
        }

        // Get the oldest and newest positions in the time frame
        TimeStampedPosition oldestPosition = positionHistory.getFirst();
        TimeStampedPosition newestPosition = positionHistory.getLast();

        // Calculate velocity: Δposition / Δtime
        double deltaPosition = newestPosition.getPosition() - oldestPosition.getPosition();
        double deltaTime = (newestPosition.getTimestamp() - oldestPosition.getTimestamp()) / 1000.0; // Convert ms to seconds

        // Avoid divide-by-zero errors
        if (deltaTime == 0) {
            return 0.0;
        }

        return deltaPosition / deltaTime; // Velocity in INCHES per second
    }

    public class TimeStampedPosition {
        private final double position; // For the arm, could be degrees or extension length
        private final long timestamp;  // Timestamp in milliseconds

        public TimeStampedPosition(double position, long timestamp) {
            this.position = position;
            this.timestamp = timestamp;
        }

        public double getPosition() {
            return position;
        }

        public long getTimestamp() {
            return timestamp;
        }

        @Override
        public String toString() {
            return "TimeStampedPosition{" +
                    "position=" + position +
                    ", timestamp=" + timestamp +
                    '}';
        }
    }

    public Command getLastCommand(){
        //redundant null checking
        if(lastCommand==null){
            return new InstantCommand();
        }
        return lastCommand;
    }

    @Override
    public void periodic() {
        //read
        slideTicks = slideR.getCurrentPosition();
//        rawAngle = 360 - (armEncoder.getVoltage()/3.3 * 360);
//        correctedAngle = rawAngle + armAngleOffset;

        //calculate slide extension
        slideExtention = (slideTicks/ticksPerIn + slideWristOffset) + 0.931 + slideOutExtension;


        //arm pid
//        armController = new PIDController(kParm * (kFarm * slideKfLut.get(slideExtention)), kIarm, kDarm);
        //feed forward
//        ff = kGarm * (Math.cos(Math.toRadians(correctedAngle)) * slideKgLut.get(slideExtention));
//        armPower = (voltageCompensation * (Math.sqrt(Math.abs(armController.calculate(correctedAngle, setArmTargetAngle))) * Math.signum(setArmTargetAngle - correctedAngle))) + ff;
        /*telemetry.addData("ff", ff);
        telemetry.addData("cos", Math.cos(Math.toRadians(angle)));;
        telemetry.addData("targetAngle", targetAngle);
        telemetry.addData("error", targetAngle - angle);*/
        slideError = setSlideTarget - slideExtention;
        telemetry.addData("slideError", slideError);

        //slide pid
        slideController = new PIDController(slideKP, slideKI, slideKD);
        slidePower = (voltageCompensation * slideController.calculate(slideExtention, setSlideTarget));
        //telemetry.addData("targetIN", targetInches);
        //telemetry.addData("slideTicks", slideTicks);

        //arm manual
//        if(manualArm){
//            arm.set(armManualPower);
//        } else {
//            //pid power
//            if(Math.abs(armPower) > armPowerCap){
//                armPower = armPower * armPowerCap;
//                arm.set(armPower);
//            } else {
//                arm.set(armPower);
//            }
//        }

        if(manualSlides){
            slideL.setPower(slideManualPower);
            slideR.setPower(slideManualPower);
        }
        else{

            if(Math.abs(slidePower)>slidePowerCap){
                slidePower = slidePowerCap * Math.signum(slidePower);
            }
            slideL.setPower(slidePower);
            slideR.setPower(slidePower);
        }

//        telemetry.addData("armAngle", correctedAngle);
//        telemetry.addData("armTarget", setArmTargetAngle);
//        telemetry.addData("armPower", armPower);
//        telemetry.addData("armManual", armManualPower);
//        telemetry.addData("armKP", armController.getP());
//        telemetry.addData("armError", setArmTargetAngle - correctedAngle);

        telemetry.addData("slidePower", slidePower);
        telemetry.addData("slideExtention", slideExtention);

        telemetry.addData("targetArmX", targetX);
////        telemetry.addData("targetArmY", targetY);
//        telemetry.addData("xArmPos", getCurrentX());
//        telemetry.addData("yArmPos", getCurrentY());
        telemetry.addData("slideVelocity", getSlideVelocity());




        //last command
        currentCommand = CommandScheduler.getInstance().requiring(this);

        if (currentCommand != null && currentCommand != lastCommand) {
            lastCommand = currentCommand;
        }
        //Redundent null checking
        if(lastCommand==null){
            lastCommand=new InstantCommand();
        }
        telemetry.addData("armSubsystemLastCommand", lastCommand != null ? lastCommand.getName() : "None");

        Log.i("slideLPower", String.valueOf(slideL.getPower()));
        Log.i("slideRPower", String.valueOf(slideR.getPower()));


    }

    public void setPowerZero(){
        arm.set(0);
    }

    public void setSlidePower(double power){
        slideL.setPower(power);
        slideR.setPower(power);
    }

    public void resetSlideEncoder(){
        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }




}
