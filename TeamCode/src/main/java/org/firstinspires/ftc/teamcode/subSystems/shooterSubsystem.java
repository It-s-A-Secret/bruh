package org.firstinspires.ftc.teamcode.subSystems;

//import static org.firstinspires.ftc.teamcode.other.Globals.armFoldX;
//import static org.firstinspires.ftc.teamcode.other.Globals.armFoldY;
//import static org.firstinspires.ftc.teamcode.other.Globals.manualArm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class shooterSubsystem extends SubsystemBase {

    private MotorEx shooter, shooter2;
    private MotorGroup arm;
    private Servo endStop;
    private AnalogInput armEncoder;
    private Telemetry telemetry;

    public static double kPWheel = 0.003, kIWheel = 0.00, kDWheel = 0.000025;
    public static double kFWheel = 0.000265;
    private PIDController velocityController;
    private double targetRPM = 0;
    private double currentRpm = 0;
    private boolean wheelOn = true;
    private double rpmError =0;

    private static final int TICKS_PER_REV = 28;

    InterpLUT rpmLUT = new InterpLUT();




    //last command store
    Command currentCommand;
    Command lastCommand;

    //constructor
    public shooterSubsystem(MotorEx shooter, MotorEx shooter2, Telemetry telemetry) {
        this.shooter = shooter;
        this.shooter2 = shooter2;
        this.telemetry = telemetry;
        shooter.resetEncoder();
        shooter2.resetEncoder();


        velocityController = new PIDController(kPWheel, kIWheel, kDWheel);



    }


    public void setTargetRPM(double rpm){
        wheelOn = true;
        targetRPM = rpm;
    }
    public double getCurrentRPM(){
        return currentRpm = shooter.getVelocity() * 60.0 / TICKS_PER_REV;
    }
    public void stop() {
        wheelOn = false;
    }

    public double getRPMError(){
        return rpmError;
    }




    public class TimeStampedPosition {
        private final double position; // For the arm, could be degrees or extension length
        private final long timestamp;  // Timestamp in milliseconds

        public TimeStampedPosition(double position, long timestamp) {
            this.position = position;
            this.timestamp = timestamp;
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
        velocityController.setPID(kPWheel, kIWheel, kDWheel);
        //read
        double currentRPM = getCurrentRPM();
        double ff = kFWheel * targetRPM;

        double pid = velocityController.calculate(currentRPM, targetRPM);

        double power = ff + pid;
        rpmError = currentRPM-targetRPM;

        power = Math.max(-1, Math.min(1, power));
        if(wheelOn == true) {
            shooter.set(power);
            shooter2.set(power);
        }else{
            shooter.set(0);
            shooter2.set(0);
            targetRPM = 0;
        }

        telemetry.addData("TargetRPM", targetRPM);
        telemetry.addData("CurrentRPM", currentRPM);
        telemetry.addData("Power", power);






        //last command
        currentCommand = CommandScheduler.getInstance().requiring(this);

        if (currentCommand != null && currentCommand != lastCommand) {
            lastCommand = currentCommand;
        }
        //Redundent null checking
        if(lastCommand==null){
            lastCommand=new InstantCommand();
        }
        telemetry.addData("shooterSubsystemLastCommand", lastCommand != null ? lastCommand.getName() : "None");




    }






}
