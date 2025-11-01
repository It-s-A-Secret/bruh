package org.firstinspires.ftc.teamcode.subSystems;

//import static org.firstinspires.ftc.teamcode.other.Globals.armFoldX;
//import static org.firstinspires.ftc.teamcode.other.Globals.armFoldY;
//import static org.firstinspires.ftc.teamcode.other.Globals.manualArm;
import static org.firstinspires.ftc.teamcode.other.Globals.manualSlides;
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


    private Telemetry telemetry;
    private DcMotor intake, stopper;
    private Servo gate;



    //nautilus



    //last command store
    Command currentCommand;
    Command lastCommand;

    //constructor
    public hIntakeSubsystem(DcMotor intake, DcMotor stopper, Servo gate, Telemetry telemetry) {

        this.telemetry = telemetry;
        this.stopper = stopper;
        this.intake = intake;
        this.gate = gate;


//TODO: tune the slide gain scheduling
        //Adding each val with a key


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
    public void intakeOn(){
        intake.setPower(1);
    }
    public void intakeOff(){
        intake.setPower(0);
    }
    public void intakeReverse(){
        intake.setPower(-1);
    }

    public void stopperStop(){
        stopper.setPower(1);
    }
    public void stopperIn(){
        stopper.setPower(-1);
    }
    public void stopperOff(){
        stopper.setPower(0);
    }

    public void gateClose(){
        gate.setPosition(0.25);
    }
    public void gateOpen(){
        gate.setPosition(0.6);
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


        //slide pid


//        telemetry.addData("armAngle", correctedAngle);
//        telemetry.addData("armTarget", setArmTargetAngle);
//        telemetry.addData("armPower", armPower);
//        telemetry.addData("armManual", armManualPower);
//        telemetry.addData("armKP", armController.getP());
//        telemetry.addData("armError", setArmTargetAngle - correctedAngle);


////        telemetry.addData("targetArmY", targetY);
//        telemetry.addData("xArmPos", getCurrentX());
//        telemetry.addData("yArmPos", getCurrentY());





        //last command
        currentCommand = CommandScheduler.getInstance().requiring(this);

        if (currentCommand != null && currentCommand != lastCommand) {
            lastCommand = currentCommand;
        }
        //Redundent null checking
        if(lastCommand==null){
            lastCommand=new InstantCommand();
        }
        telemetry.addData("hIntakeSubsystemLastCommand", lastCommand != null ? lastCommand.getName() : "None");



    }






}
