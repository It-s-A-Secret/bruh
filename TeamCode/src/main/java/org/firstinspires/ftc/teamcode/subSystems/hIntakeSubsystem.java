package org.firstinspires.ftc.teamcode.subSystems;



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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.other.Globals;

import java.util.LinkedList;
import java.util.function.DoubleSupplier;

@Config
public class hIntakeSubsystem extends SubsystemBase {


    private Telemetry telemetry;
    private DcMotor intake, stopper;
    private Servo gate;
    private CRServo rightTransfer, leftTransfer;



    //nautilus



    //last command store
    Command currentCommand;
    Command lastCommand;

    //constructor
    public hIntakeSubsystem(DcMotor intake, Servo gate, Telemetry telemetry) {

        this.telemetry = telemetry;
        this.stopper = stopper;
        this.intake = intake;
        this.gate = gate;
        this.rightTransfer = rightTransfer;
        this.leftTransfer=leftTransfer;


    }
    public void intakeOn(){
        intake.setPower(-1);
    }
    public void intakeOff(){
        intake.setPower(0);
    }
    public void intakeReverse(){
        intake.setPower(1);
    }




    public void gateClose(){
        gate.setPosition(Globals.gateClose);
    }
    public void gateOpen(){
        gate.setPosition(Globals.gateOpen);
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
