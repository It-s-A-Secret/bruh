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
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedList;

@Config
public class shooterSubsystem extends SubsystemBase {

    private DcMotor shooter, shooter2;
    private MotorGroup arm;
    private Servo endStop;
    private AnalogInput armEncoder;
    private Telemetry telemetry;



    //nautilus



    //last command store
    Command currentCommand;
    Command lastCommand;

    //constructor
    public shooterSubsystem(DcMotor shooter, DcMotor shooter2, Telemetry telemetry) {
        this.shooter = shooter;
        this.shooter2 = shooter2;
        this.telemetry = telemetry;




    }

    public void shoot(double power){
        shooter.setPower(power);
        shooter2.setPower(power);
    }


    public void stop(){
        shooter.setPower(0);
        shooter2.setPower(0);
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
        //read





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
