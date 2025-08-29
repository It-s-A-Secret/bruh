package org.firstinspires.ftc.teamcode.other;

import static org.firstinspires.ftc.teamcode.other.Globals.manualArm;
import static org.firstinspires.ftc.teamcode.other.Globals.manualSlides;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;

public abstract class AutoBase extends Robot{

    ElapsedTime autoTimer = new ElapsedTime();

    public void initialize() {
        super.initialize();

        //reset encoders
//        armSubsystem.resetSlideEncoder();

        //schedule(new IntakeCommand(intakeSubsystem, IntakeC
        //command.Claw.OPEN, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber));
//        manualArm = false;
//        manualSlides = false;

//        new InstantCommand(() -> armSubsystem.setArm(90)).schedule(true);
//        claw.setPosition(clawClose);


        //turn on auto drive
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));
    }

    @Override
    public void run(){
        super.run();

    }


}
