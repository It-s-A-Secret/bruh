package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.shooterSubsystem;

public class WaitForRPMCommand extends CommandBase {
    private shooterSubsystem shooterSubsystem;
    private double targetRPM;
    private double tolerance;
    private ElapsedTime timer = new ElapsedTime();

    public WaitForRPMCommand(shooterSubsystem shooterSubsystem, double targetRPM, double tolerance) {
        this.shooterSubsystem = shooterSubsystem;
        this.targetRPM = targetRPM;
        this.tolerance = tolerance;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        timer.reset();
    }

    @Override
    public void execute(){
        shooterSubsystem.setTargetRPM(targetRPM);
    }

    @Override
    public boolean isFinished(){
        if(((Math.abs(shooterSubsystem.getCurrentRPM()) > targetRPM -tolerance) && shooterSubsystem.getCurrentRPM() > targetRPM -tolerance) && timer.milliseconds() > 50){
            return true;
        } else {
            return false;
        }
    }

}
