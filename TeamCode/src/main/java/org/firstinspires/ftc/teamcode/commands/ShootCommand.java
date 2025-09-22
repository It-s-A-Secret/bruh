package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subSystems.shooterSubsystem;

public class ShootCommand extends CommandBase {

    shooterSubsystem shooterSubsystem;

    public ShootCommand(shooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        //addRequirements(driveSubsystem);
    }

    @Override
    public void execute(){
        shooterSubsystem.shoot();
    }

}
