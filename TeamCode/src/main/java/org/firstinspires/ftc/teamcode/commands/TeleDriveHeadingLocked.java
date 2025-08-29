package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

public class TeleDriveHeadingLocked extends CommandBase {

    Rotation2d desiredHeading;
    GamepadEx m_driver;

    DriveSubsystem driveSubsystem;
    public TeleDriveHeadingLocked(DriveSubsystem driveSubsystem, GamepadEx m_driver){
        this.driveSubsystem = driveSubsystem;
        this.m_driver = m_driver;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){
        desiredHeading = driveSubsystem.getPos().getRotation();
    }

    @Override
    public void execute(){
        driveSubsystem.teleDriveHeadingLocked(()->m_driver.getButton(GamepadKeys.Button.LEFT_BUMPER), true, 10, m_driver.getLeftX(), m_driver.getLeftY(), desiredHeading);
    }
}
