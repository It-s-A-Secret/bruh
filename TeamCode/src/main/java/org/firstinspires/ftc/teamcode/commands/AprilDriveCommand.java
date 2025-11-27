package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.LimelightSubsystem;

import java.util.Optional;
import java.util.function.DoubleSupplier;

public class AprilDriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private LimelightSubsystem limelightSubsystem;

    private DoubleSupplier strafe, forward, turn;
    private double angle;
    private boolean arcTanZones;
    private int arcTanAngleRange;
    private GamepadEx driver;

    public AprilDriveCommand(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem, GamepadEx driver, boolean arcTanZones, int arcTanAngleRange, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn) {
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.driver = driver;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        this.arcTanZones = arcTanZones;
        this.arcTanAngleRange = arcTanAngleRange;

        addRequirements(driveSubsystem, limelightSubsystem);
    }
    @Override
    public void initialize(){

        limelightSubsystem.initializeCamera();
    }
    @Override
    public void execute(){
        angle = limelightSubsystem.getX(driveSubsystem.getPos().getRotation().getDegrees());
        driveSubsystem.teleDrive(driver, arcTanZones, arcTanAngleRange, strafe.getAsDouble(), forward.getAsDouble(), driveSubsystem.headingAlign(angle));
    }

}
