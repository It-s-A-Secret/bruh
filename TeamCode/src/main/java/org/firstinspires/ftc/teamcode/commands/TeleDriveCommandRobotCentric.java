package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class TeleDriveCommandRobotCentric extends CommandBase {
    private DriveSubsystem driveSubsystem;

    private DoubleSupplier strafe, forward, turn;
    private boolean arcTanZones, isBlue;
    private int arcTanAngleRange;
    private GamepadEx driver;

    public TeleDriveCommandRobotCentric(DriveSubsystem driveSubsystem, GamepadEx driver, boolean arcTanZones, int arcTanAngleRange, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn) {
        this.driveSubsystem = driveSubsystem;
        this.driver = driver;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        this.arcTanZones = arcTanZones;
        this.arcTanAngleRange = arcTanAngleRange;
        this.isBlue = isBlue;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute(){
        driveSubsystem.teleDrive(driver, arcTanZones, arcTanAngleRange, strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());
    }

}
