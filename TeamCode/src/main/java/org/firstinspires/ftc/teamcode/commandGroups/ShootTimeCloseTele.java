package org.firstinspires.ftc.teamcode.commandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.WaitForRPMCommand;
import org.firstinspires.ftc.teamcode.subSystems.hIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.shooterSubsystem;

public class ShootTimeCloseTele extends SequentialCommandGroup{
    public ShootTimeCloseTele(shooterSubsystem shooterSubsystem, hIntakeSubsystem hIntakeSubsystem, int time, double targetRpm){
        addCommands(
                new InstantCommand(() -> shooterSubsystem.setTargetRPM(targetRpm)),
//                new InstantCommand(()-> hIntakeSubsystem.stopperStop()),
                new WaitForRPMCommand(shooterSubsystem, targetRpm, 25).withTimeout(1500),
//                new WaitCommand(    1000),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),

                new InstantCommand(() -> hIntakeSubsystem.stopperIn()),
                new WaitCommand(1000),
                new InstantCommand(()-> hIntakeSubsystem.stopperStop()),
                new InstantCommand(()-> hIntakeSubsystem.intakeOff()),
                new InstantCommand(()-> hIntakeSubsystem.stopperOff()),

                new InstantCommand(() -> shooterSubsystem.stop())
        );
    }
}
