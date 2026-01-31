package org.firstinspires.ftc.teamcode.commandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.WaitForRPMCommand;
import org.firstinspires.ftc.teamcode.subSystems.hIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.shooterSubsystem;

public class ShootTimeAuto extends SequentialCommandGroup{
    public ShootTimeAuto(shooterSubsystem shooterSubsystem, hIntakeSubsystem hIntakeSubsystem, int time, double targetRpm){
        addCommands(
                new InstantCommand(()-> shooterSubsystem.setHoodFarAuto()),

                new InstantCommand(() -> shooterSubsystem.setTargetRPM(targetRpm)),
                new InstantCommand(()-> hIntakeSubsystem.gateOpen()),
        new WaitForRPMCommand(shooterSubsystem, targetRpm, 25).withTimeout(2000),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),

                new WaitCommand(200),



//                new WaitCommand(500),
                new WaitForRPMCommand(shooterSubsystem, targetRpm, 25).withTimeout(750),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),



                new WaitCommand(200),
                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),


//                new WaitCommand(500),
                new WaitForRPMCommand(shooterSubsystem, targetRpm, 25).withTimeout(500),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),

                new WaitCommand(500),

                new InstantCommand(()-> hIntakeSubsystem.intakeOff()),

                new InstantCommand(() -> shooterSubsystem.setTargetRPM(0)),
                new InstantCommand(() -> shooterSubsystem.stop()),
                new InstantCommand(()-> hIntakeSubsystem.gateClose())
        );
    }
}
