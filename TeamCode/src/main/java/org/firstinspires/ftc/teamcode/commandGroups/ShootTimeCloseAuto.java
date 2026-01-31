package org.firstinspires.ftc.teamcode.commandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.WaitForRPMCommand;
import org.firstinspires.ftc.teamcode.subSystems.hIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.shooterSubsystem;

public class ShootTimeCloseAuto extends SequentialCommandGroup{
    public ShootTimeCloseAuto(shooterSubsystem shooterSubsystem, hIntakeSubsystem hIntakeSubsystem, int time, double targetRpm){
        addCommands(
                new InstantCommand(() -> shooterSubsystem.setTargetRPM(targetRpm)),
                new InstantCommand(()-> hIntakeSubsystem.gateOpen()),
                new InstantCommand(()-> shooterSubsystem.setHoodCloseAuto()),


//                new InstantCommand(()-> hIntakeSubsystem.stopperStop()),
                new WaitForRPMCommand(shooterSubsystem, targetRpm, 25).withTimeout(1000),
//                new WaitCommand(    1000),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),

                new WaitCommand(1000),
                new InstantCommand(()-> hIntakeSubsystem.intakeOff()),


                new InstantCommand(() -> shooterSubsystem.setTargetRPM(0)),
                new InstantCommand(() -> shooterSubsystem.stop()),
                new InstantCommand(()-> hIntakeSubsystem.gateClose())

        );
    }
}
