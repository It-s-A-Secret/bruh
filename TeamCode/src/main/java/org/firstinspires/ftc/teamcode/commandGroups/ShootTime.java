package org.firstinspires.ftc.teamcode.commandGroups;
import static org.firstinspires.ftc.teamcode.other.Globals.armSubIntakeY;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.WaitForRPMCommand;
import org.firstinspires.ftc.teamcode.subSystems.hIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.shooterSubsystem;

public class ShootTime extends SequentialCommandGroup{
    public ShootTime(shooterSubsystem shooterSubsystem, hIntakeSubsystem hIntakeSubsystem, int time, double targetRpm){
        addCommands(
                new InstantCommand(() -> shooterSubsystem.setTargetRPM(targetRpm)),
                new InstantCommand(()-> hIntakeSubsystem.stopperStop()),
                new WaitForRPMCommand(shooterSubsystem, targetRpm, 25).withTimeout(1500),
//                new WaitCommand(    1000),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),
                new WaitCommand(200),

                new InstantCommand(() -> hIntakeSubsystem.stopperIn()),
                new WaitCommand(200),
                new InstantCommand(()-> hIntakeSubsystem.stopperStop()),


//                new WaitCommand(500),
                new WaitForRPMCommand(shooterSubsystem, targetRpm, 25).withTimeout(500),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),
                new WaitCommand(200),

                new InstantCommand(() -> hIntakeSubsystem.stopperIn()),
                new WaitCommand(200),
                new InstantCommand(()-> hIntakeSubsystem.stopperStop()),
                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),

//                new WaitCommand(500),
                new WaitForRPMCommand(shooterSubsystem, targetRpm, 25).withTimeout(500),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),
                new WaitCommand(200),

                new InstantCommand(() -> hIntakeSubsystem.stopperIn()),
                new WaitCommand(200),
                new InstantCommand(()-> hIntakeSubsystem.stopperStop()),
                new InstantCommand(()-> hIntakeSubsystem.intakeOff()),
                new InstantCommand(()-> hIntakeSubsystem.stopperOff()),

                new InstantCommand(() -> shooterSubsystem.stop())
        );
    }
}
