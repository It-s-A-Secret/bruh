package org.firstinspires.ftc.teamcode.opModes;




import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueFirstRowIntakeTwo;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueFirstRowReadyTwo;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueSecondRowIntakeTwo;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueSecondRowReadyTwo;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueShootBack;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redFirstRowIntakeTwo;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redFirstRowReadyTwo;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redSecondRowIntakeTwo;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redSecondRowReadyTwo;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redShootBack;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingBlueBack;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingRedBack;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.ShootTime;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;


@Autonomous(name="bruhBlueBack")
public class bruhBlueBackAuto extends Robot {

    @Override
    public void initialize(){
        super.initialize();


        //turn on auto drive
        driveSubsystem.setStartingPos(startingBlueBack);
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));




        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingBlueBack)),
                new InstantCommand(() -> hIntakeSubsystem.gateOpen()),

                //go to far shoot zone
                new DriveToPointCommand(driveSubsystem, blueShootBack, 5, 2),
                new WaitCommand(500),
                //shoot
                new ShootTime(shooterSubsystem,hIntakeSubsystem,0,3900),
                new WaitCommand(500),
                //getting first row
              //  new DriveToPointCommand(driveSubsystem, redFirstRowReady, 5, 5),
                new DriveToPointCommand(driveSubsystem, blueFirstRowReadyTwo, 5, 5),


                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),
                new InstantCommand(() -> hIntakeSubsystem.gateClose()),

                //new DriveToPointCommand(driveSubsystem, redFirstRowIntake, 5, 5),
                new DriveToPointCommand(driveSubsystem, blueFirstRowIntakeTwo, 5, 5),

                new WaitCommand(1000),
                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),
                new InstantCommand(() -> hIntakeSubsystem.gateOpen()),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new WaitCommand(500),
                            new InstantCommand(() -> shooterSubsystem.setTargetRPM(3900))
                        ),
                //shooting first row
                    new DriveToPointCommand(driveSubsystem, blueShootBack, 5, 2)
                ),
                new WaitCommand(500),
                new ShootTime(shooterSubsystem,hIntakeSubsystem,0,3900),
                new WaitCommand(500),
                //intake second row
                //new DriveToPointCommand(driveSubsystem, redSecondRowReady, 5, 5),
                new DriveToPointCommand(driveSubsystem, blueSecondRowReadyTwo, 5, 5),

                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),
                new InstantCommand(() -> hIntakeSubsystem.gateClose()),

                //new DriveToPointCommand(driveSubsystem, redSecondRowIntake, 5, 5),
                new DriveToPointCommand(driveSubsystem, blueSecondRowIntakeTwo, 5, 5),

                new WaitCommand(1000),

                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),
                new InstantCommand(() -> hIntakeSubsystem.gateOpen()),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(3900))
                        ),
                        //shooting first row
                        new DriveToPointCommand(driveSubsystem, blueShootBack, 5, 2)
                ),
                new WaitCommand(500),
                new ShootTime(shooterSubsystem,hIntakeSubsystem,0,3900),
                new WaitCommand(500),
                new DriveToPointCommand(driveSubsystem, new Pose2d(17, 42, Rotation2d.fromDegrees(22)), 5, 3)

                //intake third row
//                new DriveToPointCommand(driveSubsystem, redThirdRowReady, 5, 5),
//                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),
////                new InstantCommand(() -> hIntakeSubsystem.gateClose()),
//
//                new DriveToPointCommand(driveSubsystem, redThirdRowIntake, 5, 5),
//                new WaitCommand(1000),
//
//                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),
//                new InstantCommand(() -> hIntakeSubsystem.gateOpen())
//
//                new ParallelCommandGroup(
//                        new SequentialCommandGroup(
//                                new WaitCommand(500),
//                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(3900))
//                        ),
//                        //shooting first row
//                        new DriveToPointCommand(driveSubsystem, redShootBack, 5, 5)
//                ),
//                new WaitCommand(500),
//                new ShootTime(shooterSubsystem,hIntakeSubsystem,0,3900),
//                new WaitCommand(500)











                ));



    }


}
