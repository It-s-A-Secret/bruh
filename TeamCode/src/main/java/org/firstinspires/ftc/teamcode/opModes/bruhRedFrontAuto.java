package org.firstinspires.ftc.teamcode.opModes;




import static org.firstinspires.ftc.teamcode.other.PosGlobals.redFirstRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redFirstRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redSecondRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redSecondRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redShootBack;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redShootFront;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redThirdRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redThirdRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingPosRight;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingRedBack;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingRedFront;

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


@Autonomous(name="bruhRedFront")
public class bruhRedFrontAuto extends Robot {

    @Override
    public void initialize(){
        super.initialize();


        //turn on auto drive
        driveSubsystem.setStartingPos(startingRedFront);
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));




        schedule(new SequentialCommandGroup(
               // new InstantCommand(() -> driveSubsystem.setStartingPos(startingRedFront)),
                //wait
                new WaitCommand(100),

                //hold pos
                new InstantCommand(() -> driveSubsystem.driveToPoint(startingRedFront)),
                new InstantCommand(() -> hIntakeSubsystem.gateOpen()),

                //go to far shoot zone
//                new DriveToPointCommand(driveSubsystem, new Pose2d(-25, 25, Rotation2d.fromDegrees(130)), 5, 2),
                new DriveToPointCommand(driveSubsystem, redShootFront, 5, 2),

                new WaitCommand(500),
                //shoot
                new ShootTime(shooterSubsystem,hIntakeSubsystem,0,2325),
                new InstantCommand(() -> hIntakeSubsystem.intakeReverse()),

                new WaitCommand(500),
                //getting first row
                new DriveToPointCommand(driveSubsystem, redThirdRowReady, 5, 5),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),
                new InstantCommand(() -> hIntakeSubsystem.gateClose()),

                new DriveToPointCommand(driveSubsystem, redThirdRowIntake, 5, 5).withTimeout(2000),
                new WaitCommand(1000),
                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),
                new InstantCommand(() -> hIntakeSubsystem.gateOpen()),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new WaitCommand(500),
                            new InstantCommand(() -> shooterSubsystem.setTargetRPM(2325))
                        ),
                //shooting first row
                    new DriveToPointCommand(driveSubsystem, redShootFront, 5, 2)
                ),
                new WaitCommand(500),
                new ShootTime(shooterSubsystem,hIntakeSubsystem,0,2325),
                new WaitCommand(500),
                new InstantCommand(() -> hIntakeSubsystem.intakeReverse()),
                //intake second row
                new DriveToPointCommand(driveSubsystem, redSecondRowReady, 5, 5),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),
                new InstantCommand(() -> hIntakeSubsystem.gateClose()),

                new DriveToPointCommand(driveSubsystem, redSecondRowIntake, 5, 5).withTimeout(2000),
                new WaitCommand(1000),

                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),
                new InstantCommand(() -> hIntakeSubsystem.gateOpen()),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(2325))
                        ),
                        //shooting first row
                        new DriveToPointCommand(driveSubsystem, redShootFront, 5, 2)
                ),
                new WaitCommand(500),
                new ShootTime(shooterSubsystem,hIntakeSubsystem,0,2325),
                new WaitCommand(500),
                new DriveToPointCommand(driveSubsystem, new Pose2d(15, -5, Rotation2d.fromDegrees(-45)), 5, 3)

//                new DriveToPointCommand(driveSubsystem, new Pose2d(7, 123, Rotation2d.fromDegrees(-25)), 5, 3)

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
