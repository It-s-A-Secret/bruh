package org.firstinspires.ftc.teamcode.opModes;




import static org.firstinspires.ftc.teamcode.other.Globals.farRPM;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueBackFinish;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redBackFinish;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redFirstRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redFirstRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redSecondRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redSecondRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redShootBack;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redThirdRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redThirdRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redTurtleIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redTurtleIntake2;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redTurtleIntakeReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redTurtleIntakeReady2;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingRedBack;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.ShootTime;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;


@Autonomous(name="bruhRedBackHumanPlayer")
public class bruhRedBackTurtlewalkers extends Robot {

    @Override
    public void initialize(){
        super.initialize();

        shooterSubsystem.teamRed();
        driveSubsystem.setStartingPos(startingRedBack);
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));
        shooterSubsystem.turretOff();
        shooterSubsystem.setHoodFar();





                schedule(new SequentialCommandGroup(
                // new InstantCommand(() -> driveSubsystem.setStartingPos(startingRedFront)),
                //wait
                new WaitCommand(100),

                //hold pos
                new InstantCommand(() -> driveSubsystem.driveToPoint(startingRedBack)),
                new InstantCommand(() -> shooterSubsystem.turretOn()),


                //go to far shoot zone
//                new DriveToPointCommand(driveSubsystem, new Pose2d(-25, 25, Rotation2d.fromDegrees(130)), 5, 2),
//                new DriveToPointCommand(driveSubsystem, redShootFrontStraight, 5, 2),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(farRPM))
                        ),
                        //shooting first row
                        new DriveToPointCommand(driveSubsystem, redShootBack, 5, 2)
                ),
                //shoot
                new ShootTime(shooterSubsystem,hIntakeSubsystem,0,farRPM),
                new InstantCommand(() -> hIntakeSubsystem.intakeReverse()),




                //getting first row
                new ParallelCommandGroup(
                        new InstantCommand(() -> hIntakeSubsystem.gateClose()),
                        new InstantCommand(() -> shooterSubsystem.turretOff()),
                        new DriveToPointCommand(driveSubsystem, redTurtleIntakeReady, 5, 5)
                ),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),




                new DriveToPointCommand(driveSubsystem, redTurtleIntake, 5, 5).withTimeout(1500),
                        new WaitCommand(550),

//                new WaitCommand(300),
                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),


                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(farRPM)),
                                new WaitCommand(500),
                                new InstantCommand(() -> shooterSubsystem.turretOn())
                                ),
                        new DriveToPointCommand(driveSubsystem, redShootBack, 5, 2)
                        //shooting first row


                ),
//                new WaitCommand(300),

                new ShootTime(shooterSubsystem,hIntakeSubsystem,0,farRPM),
                new InstantCommand(() -> hIntakeSubsystem.intakeReverse()),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> hIntakeSubsystem.gateClose()),
                                new InstantCommand(() -> shooterSubsystem.turretOff()),
                                new DriveToPointCommand(driveSubsystem, redTurtleIntakeReady, 5, 5)
                        ),
                        new InstantCommand(() -> hIntakeSubsystem.intakeOn()),




                        new DriveToPointCommand(driveSubsystem, redTurtleIntake, 5, 5).withTimeout(1500),
                        new WaitCommand(550),



//                new WaitCommand(300),
                        new InstantCommand(() -> hIntakeSubsystem.intakeOff()),


                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> shooterSubsystem.setTargetRPM(farRPM)),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> shooterSubsystem.turretOn())
                                ),
                                new DriveToPointCommand(driveSubsystem, redShootBack, 5, 2)
                                //shooting first row


                        ),
                        new ShootTime(shooterSubsystem,hIntakeSubsystem,0,farRPM),
//                new WaitCommand(300),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> hIntakeSubsystem.gateClose()),
                                new InstantCommand(() -> shooterSubsystem.turretOff()),
                                new DriveToPointCommand(driveSubsystem, redTurtleIntakeReady, 5, 5)
                        ),
                        new InstantCommand(() -> hIntakeSubsystem.intakeOn()),




                        new DriveToPointCommand(driveSubsystem, redTurtleIntake, 5, 5).withTimeout(1500),
                        new WaitCommand(550),


//                new WaitCommand(300),
                        new InstantCommand(() -> hIntakeSubsystem.intakeOff()),


                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> shooterSubsystem.setTargetRPM(farRPM)),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> shooterSubsystem.turretOn())
                                ),
                                new DriveToPointCommand(driveSubsystem, redShootBack, 5, 2)
                                //shooting first row


                        ),
//                new WaitCommand(300),

                        new ShootTime(shooterSubsystem,hIntakeSubsystem,0,farRPM),
//                new WaitCommand(300),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> hIntakeSubsystem.gateClose()),
                                new InstantCommand(() -> shooterSubsystem.turretOff()),
                                new DriveToPointCommand(driveSubsystem, redTurtleIntakeReady, 5, 5)
                        ),
                        new InstantCommand(() -> hIntakeSubsystem.intakeOn()),




                        new DriveToPointCommand(driveSubsystem, redTurtleIntake, 5, 5).withTimeout(1500),


//                new WaitCommand(300),
                        new InstantCommand(() -> hIntakeSubsystem.intakeOff()),


                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> shooterSubsystem.setTargetRPM(farRPM)),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> shooterSubsystem.turretOn())
                                ),
                                new DriveToPointCommand(driveSubsystem, redShootBack, 5, 2)
                                //shooting first row


                        ),
//                new WaitCommand(300),

                        new ShootTime(shooterSubsystem,hIntakeSubsystem,0,farRPM),
                        new InstantCommand(() -> hIntakeSubsystem.intakeOff()),
                        new InstantCommand(()-> shooterSubsystem.turretOff()),
                        new DriveToPointCommand(driveSubsystem, redBackFinish, 5, 5)














                ));



    }


}
