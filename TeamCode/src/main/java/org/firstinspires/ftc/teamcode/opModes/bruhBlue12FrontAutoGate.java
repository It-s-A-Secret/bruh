package org.firstinspires.ftc.teamcode.opModes;




import static org.firstinspires.ftc.teamcode.other.Globals.closeRPM;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueFirstRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueFirstRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueOpenGate;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueOpenGateCheckpoint;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueSecondRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueSecondRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueShootFront;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueThirdRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueThirdRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redFirstRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redFirstRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redOpenGate;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redOpenGateCheckpoint;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redSecondRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redSecondRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redShootFront;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redThirdRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redThirdRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingBlueFront;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingRedFront;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.ShootTimeCloseTele;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;


@Autonomous(name="bruhBlue12FrontGATED")
public class bruhBlue12FrontAutoGate extends Robot {

    @Override
    public void initialize(){
        super.initialize();


        //turn on auto drive
        shooterSubsystem.teamBlue();
        driveSubsystem.setStartingPos(startingBlueFront);
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));
        shooterSubsystem.turretOff();




        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> shooterSubsystem.turretOn()),
                // new InstantCommand(() -> driveSubsystem.setStartingPos(startingRedFront)),
                //wait
                new WaitCommand(100),

                //hold pos
                new InstantCommand(() -> driveSubsystem.driveToPoint(startingBlueFront)),


                //go to far shoot zone
//                new DriveToPointCommand(driveSubsystem, new Pose2d(-25, 25, Rotation2d.fromDegrees(130)), 5, 2),
//                new DriveToPointCommand(driveSubsystem, redShootFrontStraight, 5, 2),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(2300))
                        ),
                        //shooting first row
                        new DriveToPointCommand(driveSubsystem, blueShootFront, 5, 2)
                ),
                //shoot
                new ShootTimeCloseTele(shooterSubsystem,hIntakeSubsystem,0,2300),
                new InstantCommand(() -> hIntakeSubsystem.intakeReverse()),




                //getting first row
                new ParallelCommandGroup(
                        new InstantCommand(() -> hIntakeSubsystem.gateClose()),
                        new DriveToPointCommand(driveSubsystem, blueThirdRowReady, 5, 5),
                        new InstantCommand(() -> shooterSubsystem.turretOff())
                ),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),




                new DriveToPointCommand(driveSubsystem, blueThirdRowIntake, 5, 5).withTimeout(1500),
//                new WaitCommand(300),
                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),


                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(2300))
                        ),
                //shooting first row
                        new SequentialCommandGroup(
                                new DriveToPointCommand(driveSubsystem, blueOpenGateCheckpoint, 5, 2).withTimeout(1000),
                                new DriveToPointCommand(driveSubsystem, blueOpenGate, 5, 2).withTimeout(1000),
                        new WaitCommand(250),
                                new DriveToPointCommand(driveSubsystem, blueOpenGateCheckpoint, 5, 2).withTimeout(1000),
                                new ParallelCommandGroup(
                                new DriveToPointCommand(driveSubsystem, blueShootFront, 5, 2),
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                new InstantCommand(() -> shooterSubsystem.turretOn())
                                        )
                                )
                        )

                ),
//                new WaitCommand(300),

                new ShootTimeCloseTele(shooterSubsystem,hIntakeSubsystem,0,2300),
                new InstantCommand(() -> hIntakeSubsystem.intakeReverse()),

                //intake second row
                new ParallelCommandGroup(
                        new InstantCommand(() -> hIntakeSubsystem.gateClose()),
                        new DriveToPointCommand(driveSubsystem, blueSecondRowReady, 5, 5),
                        new InstantCommand(() -> shooterSubsystem.turretOff())
                ),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),

                new DriveToPointCommand(driveSubsystem, blueSecondRowIntake, 3, 5).withTimeout(1500),
//                new WaitCommand(300),
//                new DriveToPointCommand(driveSubsystem, redSecondRowReadyCheckpoint, 3, 5),
                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),



                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(2300)),
                                new WaitCommand(500),
                                    new InstantCommand(() -> shooterSubsystem.turretOn())
                        ),
                        //shooting first row
                        new DriveToPointCommand(driveSubsystem, blueShootFront, 5, 2)
                ),
//                new WaitCommand(300),

                new ShootTimeCloseTele(shooterSubsystem,hIntakeSubsystem,0,2300),
                new InstantCommand(() -> hIntakeSubsystem.intakeReverse()),



                //intake third row
                new ParallelCommandGroup(
                        new InstantCommand(() -> hIntakeSubsystem.gateClose()),
                        new DriveToPointCommand(driveSubsystem, blueFirstRowReady, 5, 5),
                        new InstantCommand(() -> shooterSubsystem.turretOff())
                        ),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),


                new DriveToPointCommand(driveSubsystem, blueFirstRowIntake, 5, 5).withTimeout(1500),
//                new WaitCommand(300),
                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),



                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(2300)),
                                new WaitCommand(500),
                                new InstantCommand(() -> shooterSubsystem.turretOn())

                                ),
                        //shooting first row
                        new DriveToPointCommand(driveSubsystem, blueShootFront, 5, 2)
                ),
//                new WaitCommand(300),
//                new WaitCommand(500),
                new ShootTimeCloseTele(shooterSubsystem,hIntakeSubsystem,0,2300),
                new DriveToPointCommand(driveSubsystem, new Pose2d(6, -17, Rotation2d.fromDegrees(45)), 5, 3),
                new InstantCommand(()-> shooterSubsystem.turretOff())













        ));



    }


}
