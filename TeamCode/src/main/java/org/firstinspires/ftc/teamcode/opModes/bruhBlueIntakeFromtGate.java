package org.firstinspires.ftc.teamcode.opModes;




import static org.firstinspires.ftc.teamcode.other.Globals.closeRPM;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueOpenGate;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueOpenGateCheckpoint;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueOpenGateIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueOpenGateIntakeReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueOpenGateIntakeReadyCheckpoint;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueSecondRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueSecondRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueShootFront;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueThirdRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueThirdRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redOpenGate;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redOpenGateCheckpoint;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redOpenGateIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redOpenGateIntakeReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redOpenGateIntakeReadyCheckpoint;
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


@Autonomous(name="bruhBlueIntakeFromGate")
public class bruhBlueIntakeFromtGate extends Robot {

    @Override
    public void initialize(){
        super.initialize();


        //turn on auto drive
        shooterSubsystem.teamBlue();
        driveSubsystem.setStartingPos(startingBlueFront);
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));
        shooterSubsystem.turretOff();




        schedule(new SequentialCommandGroup(
               // new InstantCommand(() -> driveSubsystem.setStartingPos(startingRedFront)),
                //wait
                new WaitCommand(100),

                //hold pos
                new InstantCommand(() -> driveSubsystem.driveToPoint(startingBlueFront)),
                new InstantCommand(() -> shooterSubsystem.turretOn()),


                //go to far shoot zone
//                new DriveToPointCommand(driveSubsystem, new Pose2d(-25, 25, Rotation2d.fromDegrees(130)), 5, 2),
//                new DriveToPointCommand(driveSubsystem, redShootFrontStraight, 5, 2),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(closeRPM))
                        ),
                        //shooting first row
                        new DriveToPointCommand(driveSubsystem, blueShootFront, 5, 2)
                ),
                //shoot
                new ShootTimeCloseTele(shooterSubsystem,hIntakeSubsystem,0,closeRPM),
                new InstantCommand(() -> hIntakeSubsystem.intakeReverse()),





                //getting first row
                new ParallelCommandGroup(
                        new InstantCommand(() -> hIntakeSubsystem.gateClose()),
                        new DriveToPointCommand(driveSubsystem, blueThirdRowReady, 5, 5)
                ),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),




                new DriveToPointCommand(driveSubsystem, blueThirdRowIntake, 5, 5).withTimeout(1500),
//                new WaitCommand(300),
                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),


                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(closeRPM))
                        ),
                //shooting first row
                        new SequentialCommandGroup(
                                new DriveToPointCommand(driveSubsystem, blueOpenGateCheckpoint, 5, 2).withTimeout(1000),
                                new DriveToPointCommand(driveSubsystem, blueOpenGate, 5, 2).withTimeout(1000),
                        new WaitCommand(250),
                                new DriveToPointCommand(driveSubsystem, blueOpenGateCheckpoint, 5, 2).withTimeout(1000),
                                new DriveToPointCommand(driveSubsystem, blueShootFront, 5, 2)
                        )

                ),
//                new WaitCommand(300),

                new ShootTimeCloseTele(shooterSubsystem,hIntakeSubsystem,0,closeRPM),
                new InstantCommand(() -> hIntakeSubsystem.intakeReverse()),

                //intake second row
                new ParallelCommandGroup(
                        new InstantCommand(() -> hIntakeSubsystem.gateClose()),
                        new DriveToPointCommand(driveSubsystem, blueSecondRowReady, 5, 5)
                ),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),

                new DriveToPointCommand(driveSubsystem, blueSecondRowIntake, 3, 5).withTimeout(1500),
//                new WaitCommand(300),
//                new DriveToPointCommand(driveSubsystem, redSecondRowReadyCheckpoint, 3, 5),
                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),



                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(closeRPM))
                        ),
                        //shooting first row
                        new DriveToPointCommand(driveSubsystem, blueShootFront, 5, 2)
                ),
//                new WaitCommand(300),

                new ShootTimeCloseTele(shooterSubsystem,hIntakeSubsystem,0,closeRPM),
                new InstantCommand(() -> hIntakeSubsystem.intakeReverse()),



                //intake third row
                new ParallelCommandGroup(
                        new InstantCommand(() -> hIntakeSubsystem.gateClose()),
                        new DriveToPointCommand(driveSubsystem, blueOpenGateIntakeReadyCheckpoint, 5, 5)
                ),
                new DriveToPointCommand(driveSubsystem, blueOpenGateIntakeReady, 5, 5),
                new WaitCommand(200),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),


                new DriveToPointCommand(driveSubsystem, blueOpenGateIntake, 5, 5).withTimeout(1500),
                new WaitCommand(1000),
                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),



                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(closeRPM))
                        ),
                        //shooting first row
                        new DriveToPointCommand(driveSubsystem, blueShootFront, 5, 2)
                ),
//                new WaitCommand(300),
//                new WaitCommand(500),
                new ShootTimeCloseTele(shooterSubsystem,hIntakeSubsystem,0,closeRPM),
                new DriveToPointCommand(driveSubsystem, new Pose2d(6, -15, Rotation2d.fromDegrees(45)), 5, 3),
                new InstantCommand(()-> shooterSubsystem.turretOff())














        ));



    }


}
