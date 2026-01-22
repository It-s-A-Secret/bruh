package org.firstinspires.ftc.teamcode.opModes;




import static org.firstinspires.ftc.teamcode.other.Globals.farRPM;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueBackFinish;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueFirstRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueFirstRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueSecondRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueSecondRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueShootBack;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueThirdRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.blueThirdRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redFirstRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redFirstRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redSecondRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redSecondRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redShootBack;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redThirdRowIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.redThirdRowReady;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingBlueBack;
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


@Autonomous(name="bruhBlue12Back")
public class bruhBlue12Back extends Robot {

    @Override
    public void initialize(){
        super.initialize();

        shooterSubsystem.teamBlue();
        driveSubsystem.setStartingPos(startingBlueBack);
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));
        shooterSubsystem.turretOff();
        shooterSubsystem.setHoodFar();





                schedule(new SequentialCommandGroup(
                // new InstantCommand(() -> driveSubsystem.setStartingPos(startingRedFront)),
                //wait
                new WaitCommand(100),

                //hold pos
                new InstantCommand(() -> driveSubsystem.driveToPoint(startingBlueBack)),
                new InstantCommand(() -> shooterSubsystem.turretOn()),


                //go to far shoot zone
//                new DriveToPointCommand(driveSubsystem, new Pose2d(-25, 25, Rotation2d.fromDegrees(130)), 5, 2),
//                new DriveToPointCommand(driveSubsystem, redShootFrontStraight, 5, 2),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(farRPM))
                        ),
                        //shooting first row
                        new DriveToPointCommand(driveSubsystem, blueShootBack, 5, 2)
                ),
                //shoot
                new ShootTime(shooterSubsystem,hIntakeSubsystem,0,farRPM),
                new InstantCommand(() -> hIntakeSubsystem.intakeReverse()),




                //getting first row
                new ParallelCommandGroup(
                        new InstantCommand(() -> hIntakeSubsystem.gateClose()),
                        new DriveToPointCommand(driveSubsystem, blueFirstRowReady, 5, 5)
                ),
                new InstantCommand(() -> hIntakeSubsystem.intakeOn()),




                new DriveToPointCommand(driveSubsystem, blueFirstRowIntake, 5, 5).withTimeout(1500),
//                new WaitCommand(300),
                new InstantCommand(() -> hIntakeSubsystem.intakeOff()),


                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(farRPM))
                        ),
                        new DriveToPointCommand(driveSubsystem, blueShootBack, 5, 2)
                        //shooting first row


                ),
//                new WaitCommand(300),

                new ShootTime(shooterSubsystem,hIntakeSubsystem,0,farRPM),
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
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(farRPM))
                        ),
                        //shooting first row
                        new DriveToPointCommand(driveSubsystem, blueShootBack, 5, 2)
                ),
//                new WaitCommand(300),

                new ShootTime(shooterSubsystem,hIntakeSubsystem,0,farRPM),
                new InstantCommand(() -> hIntakeSubsystem.intakeReverse()),
                new InstantCommand(() -> hIntakeSubsystem.gateClose()),



                //intake third row
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
                                new InstantCommand(() -> shooterSubsystem.setTargetRPM(farRPM))
                        ),
                        //shooting first row
                        new DriveToPointCommand(driveSubsystem, blueShootBack, 5, 2)
                ),
//                new WaitCommand(300),
//                new WaitCommand(500),
                new ShootTime(shooterSubsystem,hIntakeSubsystem,0,farRPM),
                new InstantCommand(()-> shooterSubsystem.turretOff()),
                        new DriveToPointCommand(driveSubsystem, blueBackFinish, 5, 5)













                ));



    }


}
