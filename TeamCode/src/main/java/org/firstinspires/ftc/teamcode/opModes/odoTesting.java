package org.firstinspires.ftc.teamcode.opModes;



import static org.firstinspires.ftc.teamcode.other.PosGlobals.moveOdoTestY;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.moveOdoTestX;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingPosRight;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingPosTest;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.AutoBase;
import org.firstinspires.ftc.teamcode.other.Robot;

@Autonomous(name="odoTesting")
public class odoTesting extends AutoBase {

    @Override
    public void initialize(){
        super.initialize();

        //turn on auto drive
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));




        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosTest)),
                //wait
                new WaitCommand(6),

                //hold pos
                new InstantCommand(() -> driveSubsystem.driveToPoint(startingPosTest)),


                new DriveToPointCommand(driveSubsystem, moveOdoTestX ,3, 5).withTimeout(2000),
                new WaitCommand(500),
                new DriveToPointCommand(driveSubsystem, moveOdoTestY ,3, 5).withTimeout(2000)

                //wait

        ));



    }


}
