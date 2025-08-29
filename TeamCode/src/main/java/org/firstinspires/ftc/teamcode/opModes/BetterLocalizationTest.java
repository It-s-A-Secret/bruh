package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingPosLeft;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.LocalTestCommand;
import org.firstinspires.ftc.teamcode.commands.SetStartingPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;

@TeleOp(name = "leftSideLocalTest")
public class BetterLocalizationTest extends Robot {

    LocalTestCommand localTestCommand;
    SetStartingPosCommand setStartingPosCommand;

    @Override
    public void initialize() {
        super.initialize();

        schedule( new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosLeft)));

        schedule( new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosLeft)));
        localTestCommand = new LocalTestCommand(driveSubsystem, pinpoint, telemetry, m_driver, true, 10, m_driver::getLeftX, m_driver::getLeftY, m_driver::getRightX);
        driveSubsystem.setDefaultCommand(localTestCommand);
    }


}
