package org.firstinspires.ftc.teamcode.opModes;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;

@Config
@Autonomous(name="autoTest")
public class autoTest extends Robot {

    private Button dUp1, dDown1, dLeft1, dRight1, bLeft1, bRight1;

    public static double sprintDistance = 20;

    @Override
    public void initialize(){
        super.initialize();



        dUp1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_UP);
        dDown1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_DOWN);
        dLeft1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_LEFT);
        dRight1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_RIGHT);
        bLeft1 = new GamepadButton(m_driver, GamepadKeys.Button.LEFT_BUMPER);
        bRight1 = new GamepadButton(m_driver, GamepadKeys.Button.RIGHT_BUMPER);

        dUp1.whenPressed(new DriveToPointCommand(driveSubsystem, new Pose2d(0, sprintDistance, new Rotation2d(0)), 2, 2));
        dDown1.whenPressed(new DriveToPointCommand(driveSubsystem, new Pose2d(0, -sprintDistance, new Rotation2d(0)), 2, 2));
        dLeft1.whenPressed(new DriveToPointCommand(driveSubsystem, new Pose2d(-sprintDistance, 0, new Rotation2d(0)), 2, 2));
        dRight1.whenPressed(new DriveToPointCommand(driveSubsystem, new Pose2d(sprintDistance, 0, new Rotation2d(0)), 2, 2));
        bLeft1.whenPressed(new DriveToPointCommand(driveSubsystem, new Pose2d(0, 0, new Rotation2d(Math.toRadians(90))), 2, 2));
        bRight1.whenPressed(new DriveToPointCommand(driveSubsystem, new Pose2d(0, 0, new Rotation2d(Math.toRadians(-90))), 2, 2));

//        armSubsystem.resetSlideEncoder();


        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
                new DriveToPointCommand(driveSubsystem, new Pose2d(0, 0, new Rotation2d(0)), 2, 2)
        ));

        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));

    }

}
