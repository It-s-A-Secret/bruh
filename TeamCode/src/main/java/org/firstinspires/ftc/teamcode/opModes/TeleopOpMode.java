package org.firstinspires.ftc.teamcode.opModes;


import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighBasketX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighBasketY;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallX;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallY;
import static org.firstinspires.ftc.teamcode.other.Globals.armLowBasketY;
import static org.firstinspires.ftc.teamcode.other.Globals.manualArm;
import static org.firstinspires.ftc.teamcode.other.Globals.manualSlides;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenBasket;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenIntake;
import static org.firstinspires.ftc.teamcode.other.Globals.rollIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenBasket;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.teamcode.commandGroups.ShootTime;

import org.firstinspires.ftc.teamcode.commands.TeleDriveCommand;

import org.firstinspires.ftc.teamcode.other.Robot;



@Config
@TeleOp(name="teleOpFunnyTest")
public class TeleopOpMode extends Robot {



    //buttons
    private Button cross1, back2, start2, dUp1, dDown1, dLeft1, dRight1, bRight1, bLeft1, triangle1, triangle2, square1, touchpad1, touchpad2, start1, square2, dUp2, bRight2, bLeft2, dRight2, dDown2, cross2, circle1, circle2, dLeft2, back1, stickButtonLeft2;
    private Trigger tLeft1, tRight1, tLeft2, tRight2;

    //teleop mode
    public static boolean teleopSpec = false;
    public static boolean parallelizing = false;

    public static double rpm = 3900;


    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void initialize(){
        super.initialize();

        //configureMoreCommands();
        configureButtons();
        manualArm = false;
        manualSlides = false;

//        new ArmCoordinatesCommand(armSubsystem, 12, 7).schedule(true);



    }

    /*public void configureMoreCommands() {

    }*/

    public void configureButtons() {
        square1 = new GamepadButton(m_driver, GamepadKeys.Button.X);
        square2 = new GamepadButton(m_driverOp, GamepadKeys.Button.X);
        start2 = new GamepadButton(m_driverOp, GamepadKeys.Button.START);
        back2 = new GamepadButton(m_driverOp, GamepadKeys.Button.BACK);
        dUp1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_UP);
        dUp2 = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_UP);
        dDown1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_DOWN);
        dDown2 = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_DOWN);
        dLeft1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_LEFT);
        dLeft2 = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_LEFT);
        dRight1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_RIGHT);
        bRight1 = new GamepadButton(m_driver, GamepadKeys.Button.RIGHT_BUMPER);
        bLeft1 = new GamepadButton(m_driver, GamepadKeys.Button.LEFT_BUMPER);
        bRight2 = new GamepadButton(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER);
        triangle1 = new GamepadButton(m_driver, GamepadKeys.Button.Y);
        triangle2 = new GamepadButton(m_driverOp, GamepadKeys.Button.Y);
        cross1 = new GamepadButton(m_driver, GamepadKeys.Button.A);
        cross2 = new GamepadButton(m_driverOp, GamepadKeys.Button.A);
        bLeft2 = new GamepadButton(m_driverOp, GamepadKeys.Button.LEFT_BUMPER);
        dRight1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_RIGHT);
        dRight2 = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_RIGHT);
        tLeft1 = new Trigger(() -> m_driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1);
        tRight1 = new Trigger(() -> m_driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1);
        tLeft2 = new Trigger(() -> m_driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1);
        tRight2 = new Trigger(() -> m_driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1);
        start1 = new GamepadButton(m_driver, GamepadKeys.Button.START);
        circle1 = new GamepadButton(m_driver, GamepadKeys.Button.B);
        circle2 = new GamepadButton(m_driverOp, GamepadKeys.Button.B);
        back1 = new GamepadButton(m_driver, GamepadKeys.Button.BACK);
        stickButtonLeft2 = new GamepadButton(m_driverOp, GamepadKeys.Button.LEFT_STICK_BUTTON);




        //sub intake




        //dUp2.whenReleased(armInSubCommand);
        //rotate intake




        //intake close



        //In-sub adjuster for secondary



//        new Trigger(()->Math.abs(m_driverOp.getRightY()) > .1)
//            .whenActive(new InstantCommand(()->intakeSubsystem.setPitch(pitchWhenIntake)).andThen(new ArmManualCommand(armSubsystem, m_driverOp::getLeftY, m_driverOp::getRightY)));
//        .whenActive(new JacobSlideValorantAimer(armSubsystem, m_driverOp::getRightY, ()->m_driverOp.getLeftX() * 90));

        //retract after intaking

//        dDown1.whenPressed(new FlipSample(armSubsystem, intakeSubsystem, secondaryArmSubsystem));
        //retract after intaking and basket (spec mode)

        triangle1.whenPressed(new ShootTime(shooterSubsystem,hIntakeSubsystem, 10, 3900));
        cross1.whenPressed(new ShootTime(shooterSubsystem,hIntakeSubsystem, 10, 3250));


        circle1.whenActive(new ParallelCommandGroup(
                new InstantCommand(()-> hIntakeSubsystem.intakeOn()),
                new InstantCommand(()-> hIntakeSubsystem.stopperStop())
        ));
        circle1.whenInactive(new ParallelCommandGroup(
                new InstantCommand(()-> hIntakeSubsystem.intakeOff()),
                new InstantCommand(()-> hIntakeSubsystem.stopperOff())
        ));
        square1.whenActive(new ParallelCommandGroup(
                new InstantCommand(()-> hIntakeSubsystem.intakeReverse()),
                new InstantCommand(()-> hIntakeSubsystem.stopperStop())
        ));
        square1.whenInactive(new ParallelCommandGroup(
                new InstantCommand(()-> hIntakeSubsystem.intakeOff()),
                new InstantCommand(()-> hIntakeSubsystem.stopperOff())
        ));

        dUp1.whenPressed(new InstantCommand(() -> hIntakeSubsystem.gateOpen()));
        dDown1.whenPressed(new InstantCommand(() -> hIntakeSubsystem.gateClose()));

                //wall intake
//        tRight1.toggleWhenActive(new teleopSpecScore(driveSubsystem,armSubsystem,intakeSubsystem));
//        tLeft1.whenActive(new VisionToSampleInterpolate(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem, false, ()->{return false;},m_driver::getLeftX, m_driver::getLeftY, m_driver::getRightX));
//        tLeft1.toggleWhenActive(new TeleDriveHeadingLocked(driveSubsystem, m_driver));
//        tLeft1.whileActiveContinuous(new holdDTPosCommand(driveSubsystem)); //heading lock
//            tLeft1.whenActive(new ConditionalCommand(
//                        new SequentialCommandGroup(
//                                new WaitForArmCommand(armSubsystem, ArmSubsystem.armMinAngle, 5),
//                                new WaitForSlideCommand(armSubsystem, 10, 2),
//                                new LimelightToSample(driveSubsystem, armSubsystem, secondaryArmSubsystem, intakeSubsystem, limelightSubsystem)
//                        ),
//                        new LimelightToSample(driveSubsystem, armSubsystem, secondaryArmSubsystem, intakeSubsystem, limelightSubsystem),
//                        ()-> armSubsystem.getArmAngle()>7 || armSubsystem.getSlideExtention()>11
//            ))
//                    .whileActiveContinuous(new holdDTPosCommand(driveSubsystem));
//        //chambers
//        square2.whenPressed(new HighChamberCommand(armSubsystem, intakeSubsystem));
//        square2.whenReleased(new ScoreHighChamberCommand(armSubsystem, intakeSubsystem));
        //auto spec scoring




        //dropping sample (into observation zone)



        //specMech





        //baskets




        //low basket


//        //climbing


        //Default Commands
        driveSubsystem.setDefaultCommand(new TeleDriveCommand(driveSubsystem, m_driver, true, 10, m_driver::getLeftX, m_driver::getLeftY, m_driver::getRightX));


    }

    @Override
    public void run(){
        super.run();


        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        //retract and move arm out of the way



        //switch teleop mode
        if(currentGamepad1.touchpad && !previousGamepad1.touchpad){
            teleopSpec = !teleopSpec;
        }

        //switch parrallelizing
        if(currentGamepad2.touchpad && !previousGamepad2.touchpad){
            parallelizing = !parallelizing;
        }

        if(parallelizing){
            gamepad2.setLedColor(128,0,128, LED_DURATION_CONTINUOUS);
        } else{
            gamepad2.setLedColor(255, 255, 0, LED_DURATION_CONTINUOUS);
        }

//        //manual secondary arm  (couldnt figure out how to do it using conditional commands though didnt try very hard)
//        if(gamepad2.left_stick_button){
//            schedule(new InstantCommand(() -> secondaryArmSubsystem.setDiffyYaw(0)));
//        } else if(Math.abs(m_driverOp.getLeftX()) > .05){
//            schedule(new InstantCommand(() -> secondaryArmSubsystem.setDiffyYaw(m_driverOp.getLeftX() * 90 /*makes the range 180degrees*/)));
//        }

//        //manual slides
//        if(Math.abs(m_driverOp.getRightY()) > .1) {
//            schedule(new ArmManualCommand(armSubsystem, m_driverOp::getRightY));
//        }


    }


}