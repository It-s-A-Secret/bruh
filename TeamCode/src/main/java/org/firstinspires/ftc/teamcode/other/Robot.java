package org.firstinspires.ftc.teamcode.other;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.hIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.shooterSubsystem;


@Config
public abstract class Robot extends CommandOpMode {

    //commands




    //commmand groups


    //test statics
    public static double x = 0, y = 0;

    //hardware
    public MotorEx BL, BR, FL, FR, armLeft, armRight;
    public MotorEx shooter, shooter2, turret;
    public DcMotor intake, stopper;
    public Servo gate;
    public Servo hood;
    public CRServo rightTransfer, leftTransfer;

    public GoBildaPinpointDriver pinpoint;
    private MecanumDrive mecanumDrive;
    public IMU gyro;

    //subsystems
    public DriveSubsystem driveSubsystem;
    public shooterSubsystem shooterSubsystem;
    public hIntakeSubsystem hIntakeSubsystem;
    public LimelightSubsystem limelightSubsystem;

    private LynxModule controlHub;

    //voltage
    private VoltageSensor voltageSensor;
    public static double batteryVoltage = 12;
    final double nominalVoltage = 12;
    public static double voltageCompensation = 1;
    private ElapsedTime voltageReadInterval = new ElapsedTime();

    //gamePads
    public GamepadEx m_driver;
    public GamepadEx m_driverOp;
    public Gamepad standardDriver1;
    public Gamepad standardDriver2;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    public CustomButton customButton;
    enum CustomButton {
        TOUCH, LEFTSTICKBUTTON, RIGHTSTICKBUTTON
    }
    public static CustomButton CustomButton;

    //random Todo: Need to clean up my loop time telemetry
    ElapsedTime time = new ElapsedTime();

    boolean manual = false;
    boolean flag = false;

    public void initialize(){
        time.reset();

        //general system
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        manual = false;

        //Pinpoint
        configurePinpoint();

        //dt
        FL = new MotorEx(hardwareMap, "FL");
        FR = new MotorEx(hardwareMap, "FR");
        BL = new MotorEx(hardwareMap, "BL");
        BR = new MotorEx(hardwareMap, "BR");
        FL.setRunMode(MotorEx.RunMode.RawPower);
        FR.setRunMode(MotorEx.RunMode.RawPower);
        BL.setRunMode(MotorEx.RunMode.RawPower);
        BR.setRunMode(MotorEx.RunMode.RawPower);
        FL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        mecanumDrive = new MecanumDrive(FL, FR, BL, BR);
        gyro = hardwareMap.get(IMU.class, "imu");
        gyro.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP)
                )
        );

        driveSubsystem = new DriveSubsystem(FR, FL, BR, BL, mecanumDrive, telemetry, pinpoint);
        register(driveSubsystem);time.reset();

        //general system
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        manual = false;

        //Pinpoint
        configurePinpoint();

        //dt
        FL = new MotorEx(hardwareMap, "FL");
        FR = new MotorEx(hardwareMap, "FR");
        BL = new MotorEx(hardwareMap, "BL");
        BR = new MotorEx(hardwareMap, "BR");
        FL.setRunMode(MotorEx.RunMode.RawPower);
        FR.setRunMode(MotorEx.RunMode.RawPower);
        BL.setRunMode(MotorEx.RunMode.RawPower);
        BR.setRunMode(MotorEx.RunMode.RawPower);
        FL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        FR.setInverted(false);
        BR.setInverted(false);
        FL.setInverted(false);
        BL.setInverted(true);

        mecanumDrive = new MecanumDrive(FL, FR, BL, BR);
        gyro = hardwareMap.get(IMU.class, "imu");
        gyro.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP)
                )
        );

        driveSubsystem = new DriveSubsystem(FR, FL, BR, BL, mecanumDrive, telemetry, pinpoint);
        register(driveSubsystem);


        shooter = new MotorEx(hardwareMap, "shooter");
        shooter2 = new MotorEx(hardwareMap, "shooter2");
        shooter.setRunMode(MotorEx.RunMode.RawPower);
        shooter2.setRunMode(MotorEx.RunMode.RawPower);
        shooter.setInverted(true);
        shooter2.setInverted(true);
        turret = new MotorEx(hardwareMap, "turret");
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);
        shooterSubsystem = new shooterSubsystem(shooter, shooter2, turret, hood, driveSubsystem, telemetry);
        register(shooterSubsystem);

        intake = hardwareMap.get(DcMotor.class, "intake");

        gate = hardwareMap.get(Servo.class, "gate");

        hIntakeSubsystem = new hIntakeSubsystem(intake, gate, telemetry);
        register(hIntakeSubsystem);


        limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetry);
        register(limelightSubsystem);

        m_driver = new GamepadEx(gamepad1);
        m_driverOp = new GamepadEx(gamepad2);

        configureCommands();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset",pinpoint.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Y offset",pinpoint.getYOffset(DistanceUnit.INCH));
        telemetry.addData("pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Device Version Number:",pinpoint.getDeviceVersion());
        telemetry.addData("Device SCalar",pinpoint.getYawScalar());
        telemetry.update();



    }

    @Override
    public void run(){
        if (!flag) {
            super.run(); //whatever you need to run once
            voltageReadInterval.reset();
            flag = true;
        }

        super.run();


        //voltage
        if(voltageReadInterval.seconds() >= 1){
            voltageReadInterval.reset();
            batteryVoltage = voltageSensor.getVoltage();
            voltageCompensation = batteryVoltage/nominalVoltage;
        }

        //other telemetry

        //loopTime
        telemetry.addData("hz ", 1/(time.seconds()));
        telemetry.update();
        time.reset();
        //clear cache
        controlHub.clearBulkCache();
    }

    public void configureCommands(){

        //home poses

        //scoring


        //intaking
        //intake from closer

        //intaking from the wall

        //arm auto parking






        //climbing



        //scoring

        //intakeRightScoreFrontHighChamberCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber);
        //intaking



        //command groups
        ;

    }


    private void configurePinpoint() {
        telemetry.addLine("Configuring Pinpoint...");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

       pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */

       //pinpoint.setOffsets(73.66, 162.56); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setOffsets(0,-5.275);
        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
       pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);


//        pinpoint.setEncoderResolution(15.0313);

        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
       pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);


       //set yaw scalar

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
       pinpoint.recalibrateIMU();
       //pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset",pinpoint.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Y offset",pinpoint.getYOffset(DistanceUnit.INCH));
        telemetry.addData("pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Device Version Number:",pinpoint.getDeviceVersion());
        telemetry.addData("Device SCalar",pinpoint.getYawScalar());
        telemetry.update();
    }

}