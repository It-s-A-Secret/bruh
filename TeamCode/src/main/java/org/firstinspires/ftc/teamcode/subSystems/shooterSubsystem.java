package org.firstinspires.ftc.teamcode.subSystems;

//import static org.firstinspires.ftc.teamcode.other.Globals.armFoldX;
//import static org.firstinspires.ftc.teamcode.other.Globals.armFoldY;
//import static org.firstinspires.ftc.teamcode.other.Globals.manualArm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.other.Globals;
import org.firstinspires.ftc.teamcode.other.PosGlobals;

@Config
public class shooterSubsystem extends SubsystemBase {

    private MotorEx shooter, shooter2;
    private MotorEx turret;

    private Servo hood;

    private MotorGroup arm;
    private Servo endStop;
    private AnalogInput armEncoder;
    private Telemetry telemetry;
//    private LimelightSubsystem limelightSubsystem;

    private DriveSubsystem driveSubsystem;
    public static double kPWheel = 0.003, kIWheel = 0.00, kDWheel = 0.000025;

    private PIDController headingController;
    public static double kPHeading = 0.08, kIHeading = 0.00, kDHeading = 0.000025;
    public static double kFWheel = 0.000265;
    private PIDController velocityController;
    private PIDController turretController;

    private double targetRPM = 0;
    private double currentRpm = 0;
    private boolean wheelOn = true;
    private double rpmError =0;
    public double currentAngle, targetAngle;

    public boolean turretOn;

    public boolean turretTune = false;

    private static final int TICKS_PER_REVSHOOTER = 28;    private static final double TICKS_PER_REVTURRET = 103.8;

    public enum Alliance {
        RED,
        BLUE

    }
    public static Alliance alliance = Alliance.RED;

    public static Pose2d pinpointToShooter = new Pose2d(14.25,3, Rotation2d.fromDegrees(0));





    InterpLUT rpmLUT = new InterpLUT();




    //last command store
    Command currentCommand;
    Command lastCommand;

    //constructor
    public shooterSubsystem(MotorEx shooter, MotorEx shooter2, MotorEx turret, Servo hood, DriveSubsystem driveSubsystem, Telemetry telemetry) {
        this.shooter = shooter;
        this.shooter2 = shooter2;
        this.telemetry = telemetry;
        this.turret = turret;
        this.hood = hood;
        this.driveSubsystem =  driveSubsystem;
        shooter.resetEncoder();
        shooter2.resetEncoder();
        turret.resetEncoder();



        velocityController = new PIDController(kPWheel, kIWheel, kDWheel);
        headingController = new PIDController(kPHeading, kIHeading, kDHeading);




    }



    public void setTargetRPM(double rpm){
        wheelOn = true;
        targetRPM = rpm;
    }
    public double getTargetRPM(){
        return targetRPM;
    }
    public double getCurrentRPM(){
        return currentRpm = -shooter.getVelocity() * 60.0 / TICKS_PER_REVSHOOTER;
    }
    public void stop() {
        wheelOn = false;
    }

    public double getRPMError(){
        return rpmError;
    }

    public void turretTuneTrue(){
        turretTune = true;
    }
    public void turretTuneFalse(){
        turretTune = false;
    }




    public double getTurretAngle(){
        return turret.getDistance() / TICKS_PER_REVTURRET * 360 * 20/95; //divided by resolution multiplied by 360 an then multiply by gear ratio
    }

    public double getRobotXFromGoal(){
        if(alliance == Alliance.RED) {
            return PosGlobals.REDFIELD.getY() - driveSubsystem.getPos().getY(); //NOT WRONG THE Y IS IN THE X FOR THE FIELD FOR SOME REASON
        }else{
            return -(PosGlobals.BLUEFIELD.getY() - driveSubsystem.getPos().getY()); //NOT WRONG THE Y IS IN THE X FOR THE FIELD FOR SOME REASON
        }
    }

    public double getRobotYFromGoal(){
        if(alliance == Alliance.RED) {
            return PosGlobals.REDFIELD.getX() + driveSubsystem.getPos().getX(); //NOT WRONG THE THINGS ARE FLIPPED
        }else{
            return driveSubsystem.getPos().getX()+PosGlobals.BLUEFIELD.getX();
        }
    }

    public double getRobotAngleFromGoal(){
        return 90- Math.toDegrees(Math.atan2(getRobotYFromGoal(),getRobotXFromGoal()));
    }


    public void turretOn(){
        turretOn = true;
    }

    public void turretOff(){
        turretOn = false;
    }

    public void setHoodClose(){
        hood.setPosition(Globals.hoodClose);
    }
    public void setHoodFar(){
        hood.setPosition(Globals.hoodFar);
    }

    public void teamRed(){
        alliance = Alliance.RED;
    }

    public void teamBlue(){
        alliance = Alliance.BLUE;
    }

    public void turretPower(double power){
        turret.setVelocity(power);
    }

    public void resetTurret(){
        turret.resetEncoder();
    }


    public class TimeStampedPosition {
        private final double position; // For the arm, could be degrees or extension length
        private final long timestamp;  // Timestamp in milliseconds

        public TimeStampedPosition(double position, long timestamp) {
            this.position = position;
            this.timestamp = timestamp;
        }



        @Override
        public String toString() {
            return "TimeStampedPosition{" +
                    "position=" + position +
                    ", timestamp=" + timestamp +
                    '}';
        }
    }

    public Command getLastCommand(){
        //redundant null checking
        if(lastCommand==null){
            return new InstantCommand();
        }
        return lastCommand;
    }

    @Override
    public void periodic() {
        if (!turretTune) {
            velocityController.setPID(kPWheel, kIWheel, kDWheel);
            //read
            double currentRPM = getCurrentRPM();
            double ff = kFWheel * targetRPM;

            double pid = velocityController.calculate(currentRPM, targetRPM);

            double power = ff + pid;
            rpmError = currentRPM - targetRPM;

            power = Math.max(-1, Math.min(1, power));
            if (wheelOn) {
                shooter.set(-power);
                shooter2.set(power);
            } else {
                shooter.set(0);
                shooter2.set(0);
                targetRPM = 0;
            }
            headingController.setPID(kPHeading, kIHeading, kDHeading);
            double currentAngle = getTurretAngle() + driveSubsystem.getPos().getRotation().getDegrees();
            if (!turretOn) {
                targetAngle = driveSubsystem.getPos().getRotation().getDegrees();
            } else {
                targetAngle = -getRobotAngleFromGoal();
                if (alliance == Alliance.BLUE) {
                    targetAngle = (getRobotAngleFromGoal());
                }
            }
            double pidTurret = headingController.calculate(currentAngle, targetAngle);
            double powerTurret = Math.max(-1, Math.min(1, pidTurret));

            turret.set(powerTurret);

            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("targetAngle", targetAngle);



            telemetry.addData("TargetRPM", targetRPM);
            telemetry.addData("CurrentRPM", currentRPM);
            telemetry.addData("Power", power);
            telemetry.addData("AngleFRomGoal", getRobotAngleFromGoal());
        }













        //last command
        currentCommand = CommandScheduler.getInstance().requiring(this);

        if (currentCommand != null && currentCommand != lastCommand) {
            lastCommand = currentCommand;
        }
        //Redundent null checking
        if(lastCommand==null){
            lastCommand=new InstantCommand();
        }
        telemetry.addData("shooterSubsystemLastCommand", lastCommand != null ? lastCommand.getName() : "None");




    }






}
