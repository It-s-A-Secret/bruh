package org.firstinspires.ftc.teamcode.subSystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;


@Config
public class LimelightSubsystem extends SubsystemBase {


//    public enum Alliance {
//        RED,
//        BLUE
//
//    }
//    public static Alliance alliance = Alliance.RED;



    private final Limelight3A camera;

    private boolean isDataOld = false;
    private double targetTag = 20;
    private PIDController headingController;
    public static double kPHeading = 0.035, kIHeading = 0.00, kDHeading = 0.000025;
    private double angle;

    public double currentAngle, targetAngle;
    public boolean turretOn;



    public static double CAMERA_HEIGHT = 11.23-1.5;//limelight height minus height of sample (limelight detects top of sample), 0.8 for offset cuz works?
    public static double CAMERA_ANGLE = 0; //downwards Angle
    private static final double TICKS_PER_REVTURRET = 103.8;




    Pose2d botToLimelight = new Pose2d(new Translation2d(6.556, 5.45), new Rotation2d(Math.toRadians(0)));


//    public static double TARGET_HEIGHT = ;
//
//    public static double strafeConversionFactor = ;  //whatever numbers that the actual height and stuff is.
//    public static double cameraStrafeToBot = ;
//
//    public static double sampleToRobotDistance = ;

    Telemetry telemetry;





    public LimelightSubsystem(final HardwareMap hardwareMap,Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "Limelight");

        this.telemetry = telemetry;

//
//        if (alliance == Alliance.BLUE){
//            targetTag = 20;
//        }
//        else{
//            targetTag = 24;
//        }
        headingController = new PIDController(kPHeading, kIHeading, kDHeading);

        camera.pipelineSwitch(5);

        initializeCamera();







    } //pipeline 2 is yellow

    public void initializeCamera() {

        camera.setPollRateHz(50);
        camera.start();
    }

    @Override
    public void periodic() {
//        if(camera.isRunning()) {
//
//            telemetry.addData("cameraNotRunning", "false");
//            Optional<LLResult> optionalResult = getResult(); // call this to get the limelight results
//            if (!optionalResult.isEmpty()) {
//                Log.i("limelightValid", "true");
//                LLResult result = optionalResult.get();
//                long staleness = result.getStaleness();
//                isDataOld = staleness >= 100; //100 ms
//                telemetry.addData("result", result.getFiducialResults());
//            }
//            Optional<Rotation2d> rotation = getRotation();
//            if (rotation.isPresent()) {
//                telemetry.addData("angle", rotation.get().getDegrees());
//
//
//                headingController.setPID(kPHeading, kIHeading, kDHeading);
////                double currentAngle = getTurretAngle();
//                double targetAngle = currentAngle + getRotationAngle();
//                double pid = headingController.calculate(currentAngle, targetAngle);
//                double power = Math.max(-1, Math.min(1, pid));
//                if (turretOn) {
////                    turret.set(power);
//                } else {
////                    turret.set(0);
//                }
//                telemetry.addData("currentAngle", currentAngle);
//                telemetry.addData("targetAngle", targetAngle);
//            }else{
////                turret.set(0);
//            }
//        }






    }


    public Optional<LLResult> getResult(){
        if(camera.isRunning()) {
            LLResult result = camera.getLatestResult();
            if(result==null){
                return Optional.empty();
            }
            return Optional.of(result);
        }
        return Optional.empty();
    }
    public double getAngle(){
        return 0;
    }

    public Optional<Rotation2d> getRotation(){

        Log.i("llbruh", "bruh");

        if(camera.isRunning()){
            Optional<LLResult> optionalResult = getResult();

        }
            Optional<LLResult> optionalResult = getResult();
        if(!optionalResult.isPresent()){return Optional.empty();}

        LLResult results = optionalResult.get();

        ArrayList<Rotation2d> poses = new ArrayList<Rotation2d>();

        Log.i("llSize", String.valueOf(results.getFiducialResults().size()));

        for (LLResultTypes.FiducialResult result : results.getFiducialResults()) {

            Log.i("llresult", String.valueOf(result.getFiducialId()));


            //relative to limelight
            double ID = result.getFiducialId();
            double Bob = result.getTargetXDegrees();
            double forward = Math.tan(Math.toRadians(CAMERA_ANGLE + result.getTargetYDegrees())) * CAMERA_HEIGHT;
//            forward += tyCompensation.get(forward);
            telemetry.addData("ID", ID);
            telemetry.addData("Pose stuff", Bob);

            telemetry.addData("forwardRaw", forward);




//            telemetry.addData("anglePre", String.valueOf(angle));

            angle = -Bob; //technically not field relative




            Rotation2d rotationRelativeToBot = new Rotation2d(Math.toRadians(angle));
            telemetry.addData("Rotation stuff", rotationRelativeToBot);

            if (ID != 22) {
                poses.add(rotationRelativeToBot);
            }
        }

        if(poses.size()<=0){
            return Optional.empty();
        }
        telemetry.addData("Rotation List", poses);


        Rotation2d best = new Rotation2d();
        double lowestX = Double.MAX_VALUE;

        telemetry.addData("Rotation Pose", poses.get(0));
        telemetry.addData("Random Rotation", Rotation2d.fromDegrees(45));


        return Optional.of(poses.get(0));
    }

    public double getRotationAngle(){
        if(getRotation().isPresent()) {
            return getRotation().get().getDegrees();
        }
        return 0;
    }

//    public double getTurretPosition(){
//        return turret.getDistance();
//    }
//    public double getTurretError(){
//        return getTurretAngle()-getRotationAngle();
//    }

    public void setTargetVelocity(int velocity){
        targetAngle = velocity;
    }

//    public double getTurretAngle(){
//        return turret.getDistance() / TICKS_PER_REVTURRET * 360 * 20/95; //divided by resolution multiplied by 360 an then multiply by gear ratio
//    }

    public void pauseLimelight(boolean pause){
        if(pause){
            camera.pause();
        }
        else{
            camera.start();
        }
    }
}

