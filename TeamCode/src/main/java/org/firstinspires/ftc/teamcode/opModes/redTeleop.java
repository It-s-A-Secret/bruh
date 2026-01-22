package org.firstinspires.ftc.teamcode.opModes;

//import static org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem.alliance;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RedTeleop❤️\uD83C\uDFAE❤️\uD83C\uDFAE❤️\uD83C\uDFAE")
public class redTeleop extends TeleopOpMode {
    public void initialize(){
        TeamBlue = false;
        super.initialize();
        shooterSubsystem.teamRed();
    }

    @Override
    public void run(){
        super.run();

    }


}
