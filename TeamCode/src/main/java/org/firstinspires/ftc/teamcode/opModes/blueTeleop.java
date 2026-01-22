package org.firstinspires.ftc.teamcode.opModes;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="BlueTeleop\uD83D\uDC99\uD83C\uDFAE\uD83D\uDC99\uD83C\uDFAE\uD83D\uDC99\uD83C\uDFAE")
public class blueTeleop extends TeleopOpMode {

    public void initialize(){
        TeamBlue = true;
        super.initialize();
        shooterSubsystem.teamBlue();

    }

    @Override
    public void run(){
        super.run();

    }


}
