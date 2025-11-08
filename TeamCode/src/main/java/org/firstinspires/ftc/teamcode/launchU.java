package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class launchU {
    DcMotorEx launch;
    boolean launchHighState;
    boolean launchLowState;

    public void init(HardwareMap hw) {
        launch = hw.get(DcMotorEx.class, "shooter");
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setDirection(DcMotor.Direction.FORWARD);
        //launchUState = true;v
    }
    public void highLaunch()
    {
        if (launchHighState) {
            launch.setPower(0.3);
        }
        else {
            launch.setPower(0);
        }
        //launchUState = !launchUState;
    }

    public void lowLaunch()
    {
        if (launchLowState) {
            launch.setPower(0.1);
        }
        else {
            launch.setPower(0);
        }
        //launchUState = !launchUState;
    }

//    public void launchD() {
//        launch.setPower(0);
//    }
//
//    public void launchL() {
//        launch.setPower(0.001);
//    }
//
//    public void launchM() {
//        launch.setPower(0.01);
//    }
//
//
//    public double speed() {
//        return launch.getPower();
//    }
}
