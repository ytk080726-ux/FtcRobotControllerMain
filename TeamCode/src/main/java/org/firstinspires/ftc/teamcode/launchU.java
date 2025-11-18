package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class launchU {
    DcMotorEx launch;
    boolean launchHighState;
    boolean launchLowState;
    double tps;

    public void init(HardwareMap hw) {
        launch = hw.get(DcMotorEx.class, "shooter");
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setDirection(DcMotorEx.Direction.FORWARD);
        tps=((3400/60)*28);
        launchHighState=false;
    }
    public void highLaunch()
    {
        //launch.setTargetPosition((int) (28/(Math.PI*96)));
        launch.setVelocity(tps);
    }
    public void midLaunch()
    {launch.setVelocity((3000/60)*28);}
    public void lowLaunch()
    {
        launch.setVelocity((2800/60)*28);
    }
    public void sort()
    {
        launch.setVelocity((1100/60)*28);
    }

    public void kill()
    {
        launch.setVelocity(0);
    }
}
