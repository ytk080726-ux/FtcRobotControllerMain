package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class launchU {
    DcMotorEx launch;
    boolean launchHighState;
    boolean launchLowState;
    int num;
    double tps;
    public void init(HardwareMap hw) {
        launch = hw.get(DcMotorEx.class, "shooter");
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setDirection(DcMotorEx.Direction.FORWARD);
        num=0;
    }

        //launch.setTargetPosition((int) (28/(Math.PI*96)));

    public void sort()
    {
        launch.setVelocity((1100/60)*28);
    }

    public void launch()
    {
        if(num==0) {
            launch.setVelocity(((3400 / 60) * 28));
            num = 3;
        } else if (num==3) {
            launch.setVelocity(((3000/60)*28));
            num=2;
        } else if (num==2) {
            launch.setVelocity(((2800/60)*28));
            num=1;
        }
        else{
            launch.setVelocity(0);
            num=0;
        }
    }
    public int state()
    {
     return num;
    }
}
