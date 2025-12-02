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
    int num1;
    double tps;
    public void init(HardwareMap hw) {
        launch = hw.get(DcMotorEx.class, "shooter");
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setDirection(DcMotorEx.Direction.FORWARD);
        num=0;
        num1=0;
    }

        //launch.setTargetPosition((int) (28/(Math.PI*96)));

    public void sort()
    {
        if(num1==0) {
            launch.setVelocity((850 / 60) * 28);
            num1=1;
        }
        else {
            launch.setVelocity(0);
            num1=0;
        }
    }

    public void launch()
    {
        if(num==0) {
            launch.setVelocity(((3450/ 60) * 28)); // State 3
            num = 3;
        } else if (num == 3) {
            launch.setVelocity(((2800/60)*28)); // State 2
            num=2;
        } else if (num==2) {
            launch.setVelocity(((2500/60)*28)); // State 1
            num=1;
        }
        else{
            launch.setVelocity(0);
            num=0;
        }
    }
    public int state()
    {
        if(num1==1)
            return 4;
     return num;
    }
}
