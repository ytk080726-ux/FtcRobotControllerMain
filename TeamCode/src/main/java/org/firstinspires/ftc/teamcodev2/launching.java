package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class launching {
    DcMotorEx left,right;
    int num;
    int num1;
    double tps;
    public void init(HardwareMap hw) {
        left = hw.get(DcMotorEx.class, "turretLeft");
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setDirection(DcMotorEx.Direction.FORWARD);
        right = hw.get(DcMotorEx.class, "turretRight");
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setDirection(DcMotorEx.Direction.REVERSE);
        num=0;
        num1=0;
    }

    //launch.setTargetPosition((int) (28/(Math.PI*96)));


    public void launch()
    {
        if(num==0) {
            left.setVelocity(((3460/ 60) * 28)); // State 3
            right.setVelocity(((3460/ 60) * 28)); // State 3

            num = 3;
        } else if (num == 3) {
            left.setVelocity(((2625/60)*28)); // State 2
            right.setVelocity(((2625/60)*28)); // State 2

            num=2;
        } else if (num==2) {
            left.setVelocity(((2567/60)*28)); // State 1
            right.setVelocity(((2567/60)*28)); // State 1

            num=1;
        }
        else{
            left.setVelocity(0);
            right.setVelocity(0);

            num=0;
        }
    }
    public int state()
    {
        return num;
    }
}
