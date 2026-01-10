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
        tps =0;
    }

    public void launch()
    {
        if(num==0) {
            tps=(4400);
            num = 3;
        } else if (num == 3) {
            tps=(2200);
            num=2;
        } else if (num==2) {
            tps=(1800);
            num=1;
        }
        else{
            tps=0;
            num=0;
        }
        left.setVelocity(tps);
        right.setVelocity(tps);
    }
    public int state()
    {
        return num;
    }
    public double showRPM()
    {
        return tps;
    }
    public void increase()
        {
            if(tps<(6000))
            {
                tps+=50;
            }
            left.setVelocity(tps);
            right.setVelocity(tps);

        }
        public void decrease()
        {
            if(tps>0)
            {
                tps-=50;
            }
            left.setVelocity(tps);
            right.setVelocity(tps);

        }
        }

