package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class launching {
    DcMotorEx left,right;
    int num;
    boolean state;
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

    public void launch(double distance)
    {
        if (distance < 1.5) {
            tps=(1700);
        } else if (distance > 1.5 && distance < 2.5) {
            tps=(2200);
        } else if (distance > 2.5) {
            tps=(4400);
        }
        else
        {
           tps = 0;
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

