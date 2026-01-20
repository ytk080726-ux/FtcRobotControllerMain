package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class launching {
    DcMotorEx left,right;
    int num,state;
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
        state=0;
    }

    public void launch(double distance)
    {
        if(num==0) {
            if (distance == 0) {
                tps = (0);
            } else if (distance < 1.3) {
                tps = (1700);
            } else if (distance > 1.3  && distance < 2.5) {
                tps = (2200);
            } else if (distance > 2.5) {
                tps = (3400);
            } else {
                tps = 0;
            }
        }
        else if(num==1)
        {
            if(state==0)
            {
                tps=0;
            } else if (state==1) {
                tps=1700;
            }
            else if(state==2)
            {
                tps=2200;
            }
            else if(state==3)
            {
                tps=3400;
            }
            else {
                tps=2200;
            }
        }

        left.setVelocity(tps);
        right.setVelocity(tps);
    }
    public int state()
    {
        return num;
    }
    public void setState()
    {
        if(num==1)
            num=0;
        else
            num=1;
    }
    public int giveState()
    {
        return state;
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
        public void settingState()
        {
            if(state==0)
                state=1;
            else if(state==1)
                state=2;
            else if(state==2)
                state=3;
            else if(state==3)
                state=0;
        }
        }

