package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(60,0,0,12.129);
        left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        num=0;
        tps =0;
        state=0;
    }

    public void launch(double distance)
    {
        if(num==0) {

            if (distance > 0.7 && distance <= 1.2){
                tps = 1150;
            }
            else if (distance > 1.2 && distance <= 1.4) {
                tps = (1200);
            }
            else if (distance > 1.4 && distance <= 1.6) {
                tps = (1225);
            }
            else if (distance > 1.6 && distance <= 1.8) {
                tps = (1300);
            }
            else if (distance > 1.8 && distance <= 2.0) {
                tps = (1350);
            }
            else if (distance > 2.0 && distance <= 2.2) {
                tps = (1500);
            }

            else if (distance > 2.7) {
                tps = (1775);
            }
        }
        else if(num==1)
        {
            if(state==0)
            {
                tps=500;
            } else if (state==1) {
                tps=1180;
            }
            else if(state==2)
            {
                tps=1220;
            }
            else if(state==3)
            {
                tps=1270;
            }
            else if(state==4)
            {
                tps = (1320);
            }
            else if(state==5)
            {
                tps = (1400);
            }
            else if(state==6)
            {
                tps = (1800);
            }
            else {
                tps=500;
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
                state=4;
            else if(state==4)
                state=5;
            else if(state==5)
                state=6;
            else if(state==6)
                state=0;
        }

        public void auto(double tps) {
            left.setVelocity(tps);
            right.setVelocity(tps);
        }
}



