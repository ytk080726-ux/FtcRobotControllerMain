package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;

public class pushing {

    private Servo L;
    double pos;
    boolean state;
    public void init(HardwareMap hw)
    {
        //R=hw.get(CRServo.class,"servoflyAR");
        L=hw.get(Servo.class,"servoflyAR");
        //R.setPower(0);
        pos=0.1;
        L.setPosition(pos);
    }

    public void set(boolean state)
    {
        if(state){
            L.setPosition(pos+0.5);
        }
        else
        {
            L.setPosition(pos);
        }    }


    public double getpos() {

        return L.getPosition();
    }
}
