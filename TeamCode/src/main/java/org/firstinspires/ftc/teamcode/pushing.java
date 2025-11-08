package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;

public class pushing {

    private CRServo R;
    private CRServo L;
    boolean state;
    public void init(HardwareMap hw)
    {
        R=hw.get(CRServo.class,"servoflyAR");
        R.setPower(0);
    }
    public void bp(boolean state)  {
        if(state){
        R.setPower(-0.155);
        }
        else
        {
            R.setPower(0);
        }
    }
}
