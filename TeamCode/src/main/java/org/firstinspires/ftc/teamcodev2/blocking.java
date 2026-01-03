package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class blocking {
    private Servo block;
    public void init(HardwareMap hw)
    {
        block=hw.get(Servo.class,"stopping");
    }
    public void stopping(boolean state)
    {
        if(state==true)
        {
            block.setPosition(0.5);
        }
        else
        {
            block.setPosition(0);
        }
    }
}
