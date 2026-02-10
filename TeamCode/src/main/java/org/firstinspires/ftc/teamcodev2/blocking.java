package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class blocking {
    private Servo block;
    boolean state;

    public void init(HardwareMap hw)
    {
        block=hw.get(Servo.class,"stopping");
        block.setPosition(0.20);
        state = true;
    }
    public void stopping(boolean state)
    {
        if(state)
        {
            block.setPosition(0.2);
        }
        else
        {
            block.setPosition(0.40);
        }
    }
}
