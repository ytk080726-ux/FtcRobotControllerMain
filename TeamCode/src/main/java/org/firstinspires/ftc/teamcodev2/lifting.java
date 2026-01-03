package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class lifting {
    private Servo left,right;
    private boolean state;
    public void inti(HardwareMap hw)
    {
        left=hw.get(Servo.class,"liftLeft");
        right=hw.get(Servo.class,"liftRight");
        left.setPosition(0);
        right.setPosition(0);
        state=false;
    }
    public void lifting()
    {
        if(state==false)
        {
            right.setPosition(0.5);
            left.setPosition(0.5);
            state=true;
        }
        else
        {
            left.setPosition(0);
            right.setPosition(0);
            state=false;
        }
    }
}
