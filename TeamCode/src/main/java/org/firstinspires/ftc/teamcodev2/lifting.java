package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class lifting {
    private Servo left,right;
    private boolean state;
    public void init(HardwareMap hw)
    {
        left=hw.get(Servo.class,"leftlift");
        right=hw.get(Servo.class,"rightlift");
        left.setPosition(0.35);
        right.setPosition(0.35);
    }
    public void up()
    {
        right.setPosition(0.35);
        left.setPosition(0.35);
    }

    public void down()
    {
        right.setPosition(0.18);
        left.setPosition(0.5);
    }
}
