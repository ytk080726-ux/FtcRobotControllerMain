package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class transfer {
    boolean toggle;
    private DcMotorEx transfer;
    public void init(HardwareMap hw)
    {
        transfer= hw.get(DcMotorEx.class,"transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        toggle=true;
    }

    public void start() {
        if (toggle)
        {
            transfer.setVelocity(2100);
        }
        else
        {
            transfer.setVelocity(0);
        }
        toggle=!toggle;
    }
}
