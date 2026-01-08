package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class transfer {
    private DcMotor transfer;
    public void init(HardwareMap hw)
    {
        transfer= hw.get(DcMotor.class,"transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void start(boolean state)
        {
            if(state) {
                transfer.setPower(0.15);
            }
            else
            {
                transfer.setPower(0);
            }
        }
}
