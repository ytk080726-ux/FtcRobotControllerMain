package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class transfer {
    private DcMotor transfer;
    public void init(HardwareMap hw)
    {
        transfer= hw.get(DcMotor.class,"transfer");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void start()
        {
            transfer.setPower(0.3);
        }
}
