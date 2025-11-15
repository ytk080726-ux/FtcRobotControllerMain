package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class launcherB {
    DcMotor launcherB;
    boolean state;

    public void init(HardwareMap hw)
    {
        launcherB = hw.get(DcMotor.class,"transfer");
        launcherB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherB.setDirection(DcMotor.Direction.FORWARD);
        state=true;
    }
    public void up()
    {
                launcherB.setPower(0.3);
    }
    public void kill()
    {
        launcherB.setPower(0);
    }

}
