package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class launchU {
    DcMotorEx launch;
    boolean launchHighState;
    boolean launchLowState;
    double tps;
    public enum power
    {
        high,mid,low,dead

    }
    power current;
    public void init(HardwareMap hw) {
        launch = hw.get(DcMotorEx.class, "shooter");
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setDirection(DcMotorEx.Direction.FORWARD);
        current = power.high;
    }

        //launch.setTargetPosition((int) (28/(Math.PI*96)));

    public void sort()
    {
        launch.setVelocity((1100/60)*28);
    }
    public void kill()
    {
        launch.setVelocity(0);
    }

    public void launch()
    {
        switch (current) {
            case high:
                launch.setVelocity(((3400/60)*28));
                current=power.mid;
            case mid:
                launch.setVelocity(((3000/60)*28));
                current=power.low;
            case low:
                launch.setVelocity(((2800/60)*28));
                current=power.dead;
            case dead:
                launch.setPower(0);
                current=power.high;
        }
    }
}
