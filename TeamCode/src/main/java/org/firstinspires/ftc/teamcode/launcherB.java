package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class launcherB {
    DcMotorEx launcherB;
    boolean state;

    public void init(HardwareMap hw)
    {
        launcherB = hw.get(DcMotorEx.class,"transfer");
        launcherB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherB.setDirection(DcMotorEx.Direction.FORWARD);
        launcherB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        state=true;
    }

    // One method to set the power (for redundancy).
    // Prereqs: A double between 0-1
    public void setPower(double newPower) {
        launcherB.setVelocity(newPower);
    }

}
