package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {
    boolean state;
    DcMotor intake;
    public void init(HardwareMap hw) {
        intake=hw.get(DcMotor.class,"intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        state=true;
    }

    public void type() {
        if (state)
        {
                intake.setPower(0.4);
        }
        else
        {
            intake.setPower(0);
        }
        state=!state;
    }
}
