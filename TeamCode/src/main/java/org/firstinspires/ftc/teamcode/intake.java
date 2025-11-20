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
        intake.setDirection(DcMotorSimple.Direction.REVERSE );
        state=true;
    }

    // Toggle between on and off mode when A button is pressed
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
    public void revers()
    {
        if(intake.getDirection()== DcMotorSimple.Direction.FORWARD)
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        else
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
