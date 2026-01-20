package org.firstinspires.ftc.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testing")
public class testing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx flywheelLeft = hardwareMap.get(DcMotorEx.class,"1");
        DcMotorEx flywheelRight = hardwareMap.get(DcMotorEx.class,"2");

        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setDirection(DcMotorEx.Direction.REVERSE);



        while(true) {
            flywheelRight.setVelocity(2200);
            flywheelLeft.setVelocity(2200);
        }

    }
}
