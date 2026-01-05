package org.firstinspires.ftc.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testing")
public class testing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor testing = hardwareMap.get(DcMotor.class,"1");
        while(true) {
            testing.setPower(0.7);
        }

    }
}
