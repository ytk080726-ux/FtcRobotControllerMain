package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "test")
public class test extends OpMode {
    boolean state;
    DcMotorEx leftMotor, rightMotor;
    DcMotor transfer;
    Servo blocker;
    double position;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "left");
        rightMotor = hardwareMap.get(DcMotorEx.class, "right");

        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(DcMotor.Direction.REVERSE);
        transfer.setPower(0.3);

        blocker = hardwareMap.get(Servo.class, "block");
        position = 0.1;
        blocker.setPosition(position);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        state = true;
    }

    @Override
    public void loop() {
        telemetry.addData("Motor Speed", leftMotor.getVelocity());

        if (gamepad1.a) {
            leftMotor.setVelocity(((3460/60)*28));
            rightMotor.setVelocity(((3460/60)*28));

            sleep(1000);

            position = 0.40;
            blocker.setPosition(position);
        }
        else {
            position = 0.1;
            blocker.setPosition(position);

            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
        }
    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
