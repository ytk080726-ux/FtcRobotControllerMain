package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Back Red Auto")
public class realAuto extends LinearOpMode {
    DcMotor backRightMotor;
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    BNO055IMU imu;

    int turn, mode, maxDrivePower, strafe;
    long duration;
    boolean isShooting;
    double shootPower, forward;

    public void inititalSetup(){
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        isShooting = false;
    }

    public void pickMode(){
        autoDrive();
    }

    public void driveToGoal(){
        forward = 1;
        processInputsAndSleep(800);
        turn = -1;
        processInputsAndSleep(220);
        sleep(50);
    }

    public void autoDrive(){
        driveToGoal();
    }

    public void processInputsAndSleep(long duration){
        processDriveInputs();

        sleep(duration);
        forward = 0;
        turn = 0;
        strafe = 0;
        processDriveInputs();
    }

    public void processDriveInputs(){
        turn = turn * maxDrivePower;
        forward = forward * 0.5;
        strafe = strafe * maxDrivePower;

        backLeftMotor.setPower(forward);
        frontLeftMotor.setPower(forward);
        backRightMotor.setPower(forward);
        frontRightMotor.setPower(forward);

        forward = 0;
        sleep(300);
    }

    @Override
  public void runOpMode() {
         backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
         backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

         frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
         frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        inititalSetup();
        shootPower = 0.8;
        maxDrivePower = 1;
        mode = 2;
        waitForStart();
        pickMode();
    }

}
