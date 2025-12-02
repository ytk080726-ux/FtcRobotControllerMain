package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;

public class realAuto extends LinearOpMode {
    DcMotor driveLeft;
    DcMotor driveRight;
    DcMotor shootwheel;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    Servo artifactstopper;
    ColorSensor color1;
    DistanceSensor distance1;
    BNO055IMU imu;

    long duration;
    int forward;
    int turn;
    VisionPortal myVisionPortal;
    boolean isShooting;
    double shootPower;
    int mode;
    int maxDrivePower;
    int strafe;

    // Describe this function...
    public void inititalSetup(){
        // Put initialization blocks here
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        isShooting = false;
        // Holds back artifacts until we start shooting
        artifactstopper.setPosition(0.2);
    }

    // Describe this function...
    public void initializeVisionPortal(){
        VisionPortal.Builder builder = new VisionPortal.Builder();
        myVisionPortal = builder.build();
    }

    // Describe this function...
    public void pickMode(){
        autoDrive();
    }

    // Describe this function...
    public void autoDrive(){
        driveToGoal();
    }

    // Describe this function...
    public void driveToGoal(){
        forward = 1;
        processInputsAndSleep(800);
        turn = -1;
        processInputsAndSleep(220);
        sleep(500);
    }

    // Describe this function...
    public void processInputsAndSleep(long duration){
        // This helper function makes the code a bit cleaner
        processDriveInputs();
        sleep(duration);
        // Stop all movement after sleep
        forward = 0;
        turn = 0;
        strafe = 0;
        processDriveInputs();
    }

    // Describe this function...
    public void processDriveInputs(){
        turn = turn * maxDrivePower;
        forward = forward * maxDrivePower;
        strafe = strafe * maxDrivePower;
        // Combine inputs to create drive and turn (or both!)
        frontLeftDrive.setPower(forward + turn + strafe);
        frontRightDrive.setPower(forward - turn - strafe);
        backLeftDrive.setPower(forward + turn - strafe);
        backRightDrive.setPower(forward - turn + strafe);
    }

    @Override
    public void runOpMode() {
        driveLeft = hardwareMap.get(DcMotor.class, "driveLeft");
        driveRight = hardwareMap.get(DcMotor.class, "driveRight");
        shootwheel = hardwareMap.get(DcMotor.class, "shootwheel");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        artifactstopper = hardwareMap.get(Servo.class, "artifactstopper");
        color1 = hardwareMap.get(ColorSensor.class, "color1");
        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        inititalSetup();
        initializeVisionPortal();
        shootPower = 0.8;
        maxDrivePower = 1;
        // mode 0 = keyboard, 1 = gamepad, 2 = autonomous
        mode = 2;
        waitForStart();
        pickMode();
    }

}
