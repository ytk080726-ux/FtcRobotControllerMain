package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Back Red Auto")
public class realAuto extends LinearOpMode {
    DcMotor backRightMotor;
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor shootwheel;
    Servo artifactstopper;
    ColorSensor color1;
    DistanceSensor distance1;
    BNO055IMU imu;

    int forward, turn, mode, maxDrivePower, strafe;
    long duration;
    VisionPortal myVisionPortal;
    boolean isShooting;
    double shootPower;

    // Describe this function...
    public void inititalSetup(){
        // Put initialization blocks here
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);


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
        backLeftMotor.setPower(forward + turn + strafe);
        frontLeftMotor.setPower(forward - turn + strafe);
        backRightMotor.setPower(forward + turn - strafe);
        frontRightMotor.setPower(forward - turn - strafe);
    }

    @Override
  public void runOpMode() {
        shootwheel = hardwareMap.get(DcMotor.class, "shootwheel");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        //artifactstopper = hardwareMap.get(Servo.class, "artifactstopper");
        //color1 = hardwareMap.get(ColorSensor.class, "color1");
        //distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
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
