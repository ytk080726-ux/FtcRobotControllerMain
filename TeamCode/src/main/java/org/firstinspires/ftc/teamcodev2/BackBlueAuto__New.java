package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Back Blue Auto")
public class BackBlueAuto__New extends LinearOpMode {
    DcMotor backRight;
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    BNO055IMU imu;

    int turn, mode, maxDrivePower, strafe;
    long duration;
    boolean isShooting;
    double shootPower, forward;
    transfer transfer;
    launching launch;
    intake intake;
    blocking blocker;
    private VisionPortal visionPortal;
    private AprilTagProcessor AprilTagProcessor;

    public void inititalSetup(){
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        transfer = new transfer();
        launch = new launching();
        intake = new intake();
        blocker = new blocking();
        isShooting = false;
        blocker.init(hardwareMap);
        transfer.init(hardwareMap);
        launch.init(hardwareMap);
        intake.init(hardwareMap);
    }

    public void pickMode(){
        autoDrive();
    }

    public void driveToGoal(){
        forward = 1;
        processDriveInputs();
        sleep(50);
    }

    public void autoDrive(){
        driveToGoal();
    }

    public void processDriveInputs() {
        turn = turn * maxDrivePower;
        forward = forward * 0.7;
        strafe = strafe * maxDrivePower;
        transfer.start();

        launch.launch();
        sleep(500);
        blocker.stopping(true);
        sleep(2000);

        backRight.setPower(0);
        backLeft.setPower(0.55);
        frontRight.setPower(1);
        frontLeft.setPower(0);
        sleep(500);

        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
        sleep(100);
    }

    @Override
  public void runOpMode() {
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        visionPortal = (builder.build());
//        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        AprilTagProcessor.Builder AprilTagBuilder = new AprilTagProcessor.Builder();
//        AprilTagProcessor = (AprilTagBuilder.build());
//        builder.addProcessor(AprilTagProcessor);


         backLeft = hardwareMap.get(DcMotor.class, "backLeft");
         backRight = hardwareMap.get(DcMotor.class, "backRight");

         frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
         frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        inititalSetup();
        shootPower = 0.8;
        maxDrivePower = 1;
        mode = 2;
        waitForStart();
        pickMode();
    }

}
