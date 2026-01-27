package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcodev2.intaking;
import org.firstinspires.ftc.teamcodev2.launching;
import org.firstinspires.ftc.teamcodev2.blocking;
import org.firstinspires.ftc.teamcodev2.transfer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Front Blue Auto")
public class FrontBlueAuto__New extends LinearOpMode {
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
    private blocking blocking;
    intaking intake;
    private VisionPortal visionPortal;
    private AprilTagProcessor AprilTagProcessor;

    public void inititalSetup(){
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        transfer = new transfer();
        launch = new launching();
        intake = new intaking();
        blocking = new blocking();

        isShooting = false;
        blocking.init(hardwareMap);
        transfer.init(hardwareMap);
        launch.init(hardwareMap);
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
        transfer.start();
        intake.auto2();
        backRight.setPower(-forward);
        backLeft.setPower(-forward);
        frontRight.setPower(-forward);
        frontLeft.setPower(-forward);
        sleep(600);

        launch.launch(1.5);
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
        sleep(1000);

        blocking.stopping(true);
        sleep(3000);

        launch.launch(0);
        backRight.setPower(0);
        backLeft.setPower(0.55);
        frontRight.setPower(1);
        frontLeft.setPower(0);
        sleep(1070);

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
