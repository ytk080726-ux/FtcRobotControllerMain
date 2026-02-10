package org.firstinspires.ftc.teamcodev2.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcodev2.aprilthing;
import org.firstinspires.ftc.teamcodev2.blocking;
import org.firstinspires.ftc.teamcodev2.intaking;
import org.firstinspires.ftc.teamcodev2.launching;
import org.firstinspires.ftc.teamcodev2.transfer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Front Red Auto (PreLoad)")
public class FrontRedAuto__PreLoad extends LinearOpMode {
    DcMotor backRight;
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    BNO055IMU imu;
    aprilthing april;
    private Limelight3A limelight;

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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        transfer = new transfer();
        launch = new launching();
        intake = new intaking();
        blocking = new blocking();
        april=new aprilthing();

        isShooting = false;
        blocking.init(hardwareMap);
        transfer.init(hardwareMap);
        launch.init(hardwareMap);
        launch.init(hardwareMap);
        intake.init(hardwareMap);
        limelight.start();
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
        launch.auto(500);

        backRight.setPower(-1);
        backLeft.setPower(-0.9);
        frontRight.setPower(-1);
        frontLeft.setPower(-0.9);
        sleep(400);

        launch.auto(2000);
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
        sleep(2500);

        blocking.stopping(true);
        sleep(1500);

        blocking.stopping(false);
        intake.auto2();
        launch.launch(0);

        backRight.setPower(0.3);
        backLeft.setPower(0.6);
        frontRight.setPower(0.3);
        frontLeft.setPower(0.6);
        sleep(900);
    }

    @Override
    public void runOpMode() {
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

    public double getDistance() {
        double up=0.745;
        double scale= 0;
        LLResult llresult = limelight.getLatestResult();

        if(llresult != null && llresult.isValid()) {
            double tx =llresult.getTx();
            scale=april.getDistance(tx,limedistance());
            telemetry.addData("tx",tx);
        }
        return (scale);
    }
    public double limedistance()
    {
        double up=0.745;
        double scale= 0;
        LLResult llresult = limelight.getLatestResult();
        if(llresult != null && llresult.isValid()) {
            Pose3D botPose = llresult.getBotpose();
            Object LimelightHelpers;
            double ty =llresult.getTy();
            double first = (up - 0.277) / (Math.tan(Math.toRadians(20 + ty)));
            scale=first;
            telemetry.addData("ty",ty);
        }
        return (scale);
    }

}
