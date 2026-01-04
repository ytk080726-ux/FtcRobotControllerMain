package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Front Red Auto")
public class FrontRedAuto__Old extends LinearOpMode {
    DcMotor backRightMotor;
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    BNO055IMU imu;

    int turn, mode, maxDrivePower, strafe;
    long duration;
    boolean isShooting;
    double shootPower, forward;

    private VisionPortal visionPortal;
    private AprilTagProcessor AprilTagProcessor;
    pushing ballpush;
    sensor detect;
    launcherB launch;
    launchU launchu;
    intake in;

    public void inititalSetup(){
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        isShooting = false;
        ballpush= new pushing();
        launchu=new launchU();
        in = new intake();
        detect=new sensor();
        launch=new launcherB();
        isShooting = false;
        ballpush.init(hardwareMap);
        detect.init(hardwareMap);
        launch.init(hardwareMap);
        launchu.init(hardwareMap);
        in.init(hardwareMap);
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
        forward = forward * 0.3;
        strafe = strafe * maxDrivePower;
        ballpush.set(false);
        if (detect.distance() > 78 && ballpush.getpos() < 0.5) {
            launch.setPower((2800/ 60) * 28); // Start
        }
        else {
            launch.setPower(0); // Stop
        }
        backLeftMotor.setPower(forward);
        frontLeftMotor.setPower(forward);
        backRightMotor.setPower(-forward);
        frontRightMotor.setPower(-forward);
        sleep(300);

        backLeftMotor.setPower(-forward);
        frontLeftMotor.setPower(-forward);
        backRightMotor.setPower(-forward);
        frontRightMotor.setPower(-forward);
        sleep(1000);
        forward = 0;
        backLeftMotor.setPower(-forward);
        frontLeftMotor.setPower(-forward);
        backRightMotor.setPower(-forward);
        frontRightMotor.setPower(-forward);
        ballpush.set(false);
        in.auto();
        sleep(100);
        launchu.auto();
        sleep(670);
        launch.setPower(0);
        ballpush.set(true);
        sleep(1000);
        launch.setPower((2800/ 60) * 28);
        ballpush.set(false);
        sleep(200);
        launch.setPower(0);
        ballpush.set(true);
        sleep(1000);
        launch.setPower((2600/ 60) * 28);
        ballpush.set(false);
        sleep(1000);
        launch.setPower(0);
        ballpush.set(true);
        sleep(1000);
        launch.setPower((2600/ 60) * 28);
        ballpush.set(false);
        sleep(1000);
        launch.setPower(0);
        ballpush.set(true);
        sleep(1000);
        launch.setPower((2600/ 60) * 28);
        ballpush.set(false);
        sleep(1000);
        launch.setPower(0);
        ballpush.set(true);
        sleep(1000);
        launch.setPower((2600/ 60) * 28);
        ballpush.set(false);
        sleep(200);
//        forward=0.3;
//        backLeftMotor.setPower(forward);
//        frontLeftMotor.setPower(forward);
//        backRightMotor.setPower(-forward);
//        frontRightMotor.setPower(-forward);
//        sleep(200);
//        forward=0;
//        backLeftMotor.setPower(forward);
//        frontLeftMotor.setPower(forward);
//        backRightMotor.setPower(-forward);
//        frontRightMotor.setPower(-forward);
    }

//    public void displayVisionPortalData(){
//        AprilTagDetection AprilTagDetection = (AprilTagProcessor.getDetections());
//
//        for (AprilTagDetection myAprilTagDetection2 : AprilTagDetection) {
//            AprilTagDetection = myAprilTagDetection2;
//            telemetry.addData("ID", (AprilTagDetection.id));
//            telemetry.addData("Range", (AprilTagDetection.ftcPose.range));
//            telemetry.addData("Yaw", (AprilTagDetection.ftcPose.yaw));
//        }
//        telemetry.update();
//    }


    @Override
  public void runOpMode() {
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        visionPortal = (builder.build());
//        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        AprilTagProcessor.Builder AprilTagBuilder = new AprilTagProcessor.Builder();
//        AprilTagProcessor = (AprilTagBuilder.build());
//        builder.addProcessor(AprilTagProcessor);


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
