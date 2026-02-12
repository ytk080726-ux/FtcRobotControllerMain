package org.firstinspires.ftc.teamcodev2;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


import java.nio.channels.FileLock;

public class RobotUtil {
    private static Limelight3A limelight;
    static DcMotor frontLeftMotor;
    static DcMotor backLeftMotor;
    static DcMotor frontRightMotor;
    static DcMotor backRightMotor;
    static aprilthing april;
    LLResult llresult;
    double denominator;
    private double targetX;
    static IMU imu;

    private blocking blocking;



    public void maping(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        limelight.start();


        intaking intake = new intaking();
        intake.init(hardwareMap);

        transfer transfer = new transfer();
        transfer.init(hardwareMap);

        blocking blocker = new blocking();
        blocker.init(hardwareMap);

        launching launch = new launching();
        launch.init(hardwareMap);


        april = new aprilthing();
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
            double first = (up - 0.3) / (Math.tan(Math.toRadians(20 + ty)));
            scale=first;
            telemetry.addData("ty",ty);
        }
        return (scale);
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

    public void shoot() {
        if(llresult != null && llresult.isValid()&&getDistance()!=0) {
            if (targetX > 6)
            {
                frontRightMotor.setPower(-0.35);
                frontLeftMotor.setPower(0.35);
                backLeftMotor.setPower(0.35);
                backRightMotor.setPower(-0.35);
            }
            else if (targetX < 2.5)
            {
                frontRightMotor.setPower(0.35);
                frontLeftMotor.setPower(-0.35);
                backLeftMotor.setPower(-0.35);
                backRightMotor.setPower(0.35);
            }
            else {
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);

            }
        }

        blocking.stopping(true);

    }

}
