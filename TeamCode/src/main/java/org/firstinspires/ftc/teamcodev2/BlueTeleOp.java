package org.firstinspires.ftc.teamcodev2;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "BlueTeleOp")
public class BlueTeleOp extends LinearOpMode {
    private Limelight3A limelight;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    aprilthing april;
    private double targetX;
    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
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

        waitForStart();

        if (isStopRequested()) return;

        intaking intake = new intaking();
        intake.init(hardwareMap);

        transfer transfer = new transfer();
        transfer.init(hardwareMap);

        blocking blocker = new blocking();
        blocker.init(hardwareMap);

        launching launch = new launching();
        launch.init(hardwareMap);

        lifting lift = new lifting();
        lift.init(hardwareMap);

        april=new aprilthing();
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.aWasPressed()) {
                intake.type();
            }

            if (gamepad1.yWasPressed()) {
                intake.reverse();
            }

            if(gamepad1.xWasPressed()) {
                transfer.start();
            }

            blocker.stopping(gamepad1.b);

            launch.launch(getDistance());
            telemetry.update();
            telemetry.addData("mode",launch.state());
            telemetry.addData("state",launch.giveState());
            telemetry.addData("Speed",launch.showRPM());
            telemetry.addData("Distance: ",getDistance());
            telemetry.addData("limelight",limedistance());

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llresult = limelight.getLatestResult();

            if(llresult != null && llresult.isValid()) {
                llresult = limelight.getLatestResult();
                targetX = llresult.getTx();
                telemetry.addData("TX", targetX);
                telemetry.update();
            }
            else
            {
                telemetry.addData("lielight","no");
            }

            if(gamepad1.dpadLeftWasPressed())
            {
                launch.decrease();
            }
            if(gamepad1.dpadRightWasPressed())
            {
                launch.increase();
            }
            if(gamepad1.left_trigger>0)
            {
                imu.resetYaw();
            }
            if(gamepad1.dpadUpWasPressed())
            {
                launch.setState();
            }
            if(gamepad1.rightBumperWasPressed())
            {
                launch.settingState();
            }

            if(gamepad1.left_bumper)
            {
                if(llresult != null && llresult.isValid()&&getDistance()!=0) {
                    if (targetX > 7)
                    {
                        frontRightMotor.setPower(-0.5);
                        frontLeftMotor.setPower(0.5);
                        backLeftMotor.setPower(0.5);
                        backRightMotor.setPower(-0.5);
                    }
                    else if (targetX < 5)
                    {
                        frontRightMotor.setPower(0.5);
                        frontLeftMotor.setPower(-0.5);
                        backLeftMotor.setPower(-0.5);
                        backRightMotor.setPower(0.5);
                    }
                    frontRightMotor.setPower(0);
                    frontLeftMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                    gamepad1.rumble(100);
                }
            }
            telemetry.update();
        }
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