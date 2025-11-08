package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "main")

public class FieldCentricMecanumTeleOpActual extends LinearOpMode {
    intake take = new intake();
    ballStop stop = new ballStop();
    launcherB launch = new launcherB();
    launchU launchUP = new launchU();
    pushing ballpush = new pushing();

    @Override

    public void runOpMode() throws InterruptedException {
        telemetry.update();
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        FieldCentricMecanumTeleOpActual bot = new FieldCentricMecanumTeleOpActual();
        take.init(hardwareMap);
        stop.init(hardwareMap);
        launch.init(hardwareMap);
        launchUP.init(hardwareMap);
        ballpush.init(hardwareMap);
        telemetry.addData("imu", imu.getRobotYawPitchRollAngles());
        double power = 0.8;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.update();
            telemetry.addData("imu", imu.getRobotYawPitchRollAngles());
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
            double frontLeftPower = (rotY + rotX + rx) / 2;
            double backLeftPower = (rotY - rotX + rx) / 2;
            double frontRightPower = (rotY - rotX - rx) / 2;
            double backRightPower = -(rotY + rotX - rx) / 2;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            telemetry.addData("imu", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            telemetry.addData("motor speed",launchUP.speed());
            if (gamepad1.aWasPressed()) {
                take.type();
            }
            if (gamepad1.bWasPressed()) {
                launch.up();
            }
//            if (gamepad1.right_bumper) {
//                launchUP.launchD();
//            }
//            if (gamepad1.left_bumper) {
//                launchUP.launchL();
//            }
//            if (gamepad1.right_trigger > 0.1) {
//                launchUP.launchM();

            if (gamepad1.left_trigger > .3) {
                telemetry.addData("left_trigger pressed amount",gamepad1.left_trigger);
                //launchUP.launch.setPower(0.2);
                launchUP.launch.setPower(0.1);
                //launchUP.highLaunch();
                telemetry.addData("flywheel power" , launchUP.launch.getPower());
                //shooter.setPower(0.2);
//                launchUP.launchH();
            }
//            else if (gamepad1.right_trigger > .3) {
//                telemetry.addData("right_trigger pressed amount",gamepad1.right_trigger );
//                //launchUP.launch.setPower(0.2);
//                launchUP.launch.setPower(0.1);
//                //launchUP.lowLaunch();
//                telemetry.addData("flywheel power" , launchUP.launch.getPower());
//                //shooter.setPower(0.2);
////                launchUP.launchH();
//            }
            else {
                telemetry.addData("right_trigger pressed amount",gamepad1.right_trigger );
                //launchUP.launchLowState = false;
                //launchUP.launch.setPower(0.0);
                launchUP.launch.setPower(gamepad1.right_trigger * 0.15);
                telemetry.addData("flywheel power" , launchUP.launch.getPower());
            }

            ballpush.bp(gamepad1.y);
            if (gamepad1.xWasPressed()) {
                stop.position();
            }
        }
        }
//        if (gamepad1.left_trigger > .0) {
//            telemetry.addData("left_trigger pressed amount",gamepad1.left_trigger );
//            launchUP.upTwo();
//            //shooter.setPower(0.2);
////                launchUP.launchH();
//        }
//        else {
//            telemetry.addData("left_trigger pressed amount",gamepad1.left_trigger );
//            launchUP.launchUState = false;
//        }
//            ballpush.bp(gamepad1.y);
//            if (gamepad1.xWasPressed()) {
//                stop.position();
//            }
//        }
    }

