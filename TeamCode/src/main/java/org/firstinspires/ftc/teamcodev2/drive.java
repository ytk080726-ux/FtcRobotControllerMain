package org.firstinspires.ftc.teamcodev2;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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

import java.util.List;

@TeleOp(name = "(new) driveV2")
public class drive extends LinearOpMode {
    private Limelight3A limelight;
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
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

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

            if (gamepad1.rightBumperWasPressed()) {
                launch.launch();
            }

            telemetry.addData("mode",launch.state());
            telemetry.update();
            telemetry.addData("Speed",launch.showRPM());
            updating();
            telemetry.addData("Distance: ",getDistance());


            if (gamepad1.dpadUpWasPressed()) {
                lift.up();
            }

            if (gamepad1.dpadDownWasPressed()) {
                lift.down();
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
        }
        telemetry.update();
    }
    public void updating() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        if(llresult != null && llresult.isValid()) {
            Pose3D botPose = llresult.getBotpose_MT2();
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Ty", llresult.getTy());
            telemetry.addData("Ta", llresult.getTa());
            telemetry.addData("bot pose",botPose.toString());
            telemetry.addData("yaw",botPose.getOrientation().getYaw(AngleUnit.DEGREES));
        }
        else
        {
            telemetry.addData("lielight","no");
        }
    }
    public double getDistance() {
        //change bot pose to flucial not mt2 turn the degree to angle
        updating();
        double up=75;
        double scale= 0;
        LLResult llresult = limelight.getLatestResult();
        if(llresult != null && llresult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = llresult.getFiducialResults();
            Pose3D botPose = llresult.getBotpose_MT2();
            double first = (up - 27.7) / (Math.tan(20 + llresult.getTy()));
            double second=Math.sqrt(Math.pow(botPose.getPosition().x,2)+Math.pow(botPose.getPosition().y,2));
            scale=(first+second)/2;
        }
        return (scale);
    }
    public boolean correct(double m)
    {
        updating();
        double up=75;
        LLResult llresult = limelight.getLatestResult();
        if(llresult != null && llresult.isValid()&&getDistance()!=0) {
            Pose3D botPose = llresult.getBotpose_MT2();
            double degree1,degree2,degree3,degree4,tl;
            degree1=llresult.getTx();
            degree2=Math.acos(botPose.getPosition().y/getDistance());
            degree3=90+degree2;
            tl=Math.sqrt(Math.pow(getDistance(),2)+Math.pow(m,2)-2*m*getDistance()*Math.cos(degree3));
            degree4=Math.asin(m*Math.sin(degree3)/tl);
            return (degree1<degree4);
        }
        return false;
    }
    public double realpower(double m)
    {
        //change bot pose to flucial not mt2 turn the degree to angle
        LLResult llresult = limelight.getLatestResult();
        Pose3D botPose = llresult.getBotpose_MT2();
        double power= 0;
        if(correct(m))
        {
            double degree1,degree2,degree3,degree4,degree5,k,realdistahce;
            degree1=llresult.getTx();
            degree2=Math.acos(botPose.getPosition().y/getDistance());
            degree3=90+degree2;
            realdistahce=Math.sqrt(Math.pow(getDistance(),2)+Math.pow(m,2)-2*m*getDistance()*Math.cos(degree3));
            degree4=(90-degree2-degree1);
            degree5=180-degree4;
            k=Math.sin(degree5)*m/Math.cos(degree4);
            power=k+realdistahce;
        }
        return power;
    }
}