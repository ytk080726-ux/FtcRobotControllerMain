package org.firstinspires.ftc.teamcodev2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class apriltag {
    private Limelight3A limelight;
    private IMU imu;

    private double Tx;
    private double Ty;
    private double Ta;

    public void init(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);
        imu = hw.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    public void start()
    {
        limelight.start();
    }
    public void stop()
    {
        limelight.stop();
    }

    public void update() {
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
    }
    public double getTa() {
        update();
        return Ta;
    }
    public double getTx() {
        update();
        return Tx;
    }
    public double getTy() {
        update();
        return Ty;
    }

    public double getDistance() {
        update();
        double up=75;
        double scale = (up-27.7)/(Math.tan(20+getTy()));
        return (scale);
    }
}
