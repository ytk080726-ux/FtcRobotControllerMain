package org.firstinspires.ftc.teamcodev2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class apriltag {
    private Limelight3A limelight;
    private IMU imu;

    private double Tx;
    private double Ty;
    private double Ta;

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    public void start() {
        limelight.start();
    }

    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        if(llresult != null && llresult.isValid()) {
            Ta = llresult.getTa();
            Tx = llresult.getTx();
            Ty = llresult.getTy();
        }
    }
    public double getTa() {
        loop();
        return Ta;
    }
    public double getTx() {
        loop();
        return Tx;
    }
    public double getTy() {
        loop();
        return Ty;
    }

    public double getDistance() {
        loop();
        double scale = 30665.95;
        return (scale/Ta);
    }
}
