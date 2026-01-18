package org.firstinspires.ftc.teamcodev2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;


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
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    public void starting()
    {
        limelight.start();
    }
    public void stopping()
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
        else
        {
            telemetry.addData("lielight","no");
        }
    }


    public double getDistance() {
        //change bot pose to flucial not mt2 turn the degree to angle
        update();
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
    //m should be 0.4m
    public boolean correct(double m)
    {
        update();
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
