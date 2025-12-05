package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous(name = "camera")
public class camera extends OpMode  {
    webCam cam = new webCam();
    @Override
    public void init() {
        cam.init(hardwareMap,telemetry);
    }

    @Override
    public void loop() {
        cam.update();
        AprilTagDetection id20=cam.getTagBySpecificId(20);
        AprilTagDetection id21=cam.getTagBySpecificId(21);
        AprilTagDetection id22=cam.getTagBySpecificId(22);
        AprilTagDetection id23=cam.getTagBySpecificId(23);
        AprilTagDetection id24=cam.getTagBySpecificId(24);
        cam.displayDetectionTelemetry(id20);
        cam.displayDetectionTelemetry(id21);
        cam.displayDetectionTelemetry(id22);
        cam.displayDetectionTelemetry(id23);
        cam.displayDetectionTelemetry(id24);
        cam.toString();
    }
}
