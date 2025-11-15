package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class sensor {
    private DistanceSensor dis;
    private NormalizedColorSensor color;
    private float normRed, normGreen, normBlue;

    public enum detectedColor {
        Green,
        Purple,
        UNKNOWN
    }

    public void init(HardwareMap hw)
    {
        dis=hw.get(DistanceSensor.class,"distance");
        color=hw.get(NormalizedColorSensor.class,"color");
        color.setGain(4);
    }

    public detectedColor getDetectedColor() {

        if(normRed<0.3&&normBlue<0.4&&normGreen<0.5&&normRed>0.2&&normBlue>0.3&&normGreen>0.4)
        {
            return detectedColor.Green;
        } else if (normGreen<0.4&&normBlue<0.4&&normRed<0.4&&normBlue>0.3&&normGreen>0.3&&normRed>0.3) {
            return detectedColor.Purple;
        }
        else
            return detectedColor.UNKNOWN;
    }
    public double distance()
    {
        return dis.getDistance(DistanceUnit.MM);
    }
    public void output(Telemetry telemetry)
    {
        NormalizedRGBA colors = color.getNormalizedColors();
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);
    }

}
