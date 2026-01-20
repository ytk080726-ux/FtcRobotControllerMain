package org.firstinspires.ftc.teamcodev2;

public class aprilthing {
    private double angle, distance;

    public double getDistance(double ang,double dis2) {
        double dis1 = 0.1135;
        angle = 90 - ang;
        distance = Math.sqrt(Math.pow(dis2, 2) + Math.pow(dis1, 2) - 2 * dis1 * dis2 * Math.cos(Math.toRadians(angle)));
        return distance;
    }

    public double getAngle(double ang1, double dis1,double dis2)
    {
        double angle;
        angle=Math.toDegrees(Math.asin(dis1*Math.sin(Math.toRadians(90-ang1))/dis2));
        return angle;
    }
}
