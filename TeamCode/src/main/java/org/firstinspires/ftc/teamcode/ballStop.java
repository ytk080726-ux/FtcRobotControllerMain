package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

public class ballStop {
    Servo intakeServoL,intakeServoR;
    boolean state;
    double INTAKE_SERVO_OPEN_L,INTAKE_SERVO_OPEN_R,INTAKE_SERVO_CLOSED_L,INTAKE_SERVO_CLOSED_R;
    public void init(HardwareMap hw)
    {
        intakeServoL = hw.get(Servo.class, "servoAL");
        intakeServoR = hw.get(Servo.class, "servoAR");
        INTAKE_SERVO_OPEN_L = 0.05 ;
        INTAKE_SERVO_OPEN_R = 0.9 ;
        INTAKE_SERVO_CLOSED_L = INTAKE_SERVO_OPEN_L + 0.2;
        INTAKE_SERVO_CLOSED_R = INTAKE_SERVO_OPEN_R - 0.2;
        intakeServoL.setPosition(INTAKE_SERVO_OPEN_L);
        intakeServoR.setPosition(INTAKE_SERVO_OPEN_R);
        state=true;
    }
    public void position()
    {
        if (state) {
            intakeServoL.setPosition(INTAKE_SERVO_OPEN_L);
            intakeServoR.setPosition(INTAKE_SERVO_OPEN_R);
        } else {
            intakeServoL.setPosition(INTAKE_SERVO_CLOSED_L);
            intakeServoR.setPosition(INTAKE_SERVO_CLOSED_R);
        }
        state=!state;
    }
}
