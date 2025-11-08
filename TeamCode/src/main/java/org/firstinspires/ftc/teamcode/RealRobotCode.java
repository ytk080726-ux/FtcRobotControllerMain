package org.firstinspires.ftc.teamcode;//import com.qualcomm.hardware.bosch.BNO055IMU;

import static java.lang.Math.abs;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class RealRobotCode extends OpMode {

    DcMotor driveBL;
    DcMotor driveFL;
    DcMotor driveBR;
    DcMotor driveFR;
    DcMotor shootWheel;
    DcMotor elevatorMotor;
    DcMotor intakeMotor;
    //Servo openGate;
    CRServo shootServoL;
    CRServo shootServoR;
    Servo intakeServoR;
    Servo intakeServoL;
    static double INTAKE_SERVO_OPEN_L = 0.35 -0.25;
    static double INTAKE_SERVO_OPEN_R = 0.8;
    //static double INTAKE_SERVO_CLOSED_L = 0.50 -0.25 +0.05 +0.1-0.2-0.1;
    static double INTAKE_SERVO_CLOSED_L = INTAKE_SERVO_OPEN_L + 0.2;
    //static double INTAKE_SERVO_CLOSED_R = 0.4 +0.1+0.2+0.3-0.2;
    static double INTAKE_SERVO_CLOSED_R = INTAKE_SERVO_OPEN_R - 0.2;
    static double SHOOT_SERVO_CLOSED = 0.00;
    static double SHOOT_SERVO_OPEN = 1.00;
    //ColorSensor lookForBalls;
    ColorSensor color1;
    //DistanceSensor checkThedistance;
    DistanceSensor distance1;

    //BNO055IMU imu;
    IMU imu;

    //VisionPortal myVisionPortalBuilder;
            int nArtifacts;
            //AprilTagProcessor myAprilTagDetections;
            //VisionPortal myVisionPortal;
            double horizontalInput;
            //AprilTagProcessor myAprilTagDetection;
            static double SHOOT_POWER = 0.8;
            boolean isShooting;
            double verticalInput;
            double dpadY;
            double dpadX;

            //AprilTagProcessor myApriltagProcessor;
            static double MAX_DRIVE_POWER = 0.5;
            //int myAprilTagProcessorBuilder;
            int mode;
            
    //VisionPortal whatDoesThisDo;
    //WebcamName yourNameIsJamal;



/*
    @Override
    public void init() {

    }

 */


/*    // Describe this function...
    public void initializeVisionPortal(){
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortal = (myVisionPortalBuilder.build());
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myApriltagProcessor = (myAprilTagProcessorBuilder.build());
        myVisionPortalBuilder.addProcessor(myApriltagProcessor);
    }
*/
    // Describe this function...
    public void sleepConfig(long milliseconds){
        //sleep(milliseconds);
    }

    public boolean opModeIsActiveConfig() {
        //return( opModeIsActive() );
        return( false );
    }
    public void intakeOn(){
        intakeServoR.setPosition(INTAKE_SERVO_OPEN_R);
        intakeServoL.setPosition(INTAKE_SERVO_OPEN_L);
        intakeMotor.setPower(0.25);
        sleepConfig(200);
        intakeMotor.setPower(0.40);
        sleepConfig(200);
        intakeMotor.setPower(0.50);

    }
    public void intakeOff(){
        intakeServoR.setPosition(INTAKE_SERVO_CLOSED_R);
        intakeServoL.setPosition(INTAKE_SERVO_CLOSED_L);
        intakeMotor.setPower(0.00);
        shootWheel.setPower(0);
        elevatorMotor.setPower(0);
    }
    public void inititalSetup(){
        // Put initialization blocks here
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBR.setDirection(DcMotor.Direction.REVERSE);

        //elevatorMotor.setDirection(DcMotor.Direction.REVERSE);

        isShooting = false;
        // Holds back artifacts until we start shooting
        //shootServoL.setPosition(SHOOT_SERVO_CLOSED);
        //shootServoR.setPosition(SHOOT_SERVO_CLOSED);
    }

    // Describe this function...
    public void pickMode(){
        // Switch based on the mode variable
        if (mode == 0) {
            gamepadDrive();
        } else if (mode == 1) {
            autoDrive();
        }
    }
/*
    // Describe this function...
    public void keyboardDrive(){
        //while (opModeIsActiveConfig()) {
            // Convert keyboard input to a final direction value
            horizontalInput = keyboard.isPressed(108) - keyboard.isPressed(106);
            verticalInput = keyboard.isPressed(105) - keyboard.isPressed(107);
            processDriveInputs();
            if (keyboard.isPressed(112) && !isShooting) {
                shoot();
            }
            displayVisionPortalData();
        //}
    }
*/
    // Describe this function...
    public void gamepadDrive() {
    /*
        public volatile float	left_stick_x
        public volatile float	left_stick_y
        public volatile float	right_stick_x
        public volatile float	right_stick_y
        public volatile boolean	dpad_up
        public volatile boolean	dpad_down
        public volatile boolean	dpad_left
        public volatile boolean	dpad_right
        public volatile boolean	a
        public volatile boolean	b
        public volatile boolean	x
        public volatile boolean	y
    */
        //while (opModeIsActiveConfig()) {
            horizontalInput = gamepad1.left_stick_x;
            verticalInput = gamepad1.left_stick_y;

            if ((abs(horizontalInput) > 0.05) || (abs(verticalInput) > 0.05)) {
                processDriveInputs();
            } else {
                if (gamepad1.dpad_up) {
                    dpadY =  1.00;
                }
                if (gamepad1.dpad_down) {
                    dpadY = -1.00;
                }
                if (gamepad1.dpad_left) {
                    dpadX = -1.00;
                }
                if (gamepad1.dpad_right) {
                    dpadX =  1.00;
                }
                horizontalInput = dpadX;
                verticalInput = dpadY;
                processDriveInputs();
            }

            if (gamepad1.aWasPressed() && !isShooting) {
                shoot();
            }
            if (gamepad1.yWasPressed() && !isShooting) {
                intakeOn();
            }
            if (gamepad1.bWasPressed()) {
                intakeOff();
            }
            //if (gamepad1.left_trigger())
        //}


            //displayVisionPortalData();
                double speedForward = -gamepad1.left_stick_y;
                telemetry.addData("left joyStick x axis", gamepad1.left_stick_x);
                telemetry.addData("left joystick y Axis", speedForward);
                telemetry.addData( "A button", gamepad1.a);

    }


    // Describe this function...
    public void autoDrive(){
        driveToGoal();
        shootThreeArtifacts();
        driveToLoadingSpotAndBack();
        shootThreeArtifacts();
        driveToLoadingSpotAndBack();
        shootThreeArtifacts();
        gamepadDrive();
    }
    public void turnLeft(int timeMs, double maxPower, boolean stopNow){
        driveBL.setPower(maxPower * -1.0);
        driveFL.setPower(maxPower * -1.0);
        driveBR.setPower(maxPower *  1.0);
        driveFR.setPower(maxPower *  1.0);
        sleepConfig(timeMs);
        if (stopNow == true) {
            stop(500);
        }
    }
    public void turnLeft45Degrees(){
        turnLeft(500,0.25,true);
    }
    public void turnRight(int timeMs, double maxPower, boolean stopNow){
        driveBL.setPower(maxPower *  1.0);
        driveFL.setPower(maxPower *  1.0);
        driveBR.setPower(maxPower * -1.0);
        driveFR.setPower(maxPower * -1.0);
        sleepConfig(timeMs);
        if (stopNow == true)
            stop(500);
    }
    public void turnRight45Degrees(){
        turnRight(500, 0.25,true);
    }
    public void moveForward(int timeMs, double maxPower, boolean stopNow){
        driveBL.setPower(maxPower);
        driveFL.setPower(maxPower);
        driveBR.setPower(maxPower);
        driveFR.setPower(maxPower);
        sleepConfig(timeMs);
        if (stopNow == true) {
            stop(500);
        }
    }

    public void moveBackward(int timeMs, double maxPower, boolean stopNow){
        driveBL.setPower(maxPower);
        driveFL.setPower(maxPower);
        driveBR.setPower(maxPower);
        driveFR.setPower(maxPower);
        sleepConfig(timeMs);
        if (stopNow == true) {
            stop(500);
        }
    }
    public void stop(int timeMs){
        driveBL.setPower(0);
        driveBR.setPower(0);
        sleepConfig(timeMs);
    }
    // Describe this function...
    public void driveToGoal(){
        moveForward( 250, 0.25, false);
        moveForward( 250, 0.40, false);
        moveForward( 1500, 0.50, false);
        moveForward( 250, 0.40, false);
        moveForward( 250, 0.25, true);
        //moveForward(2400, 1.0, true);

        turnLeft45Degrees();

    }

    // Describe this function...
    public void driveToLoadingSpotAndBack(){
        moveBackward(250, -0.25, false);
        moveBackward(250, -0.40, false);
        moveBackward(500, -0.50, false);
        moveBackward(250, -0.40, false);
        moveBackward(250, -0.25, true);
        // moveBackward(1500, 1.0, true);

        sleepConfig(10000);

        moveForward( 250, 0.25, false);
        moveForward( 250, 0.40, false);
        moveForward( 500, 0.50, false);
        moveForward( 250, 0.40, false);
        moveForward( 250, 0.25, true);
        //moveForward(1500, 1.0, true);
    }

    // Describe this function...
    public void shootThreeArtifacts(){
        nArtifacts = 3;
        while (opModeIsActiveConfig() && nArtifacts > 0) {
            // Put loop blocks here
            if (!isShooting) {
                shoot();
                nArtifacts -= 1;
            }
            //displayVisionPortalData();
        }
    }

    // Describe this function...
    public void processDriveInputs(){

        double Drive = verticalInput * MAX_DRIVE_POWER;
        double Turn = horizontalInput * MAX_DRIVE_POWER;

        // Combine drive and turn for blended motion.
        double leftPower  = Drive + Turn;
        double rightPower = Drive - Turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0)
        {
            leftPower /= max;
            rightPower /= max;
        }

        // Combine inputs to create drive and turn (or both!)
        driveBL.setPower(leftPower);
        driveFL.setPower(leftPower);
        driveBR.setPower(rightPower);
        driveFR.setPower(rightPower);

    }

    // Describe this function...
    public void shoot(){
        // Don"t move while shooting
        stop(0);
        isShooting = true;
        // Let one artifact come through
        // shootServoL.setPosition(SHOOT_SERVO_OPEN);
        shootServoL.setPower(1.0);
        // shootServoR.setPosition(SHOOT_SERVO_OPEN);
        shootServoR.setPower(1.0);
        shootWheel.setPower(SHOOT_POWER);
        //sleepConfig(1000);//lets the fly wheel spin up to speed
        elevatorMotor.setPower(0.75);
        //sleepConfig(500);//sends one ball to the fly wheel
        //elevatorMotor.setPower(0);
        // Stop the next artifact
        //shootServoL.setPosition(SHOOT_SERVO_CLOSED);
        //shootServoR.setPosition(SHOOT_SERVO_CLOSED);
        //sleepConfig(200);
        //shootWheel.setPower(0);
        //sleepConfig(1500);
        // Allow for a new shot to be triggered
        //isShooting = false;
    }
/*
    // Describe this function...
    public void displayVisionPortalData(){
        myAprilTagDetections = (myApriltagProcessor.getDetections());
        for (String myAprilTagDetection2 : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection2;
            telemetry.addData("ID", (myAprilTagDetection.id));
            telemetry.addData("Range", (myAprilTagDetection.ftcPose.range));
            telemetry.addData("Yaw", (myAprilTagDetection.ftcPose.yaw));
        }
        telemetry.update();
    }

 */


    //@Override
    public void runOpMode() {
        driveBL = hardwareMap.get(DcMotor.class, "backLeftMotor");
        driveFL = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        driveFR = hardwareMap.get(DcMotor.class, "frontRightMotor");
        driveBR = hardwareMap.get(DcMotor.class, "backRightMotor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "transfer");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        shootWheel = hardwareMap.get(DcMotor.class, "flyWheel");
        shootServoL = hardwareMap.get(CRServo.class, "servoflyAL");
        shootServoR = hardwareMap.get(CRServo.class, "servoflyAR");
        intakeServoL = hardwareMap.get(Servo.class, "servoAL");
        intakeServoR = hardwareMap.get(Servo.class, "servoAR");
        shootServoL.setDirection(DcMotorSimple.Direction.REVERSE);
        shootServoR.setDirection(DcMotorSimple.Direction.REVERSE);
        //color1 = hardwareMap.get(ColorSensor.class, "color1");
        //distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = hardwareMap.get(IMU.class, "imu");
        inititalSetup();
        //initializeVisionPortal();

        // 0 = gamepad, 1 = autonomous
        mode = 0;
        //waitForStart();
        while (opModeIsActiveConfig()) {
            pickMode();
        }
    }

    @Override
    public void init() {
      runOpMode();
    }

    @Override
    public void loop() {
        //loop runs at 50 Hertz (i.e. 50 times per second)
        gamepadDrive();
    }

}
