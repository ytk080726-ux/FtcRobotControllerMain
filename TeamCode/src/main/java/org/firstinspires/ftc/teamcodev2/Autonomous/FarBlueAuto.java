package org.firstinspires.ftc.teamcodev2.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcodev2.BlueTeleOp;
import org.firstinspires.ftc.teamcodev2.RobotUtil;
import org.firstinspires.ftc.teamcodev2.intaking;
import org.firstinspires.ftc.teamcodev2.pedroPathing.Constants;


@Autonomous(name = "FarBlueAuto")
public class FarBlueAuto extends OpMode{
    private RobotUtil robotUtil;

    private intaking intake;

    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public enum PathState {
        // START POSITION_END POSITION
        // Drive > MOVEMENT STATE

        DRIVE_STARTPOS_SHOOT_POS,

        SHOOT_PRELOAD,
        REPOSE,

        DRIVE_SHOOTPOS_COLLECT_POS,

        DRIVE_SHOOTPOS,

        LEAVE
    }

    PathState pathState;

    private final Pose startPose = new Pose(121.76795580110496,120.97237569060775, Math.toRadians(90));
    private final Pose shoot = new Pose(52.552486187845304, 12.928176795580102, Math.toRadians(100));

    private final Pose startPose2 = new Pose(121.66795580110496,123.19392265193372, Math.toRadians(180));
    private final Pose collect = new Pose(9.58011049723757, 7.1712707182320425, Math.toRadians(180));


    private PathChain driveStartPosShootPos, reposition, driveShootPosCollectPos, driveShootPos, leave;


    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot.getHeading())
                .build();

        reposition = follower.pathBuilder()
                .addPath(new BezierLine(shoot, startPose2))
                .setLinearHeadingInterpolation(shoot.getHeading(), startPose2.getHeading())
                .build();

        driveShootPosCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, collect))
                .setLinearHeadingInterpolation(startPose.getHeading(), collect.getHeading())
                .build();

        driveShootPos = follower.pathBuilder()
                .addPath(new BezierLine(collect, shoot))
                .setLinearHeadingInterpolation(collect.getHeading(), shoot.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(shoot, collect))
                .setLinearHeadingInterpolation(shoot.getHeading(), collect.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                robotUtil.baseValues();
                follower.followPath(driveStartPosShootPos, true);

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    // ADD FLYWHEEL LOGIC

                    if (pathTimer.getElapsedTimeSeconds() < 2) {
                        robotUtil.limedistance();
                        robotUtil.getDistance();
                        robotUtil.shoot();
                    }

                    if (pathTimer.getElapsedTimeSeconds() < 2.5){
                        robotUtil.launch();
                    }
                    else if (pathTimer.getElapsedTimeSeconds() < 4.5){
                        robotUtil.block(true);
                    }
                    else if (pathTimer.getElapsedTimeSeconds() < 5){
                        robotUtil.block(false);
                    }
                    else {
                        setPathState(PathState.REPOSE);
                    }

                }
                break;


            case REPOSE:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosCollectPos);
                    setPathState(PathState.DRIVE_SHOOTPOS_COLLECT_POS);
                }
                break;
            case DRIVE_SHOOTPOS_COLLECT_POS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    // ADD INTAKE LOGIC
                    if(pathTimer.getElapsedTimeSeconds() < 4){
                        robotUtil.baseValues();
                    }
                    follower.followPath(driveShootPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS);
                }
                break;
            case DRIVE_SHOOTPOS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    // ADD FLYWHEEL LOGIC
                    if(pathTimer.getElapsedTimeSeconds() < 2){
                        robotUtil.limedistance();
                        robotUtil.getDistance();
                        robotUtil.shoot();
                    }

                    if (pathTimer.getElapsedTimeSeconds() < 3.5){
                        robotUtil.launch();
                    }
                    else if (pathTimer.getElapsedTimeSeconds() < 4.5){
                        robotUtil.block(true);
                    }
                    else if (pathTimer.getElapsedTimeSeconds() < 5){
                        robotUtil.block(false);

                    }
                    else {
                        follower.followPath(leave, true);
                        setPathState(PathState.LEAVE);
                    }
                }
                break;
            case LEAVE:
                if (!follower.isBusy()) {
                    telemetry.addLine("All states Finished");
                }
            default:
                telemetry.addLine("No State Comanded");
                break;
        }
    }


    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();

    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        intake = new intaking();
        intake.init(hardwareMap);


        robotUtil = new RobotUtil(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        robotUtil.mapingBlue();
        // add in other init mechanisms

        buildPaths();
        follower.setPose(startPose);

    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }


    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state ", pathState.toString());
        telemetry.addData("x ", follower.getPose().getX());
        telemetry.addData("y ", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }
}
