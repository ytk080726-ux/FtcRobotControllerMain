package org.firstinspires.ftc.teamcodev2.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcodev2.BlueTeleOp;
import org.firstinspires.ftc.teamcodev2.RobotUtil;
import org.firstinspires.ftc.teamcodev2.intaking;
import org.firstinspires.ftc.teamcodev2.pedroPathing.Constants;


@Autonomous(name = "BlueAuto")
public class BlueAuto extends OpMode{
    private RobotUtil robotUtil;

    private intaking intake;

    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public enum PathState {
        // START POSITION_END POSITION
        // Drive > MOVEMENT STATE

        DRIVE_STARTPOS_SHOOT_POS,

        SHOOT_PRELOAD,

        DRIVE_SHOOTPOS_COLLECT_POS,

        DRIVE_SHOOTPOS
    }

    PathState pathState;

    private final Pose startPose = new Pose(21.259668508287287,123.09392265193372, Math.toRadians(143));
    private final Pose shoot = new Pose(58.9171270718232, 84.5303867403315, Math.toRadians(143));

    private final Pose collect = new Pose(17.596685082872927, 84.12707182320442, Math.toRadians(180));

    private final Pose shoot2 = new Pose(17.596685082872927, 84.12707182320442, Math.toRadians(0));

    private PathChain driveStartPosShootPos, driveShootPosCollectPos, driveShootPos;


    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot.getHeading())
                .build();

        driveShootPosCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(shoot, collect))
                .setLinearHeadingInterpolation(shoot.getHeading(), collect.getHeading())
                .build();

        driveShootPos = follower.pathBuilder()
                .addPath(new BezierLine(collect, shoot2))
                .setLinearHeadingInterpolation(collect.getHeading(), shoot2.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); // reset timer & make new state
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    // ADD FLYWHEEL LOGIC
                    robotUtil.shoot();

                    follower.followPath(driveShootPosCollectPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_COLLECT_POS);
                }
                break;
            case DRIVE_SHOOTPOS_COLLECT_POS:
                if (!follower.isBusy()) {
                    // ADD INTAKE LOGIC
                    intake.auto2();

                    follower.followPath(driveShootPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS);
                }
                break;
            case DRIVE_SHOOTPOS:
                if (!follower.isBusy()) {
                    // ADD FLYWHEEL LOGIC
                    robotUtil.shoot();

                    telemetry.addLine("Done all paths");
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
        follower = Constants.createFollower(hardwareMap);
        robotUtil=new RobotUtil();
        intake=new intaking();
        robotUtil.maping(hardwareMap);
        double distance = robotUtil.limedistance();
        // add in other init mechanisms
        intake.init(hardwareMap);
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
