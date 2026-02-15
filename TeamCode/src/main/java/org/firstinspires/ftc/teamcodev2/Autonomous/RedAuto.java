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


@Autonomous(name = "RedAuto")
public class RedAuto extends OpMode{
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
        ALIGN_SET_TWO,
        COLLECT_SET_TWO,
        SHOOT_SET_TWO,

        ALIGN_SET_THREE,
        COLLECT_SET_THREE,
        SHOOT_SET_THREE
    }

    PathState pathState;

    private final Pose startPose = new Pose(121.76795580110496,120.97237569060775, Math.toRadians(45));
    private final Pose shoot = new Pose(83.58011049723761, 83.73480662983428, Math.toRadians(45));
    private final Pose repose = new Pose(83.48011049723761,83.63480662983428, Math.toRadians(0));

    private final Pose collect = new Pose(129.38121546961327, 81.57845303867404, Math.toRadians(0));

    private final Pose shoot2 = new Pose(83.75690607734803, 83.85635359116019, Math.toRadians(45));

    private final Pose alignSetTwo = new Pose(83.939226519337, 56.95580110497238, Math.toRadians(0));
    private final Pose collectSetTwo = new Pose(135.7403314917127, 53.917127071823224, Math.toRadians(0));

    private final Pose alignSetThree = new Pose(83.49723756906077, 34.011049723756905, Math.toRadians(0));
    private final Pose collectSetThree = new Pose(132.21546961325967, 34.922651933701644, Math.toRadians(0));

    private final Pose leaveShoot = new Pose(86.78453038674033, 116.77900552486189, Math.toRadians(22));
    private PathChain driveStartPosShootPos, reposition, driveShootPosCollectPos, driveShootPos, alignSetTwoPos, collectSetTwoPos, shootSetTwo, alignSetThreePos, collectSetThreePos, shootSetThree;


    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot.getHeading())
                .build();

        reposition = follower.pathBuilder()
                .addPath(new BezierLine(shoot, repose))
                .setLinearHeadingInterpolation(shoot.getHeading(), repose.getHeading())
                .build();

        driveShootPosCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(repose, collect))
                .setLinearHeadingInterpolation(repose.getHeading(), collect.getHeading())
                .build();

        driveShootPos = follower.pathBuilder()
                .addPath(new BezierLine(collect, shoot2))
                .setLinearHeadingInterpolation(collect.getHeading(), shoot2.getHeading())
                .build();
        alignSetTwoPos = follower.pathBuilder()
                .addPath(new BezierLine(shoot2, alignSetTwo))
                .setLinearHeadingInterpolation(shoot2.getHeading(), alignSetTwo.getHeading())
                .build();

        collectSetTwoPos = follower.pathBuilder()
                .addPath(new BezierLine(alignSetTwo, collectSetTwo))
                .setLinearHeadingInterpolation(alignSetTwo.getHeading(), collectSetTwo.getHeading())
                .build();

        shootSetTwo = follower.pathBuilder()
                .addPath(new BezierLine(collectSetTwo, shoot))
                .setLinearHeadingInterpolation(collectSetTwo.getHeading(), shoot.getHeading())
                .build();
        alignSetThreePos = follower.pathBuilder()
                .addPath(new BezierLine(shoot, alignSetThree))
                .setLinearHeadingInterpolation(shoot.getHeading(), alignSetThree.getHeading())
                .build();

        collectSetThreePos = follower.pathBuilder()
                .addPath(new BezierLine(alignSetThree, collectSetThree))
                .setLinearHeadingInterpolation(alignSetThree.getHeading(), collectSetThree.getHeading())
                .build();

        shootSetThree = follower.pathBuilder()
                .addPath(new BezierLine(collectSetThree, leaveShoot))
                .setLinearHeadingInterpolation(collectSetThree.getHeading(), leaveShoot.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                robotUtil.baseValues();
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); // reset timer & make new state
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    // ADD FLYWHEEL LOGIC

                    if (pathTimer.getElapsedTimeSeconds() < 3) {
                        robotUtil.limedistance();
                        robotUtil.getDistance();
                        robotUtil.shoot();
                    }

                    if (pathTimer.getElapsedTimeSeconds() < 3.5){
                        robotUtil.launch();
                    }
                    else if (pathTimer.getElapsedTimeSeconds() < 5.5){
                        robotUtil.block(true);
                    }
                    else if (pathTimer.getElapsedTimeSeconds() < 6){
                        robotUtil.block(false);
                    }
                    else {
                        follower.followPath(reposition, true);
                        setPathState(PathState.REPOSE);
                    }

                }
                break;

            case REPOSE:
                if (!follower.isBusy()) {

                    follower.followPath(driveShootPosCollectPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_COLLECT_POS);
                }
                break;

            case DRIVE_SHOOTPOS_COLLECT_POS:
                if (!follower.isBusy()) {
                    // ADD INTAKE LOGIC
                    robotUtil.baseValues();
                    follower.followPath(driveShootPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS);
                }
                break;

            case DRIVE_SHOOTPOS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    // ADD FLYWHEEL LOGIC
                    if(pathTimer.getElapsedTimeSeconds() < 1){
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
                        follower.followPath(alignSetTwoPos, true);
                        setPathState(PathState.ALIGN_SET_TWO);
                    }


                }
                break;

            case ALIGN_SET_TWO:
                if (!follower.isBusy()) {
                    // ADD INTAKE LOGIC
                    robotUtil.baseValues();
                    follower.followPath(collectSetTwoPos, true);
                    setPathState(PathState.COLLECT_SET_TWO);
                }
                break;

            case COLLECT_SET_TWO:
                if (!follower.isBusy()) {
                    // ADD INTAKE LOGIC
                    robotUtil.baseValues();
                    follower.followPath(shootSetTwo, true);
                    setPathState(PathState.SHOOT_SET_TWO);
                }
                break;

            case SHOOT_SET_TWO:
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
                        follower.followPath(alignSetThreePos, true);
                        setPathState(PathState.ALIGN_SET_THREE);
                    }
                }
                break;
            case ALIGN_SET_THREE:
                if (!follower.isBusy()) {
                    // ADD INTAKE LOGIC
                    robotUtil.baseValues();
                    follower.followPath(collectSetThreePos, true);
                    setPathState(PathState.COLLECT_SET_THREE);
                }
                break;
            case COLLECT_SET_THREE:
                if (!follower.isBusy()) {
                    // ADD INTAKE LOGIC
                    robotUtil.baseValues();
                    follower.followPath(shootSetThree, true);
                    setPathState(PathState.SHOOT_SET_THREE);
                }
                break;
            case SHOOT_SET_THREE:
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
                        telemetry.addLine("All States Finished");
                    }
                }
                break;
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
        robotUtil.mapingRed();
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
