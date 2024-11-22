package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Bucket Auto", group="Roadrunner")
public class BucketAuto extends SimpleAuto {
    public static class Params {
        public double waitTime = .1;
    }

    public static BucketAuto.Params PARAMS = new BucketAuto.Params();

    public BucketAuto() {
        super();
        FlightRecorder.write("BUCKET_AUTO_PARAMS", PARAMS);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // TODO: Set this to the correct initial position

        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(arm.frontArm());

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(arm.frontArm())
                        .waitSeconds(1)
                        .stopAndAdd(claw.closeClaw())
                        .waitSeconds(1)
                        .build()
        );

        waitForStart();

        Action cornerPosition = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, 8))
                .waitSeconds(PARAMS.waitTime)
                .lineToX(-13)
                .waitSeconds(PARAMS.waitTime)
                .turn(Math.PI / 4)
                .build();

        Action depositSample = drive.actionBuilder(beginPose)
                .stopAndAdd(lift.liftPosition(920))
                .stopAndAdd(arm.backArm())
                .waitSeconds(1)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(1)
                .stopAndAdd(arm.frontArm())
                .waitSeconds(1)
//                .stopAndAdd(claw.closeClaw())
//                .waitSeconds(1)
                .stopAndAdd(lift.liftPosition(0))
                .waitSeconds(1)
                .build();

        Action parkSpline = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(5, 75), Math.PI / 2)
                .turn(- Math.PI / 2)
                .lineToX(12)
                .stopAndAdd(extendo.forwardExtendo())
                .waitSeconds(2)
                .build();

        Action parkSimple = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(6, 14), Math.PI / 2)
                .lineToY(51)
                .turn(+ Math.PI / 2)
                .lineToX(15)
                .stopAndAdd(arm.midArm())
                .waitSeconds(2)
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        cornerPosition,
                        depositSample,
                        parkSimple
                )
        );
    }
}
