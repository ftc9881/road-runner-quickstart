package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Bucket Auto", group="Roadrunner")
public class BucketAuto extends SimpleAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap, 400, -.5, .5);
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
                .waitSeconds(.1)
                .lineToX(-13)
                .waitSeconds(.1)
                .turn(Math.PI / 4)
                .build();

        Action depositSample = drive.actionBuilder(beginPose)
                .stopAndAdd(lift.liftPosition(920, 0.5))
                .waitSeconds(1)
                .stopAndAdd(arm.backArm())
                .waitSeconds(1)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(1)
                .stopAndAdd(arm.frontArm())
                .waitSeconds(1)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(1)
                .stopAndAdd(lift.liftPosition(0, 0))
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
                .lineToY(70)
                .turn(- Math.PI / 2)
                .lineToX(12)
                .stopAndAdd(extendo.forwardExtendo())
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
