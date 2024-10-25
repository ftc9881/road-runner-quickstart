package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Hook Auto", group="Roadrunner")
public class HookAuto extends SimpleAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap, 200, -1, 1);

        BackClaw backClaw = new BackClaw(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // TODO: Set this to the correct initial position

        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Actions.runBlocking(backClaw.closeClaw());

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(backClaw.closeClaw())
                        .build()
        );

        waitForStart();

        Action getInPosition = drive.actionBuilder(beginPose)
                .lineToX(-16)
                .waitSeconds(1)
                .stopAndAdd(lift.liftPosition(460, 0))
                .waitSeconds(1)
                .lineToX(-20)
                .stopAndAdd(lift.liftPosition(400, 0))
                .stopAndAdd(backClaw.openClaw())
                .lineToX(-18)
                .waitSeconds(1)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        getInPosition
                )
        );
    }
}
