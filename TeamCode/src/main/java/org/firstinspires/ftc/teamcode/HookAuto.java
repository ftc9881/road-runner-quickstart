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
        Lift lift = new Lift(hardwareMap);

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
                .afterTime(0, lift.liftPosition(575))
                .lineToX(-24.5)
                .waitSeconds(0.07)
                .stopAndAdd(lift.liftPosition(390))
                .afterTime(0.03, backClaw.openClaw())
                .build();

        Action Park = drive.actionBuilder(beginPose)

/*===========================================================================================================
                Parks (duh)
===========================================================================================================*/
                .strafeTo(new Vector2d(0, -62))
                .afterTime(0.03, lift.liftPosition(0))
                .turnTo(-1.1*Math.PI/2)
                .strafeTo(new Vector2d(0, -70))
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        getInPosition,
                        Park
                )
        );
    }
}
