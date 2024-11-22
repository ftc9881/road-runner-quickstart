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

@Autonomous(name="Both Auto", group="Roadrunner")
public class BothAuto extends SimpleAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);

        Claw claw = new Claw(hardwareMap);
        BackClaw backClaw = new BackClaw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);
        Intake intake = new Intake(hardwareMap);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // TODO: Set this to the correct initial position

        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Actions.runBlocking(claw.openClaw());
        Actions.runBlocking(arm.frontArm());


        // TODO: Set this to the correct initial position
        //Sets starting position as (0, 0) [EVERYTHING IS RELATIVE TO THESE POSITION]



        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(backClaw.closeClaw())
                        .afterTime(0, pivot.up())
                        .build()
        );

        waitForStart();

        Action Hook = drive.actionBuilder(beginPose)

/*===========================================================================================================
                Hooks Preloaded Specimen
===========================================================================================================*/

                .afterTime(0, lift.liftPosition(575))
                .lineToX(-26.25)
                .waitSeconds(0.07)
                .stopAndAdd(lift.liftPosition(390))
                .afterTime(0.03, backClaw.openClaw())
                .build();

        Action SampleOne = drive.actionBuilder(beginPose)

/*===========================================================================================================
                Inputs Sample One
===========================================================================================================*/

                .afterTime(0.001, lift.liftPosition(0))
                .splineTo(new Vector2d(-6, -16), -1.018*Math.PI)
                .strafeTo(new Vector2d(-2, -38.5))   // NOTE:  Suggest you set a heading here, using strafeToLinearHeading(), so that it corrects the heading while strafing
                .afterTime(0, intake.on())
                .afterTime(0.2, extendo.forwardExtendo())
                .stopAndAdd(pivot.down())
                .waitSeconds(0.3)
                .lineToX(-13)
                .afterTime(0.4, pivot.up())
                .afterTime(0.4, extendo.backExtendo())
                .strafeTo(new Vector2d(-5, -46))
                .afterTime(0.4, claw.closeClaw())
                .turn(-Math.PI/4.8)
                .build();

        Action DepositSampleOne = drive.actionBuilder(new Pose2d(-5, -47, -Math.PI/4.8))

/*===========================================================================================================
                Deposits Sample One
===========================================================================================================*/

                .stopAndAdd(lift.liftPosition(880))
                .afterTime(0, arm.backArm())
                .waitSeconds(1.1)
                .stopAndAdd(claw.openClaw())
                .build();

        Action SampleTwo = drive.actionBuilder(new Pose2d(-5, -47, 3*Math.PI/4))

/*===========================================================================================================
                Inputs Sample Two
===========================================================================================================*/

                .afterTime(0.3, arm.frontArm())
                .afterTime(0.3, lift.liftPosition(0))
                .strafeToLinearHeading(new Vector2d(-5, -51), (1.03*Math.PI))
                .afterTime(0.2, extendo.forwardExtendo())
                .stopAndAdd(pivot.down())
                .waitSeconds(0.3)
                .lineToX(-12)
                .afterTime(0.4, pivot.up())
                .afterTime(0.4, extendo.backExtendo())
                .strafeTo(new Vector2d(-5, -46))
                .afterTime(0.4, claw.closeClaw())
                .turn(-Math.PI/4)
                .build();


        Action DepositSampleTwo = drive.actionBuilder(new Pose2d(-5, -47, 3*Math.PI/4.8))

/*===========================================================================================================
                Deposits Sample Two
===========================================================================================================*/

                .stopAndAdd(lift.liftPosition(880))
                .afterTime(0, arm.backArm())
                .waitSeconds(1.1)
                .stopAndAdd(claw.openClaw())
                .build();

        Action SampleThree = drive.actionBuilder(new Pose2d(-5, -47, 3*Math.PI/4))

/*===========================================================================================================
                Inputs Sample Three
===========================================================================================================*/

                .afterTime(0.3, arm.frontArm())
                .afterTime(0.3, lift.liftPosition(0))
                .strafeToLinearHeading(new Vector2d(-21, -30), (1.429*Math.PI))
                .afterTime(0.2, extendo.forwardExtendo())
                .stopAndAdd(pivot.down())
                .waitSeconds(0.3)
                .lineToY(-39)
                .afterTime(1, pivot.up())
                .afterTime(1, extendo.backExtendo())
                .strafeTo(new Vector2d(-4, -46))
                .afterTime(0.4, claw.closeClaw())
                .turn(-6*Math.PI/10)
                .build();


        Action DepositSampleThree = drive.actionBuilder(new Pose2d(-5, -47, 3*Math.PI/4.8))

/*===========================================================================================================
                Deposits Sample Three
===========================================================================================================*/

                .stopAndAdd(lift.liftPosition(880))
                .afterTime(0, arm.backArm())
                .waitSeconds(1.1)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)
                .stopAndAdd(arm.frontArm())
                .afterTime(0.3, lift.liftPosition(0))
                .build();


        Action TempGetInPosition = drive.actionBuilder(new Pose2d(-5, -47, 3*Math.PI/4.8))

/*===========================================================================================================
                Temporary action to move closer to the samples
===========================================================================================================*/
                .afterTime(0, intake.off())
                .strafeTo(new Vector2d(-48,-36))
                .turnTo(Math.PI/2)


                .build();


        Action Park = drive.actionBuilder(new Pose2d(-5, -47, 3*Math.PI/4.8))

/*===========================================================================================================
                Parks (duh)
===========================================================================================================*/
                .strafeTo(new Vector2d(-52,-38))
                .turnTo(-1.1*Math.PI/2)
                .lineToY(-24)
                .afterTime(0, lift.liftPosition(100))
                .afterTime(0.5, arm.backArm())
                .afterTime(0.2, lift.liftPosition(0))
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        Hook,
                        SampleOne,
                        DepositSampleOne,
                        SampleTwo,
                        DepositSampleTwo,
                        SampleThree,
                        DepositSampleThree,
                        TempGetInPosition
                )
        );
    }
}
