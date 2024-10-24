package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name="Simple Auto", group="Roadrunner")
public class SimpleAuto extends LinearOpMode {
    public class Lift {
        public DcMotor leftLift = null;
        public DcMotor rightLift = null;

        public Lift(HardwareMap hardwareMap) {
            leftLift = hardwareMap.get(DcMotor.class, "leftLift");
            rightLift = hardwareMap.get(DcMotor.class, "rightLift");

            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftLift.setDirection(DcMotor.Direction.REVERSE);
            rightLift.setDirection(DcMotor.Direction.FORWARD);
        }

        public class LiftPosition implements Action {
            private boolean initialized = false;

            private int targetPosition = 920;

            public LiftPosition(int targetPosition) {
                this.targetPosition = targetPosition;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int rightLiftPos = rightLift.getCurrentPosition();
                int leftLiftPos = leftLift.getCurrentPosition();

                double leftLiftPower = Math.max(-.3, Math.min(.3, (targetPosition - leftLiftPos) / 200.0));
                double rightLiftPower = Math.max(-.3, Math.min(.3, (targetPosition - rightLiftPos) / 200.0));

                leftLift.setPower(leftLiftPower);
                rightLift.setPower(rightLiftPower);

                packet.put("rightLiftPos", rightLiftPos);
                packet.put("leftLiftPos", leftLiftPos);

                boolean isDone = Math.min(
                        Math.abs(rightLiftPos - targetPosition),
                        Math.abs(leftLiftPos - targetPosition)
                ) < 50;

                return isDone;
            }
        }

        public Action liftPosition(int targetPosition) {
            return new LiftPosition(targetPosition);
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(.56);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(.33);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class Arm {
        private Servo arm;

        public Arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(Servo.class, "arm");
        }

        public class BackArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPosition(.05);
                return false;
            }
        }
        public Action backArm() {
            return new BackArm();
        }

        public class MidArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPosition(.33);
                return false;
            }
        }
        public Action midArm() {
            return new MidArm();
        }

        public class FrontArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPosition(.86);
                return false;
            }
        }
        public Action frontArm() {
            return new FrontArm();
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);

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

//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        .lineToX(48)
//                        .waitSeconds(3)
////                        .strafeTo(new Vector2d(0, 48))
//                        .turn(- Math.PI / 2)
//                        .build());

//                Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        .stopAndAdd(arm.frontArm())
//                        .waitSeconds(1)
//                        .stopAndAdd(claw.openClaw())
//                        .waitSeconds(1)
//                        .stopAndAdd(claw.closeClaw())
//                        .waitSeconds(1)
//                        .stopAndAdd(arm.midArm())
//                        .waitSeconds(1)
//                        .stopAndAdd(claw.openClaw())
//                        .waitSeconds(1)
//                        .stopAndAdd(claw.closeClaw())
//                        .waitSeconds(1)
//                        .stopAndAdd(arm.backArm())
//                        .waitSeconds(1)
//                        .stopAndAdd(claw.openClaw())
//                        .waitSeconds(1)
//                        .stopAndAdd(claw.closeClaw())
//                        .waitSeconds(1)
//                        .stopAndAdd(arm.frontArm())
//                        .build());

                Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(0, 8))
                        .waitSeconds(1)
                        .lineToX(-20)
                        .waitSeconds(1)
                        .turn(Math.PI / 4)
                        .stopAndAdd(arm.backArm())
                        .waitSeconds(1)
                        .stopAndAdd(claw.openClaw())
                        .waitSeconds(1)
                        .stopAndAdd(arm.frontArm())
                        .waitSeconds(1)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());

//        Actions.runBlocking(
//                new SequentialAction(
//                        claw.openClaw()
//                )
//        );
    }
}
