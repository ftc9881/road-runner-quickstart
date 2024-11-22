package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public abstract class SimpleAuto extends LinearOpMode {


    public class Lift {
        public DcMotor leftLift = null;
        public DcMotor rightLift = null;
        // intake
        public CRServo intake = null;
        public PIDFController leftLiftPID = null;

        public PIDFController rightLiftPID = null;

        public Lift(HardwareMap hardwareMap) {
            leftLift = hardwareMap.get(DcMotor.class, "leftLift");
            rightLift = hardwareMap.get(DcMotor.class, "rightLift");


            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftLift.setDirection(DcMotor.Direction.REVERSE);
            rightLift.setDirection(DcMotor.Direction.FORWARD);

            leftLift.setTargetPosition(0);
            rightLift.setTargetPosition(0);

            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftLift.setPower(1);
            rightLift.setPower(1);
        }

        public class LiftPosition implements Action {
            private boolean initialized = false;

            private int targetPosition = 0;

            public LiftPosition(int targetPosition) {
                this.targetPosition = targetPosition;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftLift.setTargetPosition(targetPosition);
                rightLift.setTargetPosition(targetPosition);

                int rightLiftPos = rightLift.getCurrentPosition();
                int leftLiftPos = leftLift.getCurrentPosition();

                boolean isWithinRange = Math.min(
                        Math.abs(rightLiftPos - targetPosition),
                        Math.abs(leftLiftPos - targetPosition)
                ) < 5;

                return !isWithinRange;
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
    public class BackClaw {
        private Servo backClaw;

        public BackClaw(HardwareMap hardwareMap) {
            backClaw = hardwareMap.get(Servo.class, "backclaw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                backClaw.setPosition(.9);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                backClaw.setPosition(.56);
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
                arm.setPosition(.29);
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

    public class Extendo {
        private Servo extendoRight;
        private Servo extendoLeft;

        public Extendo(HardwareMap hardwareMap) {
            extendoRight = hardwareMap.get(Servo.class, "extendoRight");
            extendoLeft = hardwareMap.get(Servo.class, "extendoLeft ");
        }


        public class BackExtendo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                extendoRight.setPosition(1);
                extendoLeft.setPosition(0);

                return false;
            }
        }
        public Action backExtendo() {
            return new BackExtendo();
        }

        public class ForwardExtendo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                extendoRight.setPosition(0.645);
                extendoLeft.setPosition(0.69);

                return false;
            }
        }
        public Action forwardExtendo() {
            return new ForwardExtendo();
        }

    }
    public class Pivot{
        private Servo pivot;


        public Pivot(HardwareMap hardwareMap) {
            pivot = hardwareMap.get(Servo.class, "pivot");
        }

        public class Down implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pivot.setPosition(.15);
                return false;
            }
        }
        public Action down() {
            return new Down();
        }

        public class Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pivot.setPosition(.94);
                return false;
            }
        }
        public Action up() {
            return new Up();
        }
    }
    public class Intake{
        private CRServo intake;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(CRServo.class, "intake");
        }
        public class On implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(-.6);
                return false;
            }
        }
        public Action on() {return new On();}
        public class Off implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0);
                return false;
            }
        }
        public Action off() {return new Off();}

    }

}
