package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public abstract class SimpleAuto extends LinearOpMode {
    public class Lift {
        public DcMotor leftLift = null;
        public DcMotor rightLift = null;

        double window = 200;
        double maxPower = 1;
        double minPower = -1;

        public Lift(HardwareMap hardwareMap, double window, double minPower, double maxPower) {
            this.window = window;
            this.minPower = minPower;
            this.maxPower = maxPower;

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
            private double stopPower = .2;

            private int counter = 0;

            public LiftPosition(int targetPosition, double stopPower) {
                this.targetPosition = targetPosition;
                this.stopPower = stopPower;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int rightLiftPos = rightLift.getCurrentPosition();
                int leftLiftPos = leftLift.getCurrentPosition();

                packet.put("rightLiftPos", rightLiftPos);
                packet.put("leftLiftPos", leftLiftPos);

                double leftLiftPower = Math.max(minPower, Math.min(maxPower, (targetPosition - leftLiftPos) / window));
                double rightLiftPower = Math.max(minPower, Math.min(maxPower, (targetPosition - rightLiftPos) / window));

                packet.put("leftLiftPower", leftLiftPower);
                packet.put("rightLiftPower", rightLiftPower);

                leftLift.setPower(leftLiftPower);
                rightLift.setPower(rightLiftPower);

                boolean isDone = Math.min(
                        Math.abs(rightLiftPos - targetPosition),
                        Math.abs(leftLiftPos - targetPosition)
                ) < 50;

                boolean isVeryDone = true;

                if(isDone) {
                    ++counter;

                    if(counter > 200) {
                        leftLift.setPower(stopPower);
                        rightLift.setPower(stopPower);
                        isVeryDone = true;
                    } else {
                        isVeryDone = false;
                    }
                } else {
                    counter = 0;
                }

                return isVeryDone;
            }
        }

        public Action liftPosition(int targetPosition, double stopPower) {
            return new LiftPosition(targetPosition, stopPower);
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
                arm.setPosition(.25);
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
                extendoRight.setPosition(0.62);
                extendoLeft.setPosition(0.70);

                return false;
            }
        }
        public Action forwardExtendo() {
            return new ForwardExtendo();
        }

    }

}
