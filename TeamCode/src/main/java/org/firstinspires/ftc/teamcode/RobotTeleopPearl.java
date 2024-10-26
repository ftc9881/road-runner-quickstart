package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.jvm.Code;

import androidx.annotation.Nullable;

@TeleOp(name="Robot: Teleop Pearl", group="Robot")
//@Disabled
public class RobotTeleopPearl extends LinearOpMode {
    /* Copyright (c) 2017 FIRST. All rights reserved.
     *
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     *
     * Redistributions of source code must retain the above copyright notice, this list
     * of conditions and the following disclaimer.
     *
     * Redistributions in binary form must reproduce the above copyright notice, this
     * list of conditions and the following disclaimer in the documentation and/or
     * other materials provided with the distribution.
     *
     * Neither the name of FIRST nor the names of its contributors may be used to endorse or
     * promote products derived from this software without specific prior written permission.
     *
     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
     * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
     * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
     */



        /* Declare OpMode members. */
        public DcMotor leftFront = null;
        public DcMotor rightFront = null;
        public DcMotor rightBack = null;
        public DcMotor leftBack = null;
        public DcMotor leftLift = null;
        public DcMotor rightLift = null;


        public Servo extendoLeft = null;
        public Servo extendoRight = null;
        public Servo claw = null;
        public Servo backclaw = null;
        public Servo arm = null;
        public Servo pivot = null;

        public CRServo intake = null;

        public double CLAW_OPEN_POSITION = .56;
        public double CLAW_CLOSED_POSITION = .33;

        public double BACK_CLAW_OPEN_POSITION = .75;
        public double BACK_CLAW_CLOSED_POSITION = .36;


        public double ARM_FRONT_POSITION = .86;
        public double ARM_MID_POSITION = .33;
        public double ARM_BACK_POSITION = .25;

        public double EXTENDO_IN_RIGHT_POSITION = 1;
        public double EXTENDO_IN_LEFT_POSITION = 0;

        public double EXTENDO_OUT_RIGHT_POSITION = 0.62;
        public double EXTENDO_OUT_LEFT_POSITION = 0.70;

        public double PIVOT_DOWN_POSITION = .1;
        public double PIVOT_UP_POSITION = .9;

        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void runOpMode() throws InterruptedException {
            // Define and Initialize Motors
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            leftLift = hardwareMap.get(DcMotor.class, "leftLift");
            rightLift = hardwareMap.get(DcMotor.class, "rightLift");

            extendoRight = hardwareMap.get(Servo.class, "extendoRight");
            extendoLeft = hardwareMap.get(Servo.class, "extendoLeft ");
            claw = hardwareMap.get(Servo.class, "claw");
            backclaw = hardwareMap.get(Servo.class, "backclaw");
            arm = hardwareMap.get(Servo.class, "arm");
            pivot = hardwareMap.get(Servo.class, "pivot");
            intake = hardwareMap.get(CRServo.class, "intake");

            // Extendo

            boolean lastButtonA = false;

            boolean extendoOut = false;

            double extendoLeftPosition = EXTENDO_IN_LEFT_POSITION;
            double extendoRightPosition = EXTENDO_IN_RIGHT_POSITION;

            extendoRight.setPosition(extendoRightPosition);
            extendoLeft.setPosition(extendoLeftPosition);

            // Pivot

            boolean lastButtonB = false;

            boolean pivotDown = false;

            double pivotPosition = PIVOT_UP_POSITION;

            pivot.setPosition(pivotPosition);

            // Claw

            boolean clawOpen = false;
            double clawPosition = CLAW_CLOSED_POSITION; // for some reason closed is open.
            boolean lastRightBumper = false;

            claw.setPosition(clawPosition);

            // Back Claw

            boolean backClawOpen = false;
            double backClawPosition = BACK_CLAW_CLOSED_POSITION;
            boolean lastY = false;

            backclaw.setPosition(backClawPosition);

            // Arm

            boolean armForward = true;
            double armPosition = ARM_FRONT_POSITION;
            boolean lastButtonX = false;

            arm.setPosition(armPosition);

            // Lift

            int liftPosition = 0;

            boolean lastDPadDown = false;
            boolean lastDPadUp = false;

            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);

            leftLift.setDirection(DcMotor.Direction.REVERSE);
            rightLift.setDirection(DcMotor.Direction.FORWARD);

            // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.05; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;

                int rightLiftPos = rightLift.getCurrentPosition();
                int leftLiftPos = leftLift.getCurrentPosition();

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                if (gamepad1.left_bumper) {
                    frontLeftPower *= .25;
                    backLeftPower *= .25;
                    frontRightPower *= .25;
                    backRightPower *= .25;
                }

                leftFront.setPower(frontLeftPower);
                leftBack.setPower(backLeftPower);
                rightFront.setPower(frontRightPower);
                rightBack.setPower(backRightPower);

                // lift

                if(gamepad1.dpad_down && !lastDPadDown && liftPosition > 0) {
                    liftPosition = liftPosition - 1;
                }

                if(gamepad1.dpad_up && !lastDPadUp && liftPosition < 2) {
                    liftPosition = liftPosition + 1;
                }

                lastDPadDown = gamepad1.dpad_down;
                lastDPadUp = gamepad1.dpad_up;

                int targetLiftPosition = liftPosition * (920 / 2);

                double leftLiftPower = Math.max(-1, Math.min(1, (targetLiftPosition - leftLiftPos) / 200.0));
                double rightLiftPower = Math.max(-1, Math.min(1, (targetLiftPosition - rightLiftPos) / 200.0));

                leftLift.setPower(leftLiftPower);
                rightLift.setPower(rightLiftPower);

                telemetry.addData("liftPosition: ", liftPosition);
                telemetry.addData("targetLiftPosition: ", targetLiftPosition);
                telemetry.addData("leftLiftPos: ", leftLiftPos);
                telemetry.addData("rightLiftPos: ", rightLiftPos);
                telemetry.addData("leftLiftPower: ", leftLiftPower);
                telemetry.addData("rightLiftPower: ", rightLiftPower);

                // Servos

                // Extendo spasm

                if (gamepad1.b) {
                    extendoRight.setPosition(EXTENDO_OUT_RIGHT_POSITION);
                    extendoLeft.setPosition(EXTENDO_OUT_LEFT_POSITION);
                }

                // Claw

                if (gamepad1.right_bumper && !lastRightBumper) {
                    clawOpen = !clawOpen;
                }

                lastRightBumper = gamepad1.right_bumper;

                if(clawOpen) {
                    clawPosition = CLAW_OPEN_POSITION;
                } else {
                    clawPosition = CLAW_CLOSED_POSITION;
                }

                claw.setPosition(clawPosition);
                telemetry.addData("clawPosition: ", clawPosition);

                // Back Claw

                if (gamepad1.y && !lastY) {
                    backClawOpen = !backClawOpen;
                }

                lastY = gamepad1.y;

                if(backClawOpen) {
                    backClawPosition = BACK_CLAW_OPEN_POSITION;
                } else {
                    backClawPosition = BACK_CLAW_CLOSED_POSITION;
                }

                backclaw.setPosition(backClawPosition);
                telemetry.addData("backClawPosition: ", backClawPosition);

                // Arm

                if (gamepad1.x && !lastButtonX) {
                    armForward = !armForward;
                }

                lastButtonX = gamepad1.x;

                if(armForward) {
                    armPosition = ARM_FRONT_POSITION;
                } else {
                    armPosition = ARM_MID_POSITION;
                }

                arm.setPosition(armPosition);
                telemetry.addData("armPosition: ", armPosition);

                // Extendo

                if (gamepad1.a && !lastButtonA) {
                    extendoOut = !extendoOut;
                }

                lastButtonA = gamepad1.a;

                if(extendoOut) {
                    extendoLeftPosition = EXTENDO_OUT_LEFT_POSITION;
                    extendoRightPosition = EXTENDO_OUT_RIGHT_POSITION;
                } else {
                    extendoLeftPosition = EXTENDO_IN_LEFT_POSITION;
                    extendoRightPosition = EXTENDO_IN_RIGHT_POSITION;

                    pivotDown = false;
                }

                extendoLeft.setPosition(extendoLeftPosition);
                extendoRight.setPosition(extendoRightPosition);

                // Pivot

                if (gamepad1.b && !lastButtonB) {
                    pivotDown = !pivotDown;
                }

                if(!extendoOut) {
                    pivotDown = false;
                }

                lastButtonB = gamepad1.b;

                if(pivotDown) {
                    pivotPosition = PIVOT_DOWN_POSITION;
                } else {
                    pivotPosition = PIVOT_UP_POSITION;
                }

                pivot.setPosition(pivotPosition);
                telemetry.addData("pivotPosition: ", pivotPosition);

                // Intake

                intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

                telemetry.update();
            }
        }
    }
