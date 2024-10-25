package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
        public Servo arm = null;

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
            arm = hardwareMap.get(Servo.class, "arm");

            int liftPosition = 0;
            boolean lastButtonX = false;
            boolean lastButtonY = false;

            double clawPosition = .56;
            double armPosition = .86;

            extendoRight.setPosition(1);
            extendoLeft.setPosition(0);
            claw.setPosition(clawPosition);
            arm.setPosition(armPosition);

            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            extendoLeft = hardwareMap.get(Servo.class, "extendoLeft");
//            extendoRight = hardwareMap.get(Servo.class, "extendoRight");

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


//            double kP=0.00, kI=0.00, kD=0.00;
//            int setPoint=0;
//            int measuredPosition=0;
//            // specify coefficients/gains
//            PIDCoefficients coefficients = new PIDCoefficients();
//            //PIDCoefficients coefficients = new PIDCoefficients(kP, kI, kD);
//            // create the controller
//            PIDFController controller = new PIDFController(coefficients);

            // specify the setPoint
//            controller.setTargetPosition(setPoint);

            // in each iteration of the control loop
            // measure the position or output variable
            // apply the correction to the input variable
//            double correction = controller.update(measuredPosition);

            waitForStart();

            if (isStopRequested()) return;
            boolean yButton2 = false;
            boolean bButton1 = false;
            boolean bButton2 = false;
            boolean aButton2 = false;
            boolean bom = false;
            boolean aminal = false;



            boolean checkExtendo = false;
            int num = 0;

            double extendoLeftTargetPos = 0.16;
            double extendoRightTargetPos = 0.16;


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

//            if(gamepad1.a){
//                lift.setPower(-.04);
//            }
//            if(gamepad1.y){
//                lift.setPower(.04);
//            }
//            if(gamepad1.b) {
//                lift.setPower(0);
//            }
                if (gamepad1.left_bumper) {
                    frontLeftPower *= .25;
                    backLeftPower *= .25;
                    frontRightPower *= .25;
                    backRightPower *= .25;
                }
                if (gamepad1.right_bumper) {
                    frontLeftPower *= .5;
                    backLeftPower *= .5;
                    frontRightPower *= .5;
                    backRightPower *= .5;
                }

                leftFront.setPower(frontLeftPower);
                leftBack.setPower(backLeftPower);
                rightFront.setPower(frontRightPower);
                rightBack.setPower(backRightPower);

                //lift

                if(gamepad1.x && !lastButtonX && liftPosition > 0) {
                    liftPosition = liftPosition - 1;
                }

                if(gamepad1.y && !lastButtonY && liftPosition < 2) {
                    liftPosition = liftPosition + 1;
                }

                int targetLiftPosition = liftPosition * (920 / 2);

                double leftLiftPower = Math.max(-1, Math.min(1, (targetLiftPosition - leftLiftPos) / 200.0));
                double rightLiftPower = Math.max(-1, Math.min(1, (targetLiftPosition - rightLiftPos) / 200.0));

                leftLift.setPower(leftLiftPower);
                rightLift.setPower(rightLiftPower);

//                if(gamepad1.x){
//                    if(leftLiftPos < 920) {
//                        leftLift.setPower(.3 * Math.min(1, (920 - leftLiftPos) / 50));
//                    } else {
//                        leftLift.setPower(0);
//                    }
//
//                    if(rightLiftPos < 920) {
//                        rightLift.setPower(.3 * Math.min(1, (920 - rightLiftPos) / 50));
//                    } else {
//                        rightLift.setPower(0);
//                    }
//                }
//                else if(gamepad1.y){
//                    if(leftLiftPos >= 0) {
//                        leftLift.setPower(-.30 * Math.min(1, leftLiftPos / 50));
//                    } else {
//                        leftLift.setPower(0);
//                    }
//
//                    if(rightLiftPos >= 0) {
//                        rightLift.setPower(-.30 * Math.min(1, rightLiftPos / 50));
//                    } else {
//                        rightLift.setPower(0);
//                    }
//                }
//                else {
//                    leftLift.setPower(0);
//                    rightLift.setPower(0);
//                }

                telemetry.addData("liftPosition: ", liftPosition);
                telemetry.addData("targetLiftPosition: ", targetLiftPosition);
                telemetry.addData("leftLiftPos: ", leftLiftPos);
                telemetry.addData("rightLiftPos: ", rightLiftPos);
                telemetry.addData("leftLiftPower: ", leftLiftPower);
                telemetry.addData("rightLiftPower: ", rightLiftPower);

                // servos

                if(gamepad1.a) {
                    extendoRight.setPosition(1.0);
                    extendoLeft.setPosition(0.0);

                }
                if (gamepad1.b) {
                    extendoRight.setPosition(0.62);
                    extendoLeft.setPosition(0.70);
                }

                if (gamepad1.right_bumper) {
                    clawPosition = .56;
                } else if (gamepad1.left_bumper) {
                    clawPosition = .33;
                }

                claw.setPosition(clawPosition);
                telemetry.addData("clawPosition: ", clawPosition);

                if (gamepad1.dpad_up) {
                    armPosition = .05;
                }
                else if (gamepad1.dpad_down) {
                    armPosition = .86;
                }
                else if (gamepad1.dpad_right) {
                    armPosition = .33;
                }

                arm.setPosition(armPosition);
                telemetry.addData("armPosition: ", armPosition);

                lastButtonX = gamepad1.x;
                lastButtonY = gamepad1.y;
//
//
//                extendoLeft.setPosition(extendoLeftTargetPos);
//                extendoRight.setPosition(extendoRightTargetPos);
 //               telemetry.addData("Extendo", extendoLeftTargetPos);

                telemetry.update();
            }
        }
    }
