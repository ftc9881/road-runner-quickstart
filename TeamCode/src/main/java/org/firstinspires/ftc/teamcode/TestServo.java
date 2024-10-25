package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="TestServo", group="Robot")
//@Disabled
public class TestServo extends LinearOpMode {

        public CRServo intake = null;

        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void runOpMode() throws InterruptedException {
             intake = hardwareMap.get(CRServo.class, "intake");

            waitForStart();

            while (opModeIsActive()) {

                if(gamepad1.x) {
                    intake.setPower(1);
                }
                if(gamepad1.y) {
                    intake.setPower(.5);
                }
                if(gamepad1.a) {
                    intake.setPower(.25);
                }
                if(gamepad1.b) {
                    intake.setPower(.1);
                }

                telemetry.addData("power: ", intake.getPower());
                telemetry.update();
            }
        }
    }
