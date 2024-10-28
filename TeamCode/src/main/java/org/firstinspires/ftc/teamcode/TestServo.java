package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestServo", group="Robot")
//@Disabled
public class TestServo extends LinearOpMode {

        public Servo servo = null;

        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void runOpMode() throws InterruptedException {
            servo = hardwareMap.get(Servo.class, "backclaw");

            waitForStart();

            double position = 0;

            boolean lastDPadDown = false;
            boolean lastDPadUp = false;

            while (opModeIsActive()) {
                if (gamepad1.dpad_down && !lastDPadDown && position > 0) {
                    position = position - .05;
                }

                if (gamepad1.dpad_up && !lastDPadUp && position < 2) {
                    position = position + .05;
                }

                lastDPadDown = gamepad1.dpad_down;
                lastDPadUp = gamepad1.dpad_up;

                servo.setPosition(position);

                telemetry.addData("position: ", position);
                telemetry.update();
            }
        }
    }
