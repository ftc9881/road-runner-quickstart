package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Test Lift", group="Robot")
//@Disabled
public class TestLift extends LinearOpMode {
    public DcMotor leftLift = null;
    public DcMotor rightLift = null;

    public PIDFController leftLiftPID = null;
    public PIDFController rightLiftPID = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Define and Initialize Motors

        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        // Lift

        int liftPosition = 1;

        boolean lastDPadDown = false;
        boolean lastDPadUp = false;

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);

        leftLiftPID = new PIDFController(
                new PIDFController.PIDCoefficients(.005, 0, 0)
        );

        rightLiftPID = new PIDFController(
                new PIDFController.PIDCoefficients(.005, 0, 0)
        );

//        leftLiftPID.setInputBounds(-100, 100);
//        rightLiftPID.setInputBounds(-100, 100);

        leftLiftPID.setOutputBounds(-.5, .75);
        rightLiftPID.setOutputBounds(-.5, .75);

        waitForStart();

        leftLiftPID.reset();
        rightLiftPID.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // lift

            if (gamepad1.dpad_down && !lastDPadDown && liftPosition > 0) {
                liftPosition = liftPosition - 1;
            }

            if (gamepad1.dpad_up && !lastDPadUp && liftPosition < 2) {
                liftPosition = liftPosition + 1;
            }

            lastDPadDown = gamepad1.dpad_down;
            lastDPadUp = gamepad1.dpad_up;

            int targetLiftPosition = liftPosition * (920 / 2);

            leftLiftPID.targetPosition = targetLiftPosition;
            rightLiftPID.targetPosition = targetLiftPosition;

            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();

            double leftLiftPower = leftLiftPID.update(leftLiftPos);
            double rightLiftPower = leftLiftPID.update(rightLiftPos);

//                double leftLiftPower = Math.max(-1, Math.min(1, (targetLiftPosition - leftLiftPos) / 200.0));
//                double rightLiftPower = Math.max(-1, Math.min(1, (targetLiftPosition - rightLiftPos) / 200.0));

            leftLift.setPower(leftLiftPower);
            rightLift.setPower(rightLiftPower);

            telemetry.addData("liftPosition: ", liftPosition);
            telemetry.addData("targetLiftPosition: ", targetLiftPosition);
            telemetry.addData("leftLiftPos: ", leftLiftPos);
            telemetry.addData("rightLiftPos: ", rightLiftPos);
            telemetry.addData("leftLiftPower: ", leftLiftPower);
            telemetry.addData("rightLiftPower: ", rightLiftPower);

            telemetry.update();
        }
    }
}