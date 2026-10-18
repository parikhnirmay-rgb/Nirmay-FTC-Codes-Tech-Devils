package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mecanum_Drive", group = "Linear OpMode")
public class test extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Reverse left motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Original joystick controls commented out
            // double y = -gamepad1.left_stick_y;
            // double x = gamepad1.left_stick_x;
            // double rx = gamepad1.right_stick_x;

            // Initialize motor powers to zero
            double leftFrontPower = 0;
            double leftRearPower = 0;
            double rightFrontPower = 0;
            double rightRearPower = 0;

            // Y button moves robot forward at 20% power
            if (gamepad1.y) {
                leftFrontPower = 0.2;
                leftRearPower = 0.2;
                rightFrontPower = 0.2;
                rightRearPower = 0.2;
            }

            // Send power to motors
            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

            telemetry.addData("Y Button", gamepad1.y);
            telemetry.addData("LF Power", "%.2f", leftFrontPower);
            telemetry.addData("LR Power", "%.2f", leftRearPower);
            telemetry.addData("RF Power", "%.2f", rightFrontPower);
            telemetry.addData("RR Power", "%.2f", rightRearPower);
            telemetry.update();
        }

        // Stop motors
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}