
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mecanum_Drive", group = "Linear OpMode")
public class testwithoffsetscalibration extends LinearOpMode {

    // Declare four motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    @Override
    public void runOpMode() {

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); //0
        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); //1
        leftRear = hardwareMap.get(DcMotor.class, "leftRear"); //2
        rightRear = hardwareMap.get(DcMotor.class, "rightRear"); //3

        // Reverse left-side motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Initialize motor powers to zero
            double leftFrontPower = 0.0;
            double leftRearPower = 0.0;
            double rightFrontPower = 0.0;
            double rightRearPower = 0.0;

            // Hold Y to move forward at 20% power
            if (gamepad1.y) {
                leftFrontPower = 0.16;
                leftRearPower = 0.15;
                rightFrontPower = 0.26;
                rightRearPower = 0.27;
            }

            // Send power to motors
            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

            // Display motor powers
            telemetry.addData("Y Button", gamepad1.y);
            telemetry.addData("LF Power", "%.2f", leftFrontPower);
            telemetry.addData("LR Power", "%.2f", leftRearPower);
            telemetry.addData("RF Power", "%.2f", rightFrontPower);
            telemetry.addData("RR Power", "%.2f", rightRearPower);
            telemetry.update();
        }

        // Stop all motors
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}
