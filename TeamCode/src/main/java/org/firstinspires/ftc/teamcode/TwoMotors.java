package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Two Motor Drive", group="Linear OpMode")
public class TwoMotors extends LinearOpMode { // Note: Kept class name OneMotor so you don't have to rename your file!

    // 1. Declare variables for BOTH motors
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {
        // 2. Hardware Mapping (these names must match your configuration on the phone)
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Reverse the right motor so pushing both joysticks forward moves the robot forward
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Two Motors Initialized!");
        telemetry.update();

        waitForStart();

        // 3. Main Loop
        while (opModeIsActive()) {
            // Tank Drive Layout: Left stick controls left motor, right stick controls right motor
            double leftPower  = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            // Send power to the motors
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Display live telemetry on your Android phone screen
            telemetry.addData("Left Motor Power", leftPower);
            telemetry.addData("Right Motor Power", rightPower);
            telemetry.update();
        }
    }
}
