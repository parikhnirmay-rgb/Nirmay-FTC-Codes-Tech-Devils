package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mecanum_Drive", group = "Linear OpMode")
public class Mecanum_Drive extends LinearOpMode {

    // Declare the four motors exactly as requested
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    @Override
    public void runOpMode() {
        // Initialize motors. Ensure these names match your driver station configuration.
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");   // Port 0
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");  // Port 1
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");    // Port 2
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");   // Port 3

        // Reverse the left side motors so positive power moves the robot forward.
        // Depending on your specific gearbox/build, you might need to swap this to the right side instead.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read inputs from Gamepad 1
            double y  = -gamepad1.left_stick_y;  // Up / Down
            double x  = gamepad1.left_stick_x;   // Left / Right (Strafe)
            double rx = gamepad1.right_stick_x;  // Spin (Rotation)

            // Standard Mecanum Kinematics Math
            double leftFrontPower  = y + x + rx;
            double leftRearPower   = y - x + rx;
            double rightFrontPower = y - x - rx;
            double rightRearPower  = y + x - rx;

            // Find the maximum absolute power value calculated
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
            max = Math.max(max, Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(rightRearPower));

            // Normalize the values so no motor exceeds 1.0 or -1.0 power.
            // This maintains the proportional driving ratios.
            if (max > 1.0) {
                leftFrontPower  /= max;
                leftRearPower   /= max;
                rightFrontPower /= max;
                rightRearPower  /= max;
            }

            // Send calculated power to wheels
            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

            // Optional: Display power data on the Driver Station
            telemetry.addData("LF Power", "%.2f", leftFrontPower);
            telemetry.addData("LR Power", "%.2f", leftRearPower);
            telemetry.addData("RF Power", "%.2f", rightFrontPower);
            telemetry.addData("RR Power", "%.2f", rightRearPower);
            telemetry.update();
        }
    }
}