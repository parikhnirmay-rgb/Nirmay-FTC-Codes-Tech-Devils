package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Mecanum_Drive", group = "Linear OpMode")
public class Mecanum_DrivePinpoint   extends LinearOpMode {

    // Declare Drivetrain Motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    // Declare Pinpoint Computer
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        // Initialize Drivetrain Motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");   // Port 0
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");  // Port 1
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");    // Port 2
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");   // Port 3

        // Reverse the left side motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Pinpoint tracking computer using "odo" configuration name
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        /*
         * PINPOINT HARDWARE TUNING PROPERTIES
         */
        // 1. Set offsets in millimeters (Distance from center of rotation to each pod)
        // Adjust these numbers based on your exact physical placement!
        odo.setOffsets(-84.0, -12.0);

        // 2. Set the resolution matching your physical pods
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // 3. Set directions based on robot movements (FORWARD or REVERSE)
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Calibration sequence - keep the robot completely frozen during this stage
        telemetry.addData("Status", "Calibrating Pinpoint IMU...");
        telemetry.update();

        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized & Calibrated!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /* ==========================================================
             * SECTION 1: PINPOINT ODOMETRY CALCULATIONS
             * ========================================================== */
            // CRITICAL: Pull raw values over I2C first thing in the loop
            odo.update();
            Pose2D pos = odo.getPosition();

            /* ==========================================================
             * SECTION 2: MECANUM DRIVETRAIN CONTROLS
             * ========================================================== */
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

            // Normalize proportional ratios below 1.0 limit
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

            /* ==========================================================
             * SECTION 3: LIVE TELEMETRY LOGGING
             * ========================================================== */
            // Display Position Coordinates (Inches & Degrees)
            telemetry.addLine("---- ROBOT POSITION COORDINATES ----");
            telemetry.addData("X Position (in)", "%.2f", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y Position (in)", "%.2f", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading Angle (deg)", "%.1f", pos.getHeading(AngleUnit.DEGREES));

            // Display Drivetrain Powers
            telemetry.addLine("---- DRIVETRAIN POWER SIGNALS ----");
            telemetry.addData("LF Power", "%.2f", leftFrontPower);
            telemetry.addData("LR Power", "%.2f", leftRearPower);
            telemetry.addData("RF Power", "%.2f", rightFrontPower);
            telemetry.addData("RR Power", "%.2f", rightRearPower);

            telemetry.update();
        }
    }
}
