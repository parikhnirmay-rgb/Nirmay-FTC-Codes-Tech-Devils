
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Drive 2 Meters", group = "Autonomous")
public class DistanceAutonomous extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    // Pinpoint odometry
    private GoBildaPinpointDriver pinpoint;

    // Target: 2 meters = 2000 mm
    private static final double TARGET_DISTANCE = 2000.0;

    // Your fine-tuned motor powers
    private static final double LF_POWER = 0.16;
    private static final double LR_POWER = 0.15;
    private static final double RF_POWER = 0.26;
    private static final double RR_POWER = 0.27;

    // Distance tolerance (15 mm)
    private static final double TOLERANCE = 15.0;

    // Maximum autonomous runtime
    private static final double TIMEOUT_SECONDS = 20.0;

    @Override
    public void runOpMode() {

        // ==================================
        // INITIALIZE MOTORS
        // ==================================

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        // Brake when stopped
        leftFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        // ==================================
        // INITIALIZE PINPOINT
        // ==================================

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        // Your measured offsets
        pinpoint.setOffsets(90, 0, DistanceUnit.MM);

        // Odometry pod type
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // Encoder directions
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset position and IMU
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Autonomous Initialized");
        telemetry.addLine("Target: 200 cm");
        telemetry.addLine("Using fine-tuned motor powers");
        telemetry.addLine("Distance calculated using X and Y");
        telemetry.addLine("Keep robot still during calibration");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // ==================================
        // RECORD STARTING POSITION
        // ==================================

        pinpoint.update();

        Pose2D startPosition = pinpoint.getPosition();

        double startX = startPosition.getX(DistanceUnit.MM);
        double startY = startPosition.getY(DistanceUnit.MM);

        ElapsedTime timer = new ElapsedTime();

        String stopReason = "Timeout";

        try {

            // ==================================
            // MAIN AUTONOMOUS LOOP
            // ==================================

            while (opModeIsActive()
                    && timer.seconds() < TIMEOUT_SECONDS) {

                // Update Pinpoint readings
                pinpoint.update();

                Pose2D position = pinpoint.getPosition();

                double x = position.getX(DistanceUnit.MM);
                double y = position.getY(DistanceUnit.MM);

                // Calculate change from starting position
                double dx = x - startX;
                double dy = y - startY;

                // ==================================
                // NEW DISTANCE CALCULATION
                // ==================================

                // Straight-line displacement from start
                double distance = Math.hypot(dx, dy);

                // Remaining distance to target
                double remaining = TARGET_DISTANCE - distance;

                // Stop when target reached
                if (remaining <= TOLERANCE) {
                    stopReason = "Target reached";
                    break;
                }

                // ==================================
                // SLOW DOWN NEAR TARGET
                // ==================================

                double speedFactor = Math.min(
                        1.0,
                        Math.max(0.5, remaining / 400.0)
                );

                // ==================================
                // APPLY FINE-TUNED MOTOR POWERS
                // ==================================

                double lfPower = LF_POWER * speedFactor;
                double lrPower = LR_POWER * speedFactor;
                double rfPower = RF_POWER * speedFactor;
                double rrPower = RR_POWER * speedFactor;

                leftFront.setPower(lfPower);
                leftRear.setPower(lrPower);
                rightFront.setPower(rfPower);
                rightRear.setPower(rrPower);

                // ==================================
                // TELEMETRY
                // ==================================

                telemetry.addLine("=== AUTONOMOUS ===");

                telemetry.addData(
                        "Target (mm)", TARGET_DISTANCE
                );

                telemetry.addData(
                        "Distance (mm)", "%.2f", distance
                );

                telemetry.addData(
                        "Remaining (mm)", "%.2f", remaining
                );

                telemetry.addData(
                        "Elapsed Time (s)", "%.2f", timer.seconds()
                );

                telemetry.addLine("=== PINPOINT POSITION ===");

                telemetry.addData(
                        "X (mm)", "%.2f", x
                );

                telemetry.addData(
                        "Y (mm)", "%.2f", y
                );

                telemetry.addData(
                        "X Change (mm)", "%.2f", dx
                );

                telemetry.addData(
                        "Y Change (mm)", "%.2f", dy
                );

                telemetry.addData(
                        "Heading (deg)", "%.2f",
                        position.getHeading(AngleUnit.DEGREES)
                );

                telemetry.addData(
                        "Pinpoint Status",
                        pinpoint.getDeviceStatus()
                );

                telemetry.addLine("=== MOTOR POWERS ===");

                telemetry.addData("LF", "%.3f", lfPower);
                telemetry.addData("LR", "%.3f", lrPower);
                telemetry.addData("RF", "%.3f", rfPower);
                telemetry.addData("RR", "%.3f", rrPower);

                telemetry.update();

                sleep(20);
            }

        } finally {

            // Always stop all motors
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }

        telemetry.addData("Result", stopReason);
        telemetry.addLine("Motors stopped");
        telemetry.update();
    }
}
