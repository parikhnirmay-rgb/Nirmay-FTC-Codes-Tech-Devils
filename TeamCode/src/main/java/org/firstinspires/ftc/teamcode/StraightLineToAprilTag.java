
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "GoToAprilTag", group = "Autonomous")
public class StraightLineToAprilTag extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;


    // Sensors
    private GoBildaPinpointDriver pinpoint;
    private HuskyLens huskyLens;

    // Fine-tuned motor powers
    private static final double LF_POWER = 0.16;
    private static final double LR_POWER = 0.25;
    private static final double RF_POWER = 0.26;
    private static final double RR_POWER = 0.27;

    // AprilTag stopping distance
    private static final double STOP_DISTANCE_MM = 200.0;

    // HuskyLens calibration
    private static final double CALIBRATION_WIDTH_PIXELS = 76.0;
    private static final double CALIBRATION_DISTANCE_MM = 300.0;

    // Set this to the AprilTag ID you want to follow
    private static final int TARGET_TAG_ID = 1;

    // Movement settings
    private static final double TOLERANCE_MM = 15.0;
    private static final double TIMEOUT_SECONDS = 20.0;

    // Used for slow-down near target
    private static final double SLOWDOWN_DISTANCE_MM = 200.0;
    private static final double MIN_SPEED_FACTOR = 0.5;
    private static int CLOSE_DETECTIONS = 2;

    int closereadings = 0;

    @Override
    public void runOpMode() {

        // ==================================
        // INITIALIZE MOTORS
        // ==================================

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ==================================
        // INITIALIZE PINPOINT
        // ==================================

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        // Your measured offsets
        pinpoint.setOffsets(90, 0, DistanceUnit.MM);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();

        // ==================================
        // INITIALIZE HUSKYLENS
        // ==================================

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        if (!huskyLens.knock()) {
            telemetry.addLine("ERROR: HuskyLens not responding!");
            telemetry.update();
            return;
        }

        huskyLens.selectAlgorithm(
                HuskyLens.Algorithm.TAG_RECOGNITION
        );

        telemetry.addLine("Initialized");
        telemetry.addLine("Place target AprilTag directly ahead");
        telemetry.addData("Target Tag ID", TARGET_TAG_ID);
        telemetry.addLine("Keep robot still during IMU calibration");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        String stopReason = "Not started";

        try {

            // ==================================
            // STEP 1: FIND THE APRILTAG
            // ==================================

            HuskyLens.Block targetTag = null;

            // Allow a short period to detect the tag
            ElapsedTime detectionTimer = new ElapsedTime();

            while (opModeIsActive()) {

                HuskyLens.Block[] blocks = huskyLens.blocks();

                for (HuskyLens.Block block : blocks) {

                    if (block.width > 0) {
                        targetTag = block;
                        break;
                    }
                }

                if (targetTag != null) {
                    break;
                }

                telemetry.addLine("Searching for AprilTag...");
                telemetry.update();
                telemetry.addData("Number of Tags", blocks.length);

                for (HuskyLens.Block block : blocks) {
                    telemetry.addData("Detected Tag ID", block.id);
                    telemetry.addData("Tag Width", block.width);
                }

                telemetry.update();

                sleep(50);
            }

            // Do not drive if tag wasn't found


            // ==================================
            // STEP 2: CALCULATE TARGET ONCE
            // ==================================


            double angleDegrees = Math.toDegrees(
                    Math.atan((targetTag.x - 160.0) / 442.0)
            );
            telemetry.addData("Tag angle:", angleDegrees);
            telemetry.addData(
                    "Tag Width (pixels)", targetTag.width
            );
            telemetry.update();

            // Already within stopping distance

            // ==================================
            // STEP 3: RECORD STARTING POSITION
            // ==================================

            pinpoint.update();

            Pose2D startPosition = pinpoint.getPosition();

            double startX =
                    startPosition.getX(DistanceUnit.MM);

            double startY =
                    startPosition.getY(DistanceUnit.MM);

            ElapsedTime driveTimer = new ElapsedTime();

            stopReason = "Timeout";

            // ==================================
            // STEP 4: DRIVE STRAIGHT
            // ==================================

            while (opModeIsActive()
                    && driveTimer.seconds() < TIMEOUT_SECONDS) {
                pinpoint.update();
                Pose2D position = pinpoint.getPosition();
                double x = position.getX(DistanceUnit.MM);
                double y = position.getY(DistanceUnit.MM);
                // Distance traveled from start
                double dx = x - startX;
                double dy = y - startY;
                double traveledDistance =
                        Math.hypot(dx, dy);
                HuskyLens.Block[] newBlocks = huskyLens.blocks();

                HuskyLens.Block updatedTag = null;

                for (HuskyLens.Block block : newBlocks) {
                    if (block.id == targetTag.id && block.width > 0) {
                        updatedTag = block;
                        break;
                    }
                }

// Don't drive if the tag is temporarily missing
                if (updatedTag == null) {
                    stopMotors();
                    closereadings = 0;
                    telemetry.addLine("AprilTag not detected");
                    telemetry.update();
                    sleep(50);
                    continue;
                }

// Update the tag with the newest detection
                targetTag = updatedTag;

                double estimatedTagDistance =
                        CALIBRATION_DISTANCE_MM
                                * CALIBRATION_WIDTH_PIXELS
                                / targetTag.width;
                // Desired forward travel distance
                double remaining =
                        estimatedTagDistance - STOP_DISTANCE_MM;
                // Stop at target
                if (remaining <= TOLERANCE_MM) {
                    stopMotors();
                    closereadings++;

                    if (closereadings >= CLOSE_DETECTIONS) {
                        stopReason = "Target reached";
                        break;
                    }

                } else {
                    closereadings = 0;

                    // Calculate speed and drive motors here
                }
                // Slow down near destination
                double speedFactor = Math.min(
                        1.0,
                        Math.max(
                                MIN_SPEED_FACTOR,
                                remaining / SLOWDOWN_DISTANCE_MM
                        )
                );
                // Your calibrated straight-driving powers
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
                telemetry.addLine("=== APRILTAG ===");
                telemetry.addData(
                        "Tag center",
                        "X=%d Y=%d",
                        targetTag.x, targetTag.y);
                telemetry.addData(
                        "Tag size",
                        "%d x %d pixels",
                        targetTag.width, targetTag.height);
                telemetry.addData(
                        "Tag ID", TARGET_TAG_ID
                );
                telemetry.addData(
                        "Initial Tag Distance (mm)",
                        "%.1f", estimatedTagDistance
                );
                telemetry.addLine("=== ODOMETRY ===");
                telemetry.addData(
                        "Traveled (mm)",
                        "%.1f", traveledDistance
                );
                telemetry.addData(
                        "Remaining (mm)",
                        "%.1f", remaining
                );
                telemetry.addData("X (mm)", "%.1f", x);
                telemetry.addData("Y (mm)", "%.1f", y);
                telemetry.addData(
                        "Heading (deg)",
                        "%.2f",
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
            // Always stop motors
            stopMotors();
            telemetry.addData("Result", stopReason);
            telemetry.addLine("Motors stopped");
            telemetry.update();
        }
    }
    private void stopMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}
