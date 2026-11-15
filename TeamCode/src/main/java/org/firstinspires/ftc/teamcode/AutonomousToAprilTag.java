
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "AprilTag Distance Drive", group = "Autonomous")
public class AutonomousToAprilTag extends LinearOpMode {

    // Motors
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    // Odometry
    private GoBildaPinpointDriver pinpoint;

    // Target distance from HuskyLens
    private double DISTANCE_MM = 0;

    // Distance tolerance
    private static final double TOLERANCE_MM = 20.0;

    // Your calibrated forward powers
    private static final double LF = 0.16;
    private static final double LR = 0.15;
    private static final double RF = 0.26;
    private static final double RR = 0.27;

    // Strafe strength
    private static final double STRAFE_POWER = 0.20;

    // Original heading correction
    private static final double HEADING_KP = 0.002;
    private static final double MAX_TURN = 0.04;

    // Safety timeout
    private static final double TIMEOUT = 10.0;

    // Movement state
    private boolean moving = false;

    // Starting pose
    private double startX = 0;
    private double startY = 0;
    private double startHeading = 0;

    // Movement angle from HuskyLens
    private double angleDegrees = 0;

    // Motor timer
    private final ElapsedTime timer = new ElapsedTime();

    private String result = "Ready";

    private double distance = 0;
    private double remaining = 0;

    // HuskyLens calibration
    private static final int CENTER_X = 160;
    private static final int CENTER_Y = 120;

    private static final double CALIBRATION_WIDTH_PIXELS = 76.0;
    private static final double CALIBRATION_DISTANCE_MM = 300.0;

    @Override
    public void runOpMode() {

        // =====================================
        // INITIALIZE MOTORS
        // =====================================

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

        // =====================================
        // INITIALIZE PINPOINT
        // =====================================

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        pinpoint.setOffsets(90, 0, DistanceUnit.MM);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();

        // =====================================
        // INITIALIZE HUSKYLENS
        // =====================================

        HuskyLens huskyLens =
                hardwareMap.get(HuskyLens.class, "huskylens");

        if (!huskyLens.knock()) {
            telemetry.addLine("ERROR: HUSKYLENS not responding");
            telemetry.update();
            return;
        }

        huskyLens.selectAlgorithm(
                HuskyLens.Algorithm.TAG_RECOGNITION
        );

        telemetry.addLine("Robot Initialized");
        telemetry.addLine("HUSKYLENS connected");
        telemetry.addLine("Ready for autonomous AprilTag movement");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // =====================================
        // FIND APRILTAG
        // =====================================

        HuskyLens.Block target = null;

        while (opModeIsActive() && target == null) {

            HuskyLens.Block[] blocks = huskyLens.blocks();

            int largestArea = -1;

            for (HuskyLens.Block block : blocks) {

                if (block.width <= 0 || block.height <= 0) {
                    continue;
                }

                int area = block.width * block.height;

                if (area > largestArea) {
                    largestArea = area;
                    target = block;
                }
            }

            if (target == null) {
                telemetry.addLine("NO APRILTAG DETECTED");
                telemetry.update();
                sleep(50);
            }
        }

        if (!opModeIsActive() || target == null) {
            return;
        }

        // =====================================
        // CALCULATE INITIAL ANGLE AND DISTANCE
        // =====================================

        angleDegrees = -Math.toDegrees(
                Math.atan((target.x - 160.0) / 442.0)
        );

        double estimatedTagDistance =
                CALIBRATION_DISTANCE_MM
                        * CALIBRATION_WIDTH_PIXELS
                        / target.width;

        // Stop approximately 200 mm before the tag
        DISTANCE_MM = Math.max(
                0.0,
                estimatedTagDistance - 200.0
        );

        telemetry.addData("Target Tag ID", target.id);
        telemetry.addData("Initial Angle", angleDegrees);
        telemetry.addData("Initial Tag Distance", estimatedTagDistance);
        telemetry.addData("Target Travel Distance", DISTANCE_MM);
        telemetry.update();

        // =====================================
        // RECORD STARTING POSITION
        // =====================================

        pinpoint.update();

        Pose2D startPosition = pinpoint.getPosition();

        startX = startPosition.getX(DistanceUnit.MM);
        startY = startPosition.getY(DistanceUnit.MM);

        startHeading =
                startPosition.getHeading(AngleUnit.DEGREES);

        distance = 0;
        remaining = DISTANCE_MM;

        timer.reset();

        moving = true;
        result = "Moving";

        // =====================================
        // AUTONOMOUS MOVEMENT
        // =====================================

        try {

            while (opModeIsActive() && moving) {

                // Read odometry
                pinpoint.update();

                Pose2D position = pinpoint.getPosition();

                double x = position.getX(DistanceUnit.MM);
                double y = position.getY(DistanceUnit.MM);

                double heading =
                        position.getHeading(AngleUnit.DEGREES);

                // Calculate movement from starting point
                double dx = x - startX;
                double dy = y - startY;

                distance = Math.hypot(dx, dy);

                // Continuously calculate remaining distance
                remaining = DISTANCE_MM - distance;

                // =====================================
                // STOPPING CONDITIONS
                // =====================================

                if (remaining <= TOLERANCE_MM) {

                    stopMotors();
                    moving = false;
                    result = "Target reached";

                } else if (timer.seconds() >= TIMEOUT) {

                    stopMotors();
                    moving = false;
                    result = "Timeout";

                } else {

                    // =====================================
                    // ORIGINAL MOVEMENT ANGLE MATH
                    // =====================================

                    double radians = Math.toRadians(angleDegrees);

                    double forward = Math.cos(radians);
                    double strafeRight = Math.sin(radians);

                    // =====================================
                    // ORIGINAL SLOWDOWN MATH
                    // =====================================

                    double speedFactor = Math.min(
                            1.0,
                            Math.max(0.5, remaining / 400.0)
                    );

                    // =====================================
                    // ORIGINAL HEADING CORRECTION
                    // =====================================

                    double error = startHeading - heading;

                    while (error > 180) {
                        error -= 360;
                    }

                    while (error < -180) {
                        error += 360;
                    }

                    double turn = error * HEADING_KP;

                    turn = Math.max(
                            -MAX_TURN,
                            Math.min(MAX_TURN, turn)
                    );

                    // =====================================
                    // ORIGINAL MECANUM MOTOR MATH
                    // =====================================

                    double lfPower =
                            (LF * forward
                                    + STRAFE_POWER * strafeRight
                                    - turn) * speedFactor;

                    double lrPower =
                            (LR * forward
                                    - STRAFE_POWER * strafeRight
                                    - turn) * speedFactor;

                    double rfPower =
                            (RF * forward
                                    - STRAFE_POWER * strafeRight
                                    + turn) * speedFactor;

                    double rrPower =
                            (RR * forward
                                    + STRAFE_POWER * strafeRight
                                    + turn) * speedFactor;

                    // Normalize motor powers
                    double max = Math.max(
                            1.0,
                            Math.max(
                                    Math.max(
                                            Math.abs(lfPower),
                                            Math.abs(lrPower)
                                    ),
                                    Math.max(
                                            Math.abs(rfPower),
                                            Math.abs(rrPower)
                                    )
                            )
                    );

                    lfPower /= max;
                    lrPower /= max;
                    rfPower /= max;
                    rrPower /= max;

                    // Apply motor powers
                    leftFront.setPower(lfPower);
                    leftRear.setPower(lrPower);
                    rightFront.setPower(rfPower);
                    rightRear.setPower(rrPower);

                    telemetry.addData("LF", "%.3f", lfPower);
                    telemetry.addData("LR", "%.3f", lrPower);
                    telemetry.addData("RF", "%.3f", rfPower);
                    telemetry.addData("RR", "%.3f", rrPower);
                }

                // =====================================
                // TELEMETRY
                // =====================================

                telemetry.addLine("=== APRILTAG ===");

                telemetry.addData("Target Tag ID", target.id);

                telemetry.addData(
                        "Initial Tag Distance", "%.2f",
                        estimatedTagDistance
                );

                telemetry.addData(
                        "Angle (deg)", "%.2f", angleDegrees
                );

                telemetry.addLine("=== DRIVE STATUS ===");

                telemetry.addData("Result", result);
                telemetry.addData("Moving", moving);

                telemetry.addLine("=== DISTANCE ===");

                telemetry.addData(
                        "Target (mm)", "%.1f", DISTANCE_MM
                );

                telemetry.addData(
                        "Distance (mm)", "%.2f", distance
                );

                telemetry.addData(
                        "Remaining (mm)", "%.2f", remaining
                );

                telemetry.addData(
                        "Elapsed (s)", "%.2f", timer.seconds()
                );

                telemetry.addLine("=== PINPOINT ===");

                telemetry.addData("X (mm)", "%.2f", x);
                telemetry.addData("Y (mm)", "%.2f", y);
                telemetry.addData("Heading (deg)", "%.2f", heading);

                telemetry.addData(
                        "Status", pinpoint.getDeviceStatus()
                );

                telemetry.update();

                sleep(20);
            }

        } finally {
            stopMotors();
        }
    }

    private void stopMotors() {

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}
