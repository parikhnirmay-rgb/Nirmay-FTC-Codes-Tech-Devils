
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Diagonal Distance Drive", group = "Linear OpMode")
public class Autonomous4points extends LinearOpMode {

    // Motors
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    // Odometry
    private GoBildaPinpointDriver pinpoint;

    // Target distance: 2 meters = 2000 mm
    private static final double DISTANCE_MM = 2000.0;

    // Distance tolerance
    private static final double TOLERANCE_MM = 20.0;

    // Your calibrated forward powers
    private static final double LF = 0.16;
    private static final double LR = 0.15;
    private static final double RF = 0.26;
    private static final double RR = 0.27;

    // Strafe strength (starting estimate)
    private static final double STRAFE_POWER = 0.20;

    // Heading correction
    private static final double HEADING_KP = 0.002;
    private static final double MAX_TURN = 0.04;

    // Safety timeout
    private static final double TIMEOUT = 10.0;

    // Movement state
    private boolean moving = false;

    // Button state
    private boolean lastY = false;
    private boolean lastB = false;
    private boolean lastA = false;
    private boolean lastX = false;

    // Starting pose
    private double startX = 0;
    private double startY = 0;
    private double startHeading = 0;

    // Selected movement angle
    private double angleDegrees = 0;

    // Motor timer
    private final ElapsedTime timer = new ElapsedTime();

    // Last result
    private String result = "Ready";

    // Current distance
    private double distance = 0;
    private double remaining = DISTANCE_MM;

    @Override
    public void runOpMode() {

        // =====================================
        // INITIALIZE MOTORS
        // =====================================

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        // Brake when motors stop
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

        // Your offsets
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

        // Reset position and calibrate IMU
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Robot Initialized");
        telemetry.addLine("Y = Forward 0 degrees");
        telemetry.addLine("B = Right diagonal 15 degrees");
        telemetry.addLine("A = Right diagonal 30 degrees");
        telemetry.addLine("X = Right diagonal 45 degrees");
        telemetry.addLine("BACK = Stop movement");
        telemetry.addLine("Target = 2000 mm");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // =====================================
        // MAIN LOOP
        // =====================================

        try {

            while (opModeIsActive()) {

                // Read odometry
                pinpoint.update();

                Pose2D position = pinpoint.getPosition();

                double x = position.getX(DistanceUnit.MM);
                double y = position.getY(DistanceUnit.MM);
                double heading = position.getHeading(AngleUnit.DEGREES);

                // Read buttons
                boolean yPressed = gamepad1.y;
                boolean bPressed = gamepad1.b;
                boolean aPressed = gamepad1.a;
                boolean xPressed = gamepad1.x;

                // =====================================
                // STOP WITH BACK BUTTON
                // =====================================

                if (gamepad1.back) {
                    stopMotors();
                    moving = false;
                    result = "Cancelled";
                }

                // =====================================
                // BUTTON SELECTION
                // =====================================

                if (!moving && !gamepad1.back) {

                    boolean startMovement = false;
                    double selectedAngle = 0;

                    if (yPressed && !lastY) {
                        selectedAngle = 0;
                        startMovement = true;

                    } else if (bPressed && !lastB) {
                        selectedAngle = 15;
                        startMovement = true;

                    } else if (aPressed && !lastA) {
                        selectedAngle = 30;
                        startMovement = true;

                    } else if (xPressed && !lastX) {
                        selectedAngle = 45;
                        startMovement = true;
                    }

                    if (startMovement) {

                        angleDegrees = selectedAngle;

                        // Record initial robot position
                        startX = x;
                        startY = y;
                        startHeading = heading;

                        distance = 0;
                        remaining = DISTANCE_MM;

                        timer.reset();

                        moving = true;
                        result = "Moving";
                    }
                }

                // Save button states
                lastY = yPressed;
                lastB = bPressed;
                lastA = aPressed;
                lastX = xPressed;

                // =====================================
                // AUTONOMOUS MOVEMENT
                // =====================================

                if (moving) {

                    // Calculate movement from starting point
                    double dx = x - startX;
                    double dy = y - startY;

                    // NEW: Calculate straight-line displacement
                    distance = Math.hypot(dx, dy);

                    // Calculate remaining distance
                    remaining = DISTANCE_MM - distance;

                    // Check stopping conditions
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
                        // MOVEMENT ANGLE
                        // =====================================

                        double radians = Math.toRadians(angleDegrees);

                        // Forward and sideways components
                        double forward = Math.cos(radians);
                        double strafeRight = Math.sin(radians);

                        // =====================================
                        // SLOW DOWN NEAR TARGET
                        // =====================================

                        double speedFactor = Math.min(
                                1.0,
                                Math.max(0.5, remaining / 400.0)
                        );

                        // =====================================
                        // HEADING CORRECTION
                        // =====================================

                        double error = startHeading - heading;

                        // Normalize angle error
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
                        // MECANUM MOTOR POWER
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

                        // Normalize motor powers if necessary
                        double max = Math.max(
                                1.0,
                                Math.max(
                                        Math.max(Math.abs(lfPower), Math.abs(lrPower)),
                                        Math.max(Math.abs(rfPower), Math.abs(rrPower))
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
                    }

                } else {
                    stopMotors();
                }

                // =====================================
                // TELEMETRY
                // =====================================

                telemetry.addLine("=== DRIVE STATUS ===");

                telemetry.addData("Result", result);
                telemetry.addData("Moving", moving);
                telemetry.addData("Angle (deg)", "%.1f", angleDegrees);

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

    // =====================================
    // STOP MOTORS
    // =====================================

    private void stopMotors() {

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}
