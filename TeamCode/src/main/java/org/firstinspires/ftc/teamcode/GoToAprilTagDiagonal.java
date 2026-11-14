
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Strafe To AprilTag", group = "Autonomous")
public class GoToAprilTagDiagonal extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private GoBildaPinpointDriver pinpoint;

    private static final int CENTER_X = 160;
    private static final int CENTER_Y = 120;
    private static final int CENTER_TOLERANCE = 15;
    private static final int WIDTH_CHANGE_THRESHOLD = 2;

    private static final double NARROW_RATIO = 0.75;
    private static final double TALL_RATIO = 1.33;
    private static final double CALIBRATION_WIDTH_PIXELS = 76.0;
    private static final double CALIBRATION_DISTANCE_MM = 300.0;

    @Override
    public void runOpMode() {

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

        // Pinpoint
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint"
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

        // HuskyLens
        HuskyLens huskyLens =
                hardwareMap.get(HuskyLens.class, "huskylens");

        if (!huskyLens.knock()) {
            telemetry.addLine("HuskyLens not responding");
            telemetry.update();
            return;
        }

        huskyLens.selectAlgorithm(
                HuskyLens.Algorithm.TAG_RECOGNITION
        );

        telemetry.addLine("Ready to move toward AprilTag");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        pinpoint.update();

        Pose2D startPosition = pinpoint.getPosition();

        double startX = startPosition.getX(DistanceUnit.MM);
        double startY = startPosition.getY(DistanceUnit.MM);

        // Your original calibrated diagonal powers
        double speed1 = 0.35;
        double speed2 = 0.25;

        int previousWidth = -1;

// Find the AprilTag before starting movement
        HuskyLens.Block target = null;

        while (opModeIsActive() && target == null) {

            HuskyLens.Block[] blocks = huskyLens.blocks();
            int largestArea = -1;

            for (HuskyLens.Block block : blocks) {

                if (block.width > 0 && block.height > 0) {

                    int area = block.width * block.height;

                    if (area > largestArea) {
                        largestArea = area;
                        target = block;
                    }
                }
            }

            if (target == null) {
                telemetry.addLine("Waiting for AprilTag");
                telemetry.update();
                sleep(50);
            }
        }

        if (!opModeIsActive()) return;

// Calculate initial angle and distance ONCE
        double angleDegrees = Math.toDegrees(
                Math.atan((target.x - 160.0) / 442.0)
        );

        double angle = Math.toRadians(angleDegrees);

        double estimatedTagDistance =
                CALIBRATION_DISTANCE_MM
                        * CALIBRATION_WIDTH_PIXELS
                        / target.width;

        telemetry.addData("Initial Angle", angleDegrees);
        telemetry.addData("Initial Distance", estimatedTagDistance);
        telemetry.update();

        try {
            while (opModeIsActive()) {

                HuskyLens.Block[] blocks = huskyLens.blocks();

                if (blocks.length == 0) {

                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);

                    telemetry.addLine("NO APRILTAG DETECTED");
                    telemetry.update();

                    previousWidth = -1;
                    sleep(50);
                    continue;
                }

                // Find largest visible AprilTag
                target = blocks[0];
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

                if (largestArea <= 0) {
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    sleep(50);
                    continue;
                }

                // Camera measurements
                int xError = target.x - CENTER_X;
                int yError = target.y - CENTER_Y;

                double shapeRatio =
                        (double) target.width
                                / Math.max(1, target.height);

                double forward = Math.cos(angle);
                double strafe = -Math.sin(angle);

                // 0 = straight, 1 = 45-degree diagonal
                double speed = Math.min(
                        1.0,
                        Math.abs(strafe / Math.max(0.001, forward))
                );

                // Calibrated forward motor powers blended
                // with your original diagonal powers.
                double lf, lr, rf, rr;

                if (strafe <= 0) {

                    // Tag on right
                    lf = 0.16 + (speed1 - 0.16) * speed;
                    lr = 0.15 * (1.0 - speed);
                    rf = 0.26 * (1.0 - speed);
                    rr = 0.27 + (speed2 - 0.27) * speed;

                } else {

                    // Tag on left (estimated mirror calibration)
                    lf = 0.16 * (1.0 - speed);
                    lr = 0.15 + (speed1 - 0.15) * speed;
                    rf = 0.26 + (speed2 - 0.26) * speed;
                    rr = 0.27 * (1.0 - speed);
                }

                // Update Pinpoint while moving
                pinpoint.update();

                Pose2D position = pinpoint.getPosition();

                double x = position.getX(DistanceUnit.MM);
                double y = position.getY(DistanceUnit.MM);

                double dx = x - startX;
                double dy = y - startY;

                double traveledDistance = Math.hypot(dx, dy);
                double heading =
                        position.getHeading(AngleUnit.DEGREES);
                double remaining =
                        estimatedTagDistance - traveledDistance;

                // Stop when estimated tag distance is 200 mm
                if (remaining <= 0) {

                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);

                    telemetry.addLine("TARGET REACHED");
                    telemetry.addData(
                            "Tag Distance", estimatedTagDistance);
                    telemetry.addData(
                            "Traveled", traveledDistance);
                    telemetry.addData("Heading", heading);
                    telemetry.update();

                    break;
                }

                // Drive using calibrated motor powers
                leftFront.setPower(lf);
                leftRear.setPower(lr);
                rightFront.setPower(rf);
                rightRear.setPower(rr);

                // Telemetry
                telemetry.addData("Tag ID", target.id);
                telemetry.addData("Tag X", target.x);
                telemetry.addData("Tag Y", target.y);
                telemetry.addData("Angle", "%.2f", angleDegrees);
                telemetry.addData(
                        "Distance", "%.1f", estimatedTagDistance);
                telemetry.addData(
                        "Traveled", "%.1f", traveledDistance);
                telemetry.addData("Heading", "%.2f", heading);

                telemetry.addData("LF", "%.3f", lf);
                telemetry.addData("LR", "%.3f", lr);
                telemetry.addData("RF", "%.3f", rf);
                telemetry.addData("RR", "%.3f", rr);

                if (xError < -CENTER_TOLERANCE) {
                    telemetry.addLine("TAG LEFT");
                } else if (xError > CENTER_TOLERANCE) {
                    telemetry.addLine("TAG RIGHT");
                } else {
                    telemetry.addLine("TAG CENTERED");
                }

                if (yError < -CENTER_TOLERANCE) {
                    telemetry.addLine("TAG ABOVE");
                } else if (yError > CENTER_TOLERANCE) {
                    telemetry.addLine("TAG BELOW");
                } else {
                    telemetry.addLine("TAG VERTICALLY CENTERED");
                }

                if (shapeRatio < NARROW_RATIO) {
                    telemetry.addLine("Tag looks narrow");
                } else if (shapeRatio > TALL_RATIO) {
                    telemetry.addLine("Tag looks tall");
                }

                if (previousWidth > 0) {

                    int widthChange = target.width - previousWidth;

                    if (Math.abs(widthChange)
                            > WIDTH_CHANGE_THRESHOLD) {

                        telemetry.addData(
                                "Width Change", widthChange);
                    }
                }

                telemetry.update();

                previousWidth = target.width;
                sleep(50);
            }

        } finally {

            // Always stop motors
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
    }
}
