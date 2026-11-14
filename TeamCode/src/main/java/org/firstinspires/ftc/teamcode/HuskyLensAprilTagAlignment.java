package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "HuskyLens AprilTag Alignment", group = "Vision")
public class HuskyLensAprilTagAlignment extends LinearOpMode {

    // Original HUSKYLENS image is 320 x 240 pixels.
    private static final int CENTER_X = 160;
    private static final int CENTER_Y = 120;

    // Allow a little error around the center of the image.
    private static final int CENTER_TOLERANCE = 15;

    // Ignore tiny width changes caused by detection jitter.
    private static final int WIDTH_CHANGE_THRESHOLD = 2;

    // Rough shape thresholds, not precise angle measurements.
    private static final double NARROW_RATIO = 0.75;
    private static final double TALL_RATIO = 1.33;
    private static final double CALIBRATION_WIDTH_PIXELS = 76.0;
    private static final double CALIBRATION_DISTANCE_MM = 300.0;

    @Override
    public void runOpMode() {
        HuskyLens huskyLens =
                hardwareMap.get(HuskyLens.class, "huskylens");
        if (!huskyLens.knock()) {
            telemetry.addLine("ERROR: HUSKYLENS not responding");
            telemetry.addLine("Check I2C wiring and configuration.");
            telemetry.update();
            return;
        }
        huskyLens.selectAlgorithm(
                HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.addLine("HUSKYLENS connected");
        telemetry.addLine("Point camera at an AprilTag.");
        telemetry.addLine("Turn robot slowly to test alignment.");
        telemetry.update();
        waitForStart();
        int previousWidth = -1;
        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            if (blocks.length == 0) {
                telemetry.addLine("NO APRILTAG DETECTED");
                telemetry.addLine("Move the camera to find a tag.");
                previousWidth = -1;
                telemetry.update();
                sleep(50);
                continue;
            }

// Pick the tag with the largest bounding-box area.
            // This often selects the closest clearly visible tag.
            HuskyLens.Block target = blocks[0];
            int largestArea = -1;
            for (HuskyLens.Block block : blocks) {
                int area = block.width * block.height;
                if (area > largestArea) {
                    largestArea = area;
                    target = block;
                }
            }
            int xError = target.x - CENTER_X;
            int yError = target.y - CENTER_Y;
            double shapeRatio =
                    (double) target.
                            width
                            / Math.max(1, target.height);
            double angleDegrees = Math.toDegrees(
                    Math.atan((target.x - 160.0) / 442.0)
            );
            double estimatedTagDistance =
                    CALIBRATION_DISTANCE_MM
                            * CALIBRATION_WIDTH_PIXELS
                            / target.width;
            telemetry.addData("Angle of Tag", angleDegrees);
            telemetry.addData("Target tag ID", target.id);
            telemetry.addData(
                    "Tag center",
                    "X=%d Y=%d",
                    target.x, target.y);
            telemetry.addData(
                    "Frame center",
                    "X=%d Y=%d",
                    CENTER_X, CENTER_Y);
            telemetry.addData(
                    "Horizontal error",
                    "%d pixels",
                    xError);
            telemetry.addData(
                    "Tag size",
                    "%d x %d pixels",
                    target.width, target.height);
            telemetry.addData(
                    "Width / height ratio",
                    "%.2f",
                    shapeRatio);
            telemetry.addData("Distance:", estimatedTagDistance);

// CENTER THE TAG HORIZONTALLY.
            if (xError < -CENTER_TOLERANCE) {
                telemetry.addLine(
                        "GUIDANCE: TURN LEFT to face the tag.");
            } else if (xError > CENTER_TOLERANCE) {
                telemetry.addLine(
                        "GUIDANCE: TURN RIGHT to face the tag.");
            } else {
                telemetry.addLine(
                        "HORIZONTAL: TAG CENTERED.");
            }

// VERTICAL IMAGE ALIGNMENT.
            if (yError < -CENTER_TOLERANCE) {
                telemetry.addLine(
                        "IMAGE: Tag is ABOVE the frame center.");
            } else if (yError > CENTER_TOLERANCE) {
                telemetry.addLine(
                        "IMAGE: Tag is BELOW the frame center.");
            } else {
                telemetry.addLine(
                        "IMAGE: Tag is vertically centered.");
            }

// ROUGH SHAPE CHECK.
            if (shapeRatio < NARROW_RATIO) {
                telemetry.addLine(
                        "SHAPE: Tag looks horizontally narrow.");
                telemetry.addLine(
                        "It may be viewed at an oblique angle.");
            } else if (shapeRatio > TALL_RATIO) {
                telemetry.addLine(
                        "SHAPE: Tag looks tall or rotated.");
                telemetry.addLine(
                        "Check camera tilt and tag orientation.");
            } else {
                telemetry.addLine(
                        "SHAPE: No strong narrowing detected.");
            }

// WIDTH TREND HELPS WHILE YOU TEST A TURN.
            // Keep your distance from the tag roughly constant.
            if (previousWidth > 0 && target.width > 0) {
                int widthChange =
                        target.width - previousWidth;
                if (shapeRatio < NARROW_RATIO) {
                    if (widthChange > WIDTH_CHANGE_THRESHOLD) {
                        telemetry.addLine(
                                "WIDTH INCREASING: your test turn");
                        telemetry.addLine(
                                "may be moving toward face-on.");
                    } else if (
                            widthChange < -WIDTH_CHANGE_THRESHOLD) {
                        telemetry.addLine(
                                "WIDTH DECREASING: your test turn");
                        telemetry.addLine(
                                "may be moving away from face-on.");
                    } else {
                        telemetry.addLine(
                                "WIDTH STABLE: no clear trend yet.");
                    }
                }
            }
            telemetry.addLine(
                    "Note: shape ratio cannot tell turn direction alone.");
            telemetry.update();
            previousWidth = target.width;
            sleep(50);
        }
    }
}