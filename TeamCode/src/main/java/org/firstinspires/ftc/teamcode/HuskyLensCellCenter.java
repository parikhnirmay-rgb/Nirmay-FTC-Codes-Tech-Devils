
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HuskyLens Cell Center", group = "Vision")
public class HuskyLensCellCenter extends LinearOpMode {

    // HUSKYLENS image dimensions
    private static final double IMAGE_CENTER_X = 160.0;
    private static final double IMAGE_CENTER_Y = 120.0;

    // =================================================
    // CALIBRATION AND CELL CENTER OFFSETS
    // =================================================

    // Measure the physical distance between the centers
    // of the FIRST and LAST visible AprilTags.
    // Replace 0.0 with your actual measurement in mm.
    private static final double TAG_CENTER_SEPARATION_MM = 0.0;

    // Offset from the midpoint of the two tag centers
    // to the actual center of the CELL opening.

    // Positive X = toward the rightmost tag.
    private static final double CELL_CENTER_OFFSET_X_MM = 0.0;

    // Positive Y = perpendicular to the tag line,
    // toward the TOP of the camera image.
    private static final double CELL_CENTER_OFFSET_Y_MM = 0.0;

    // =================================================

    @Override
    public void runOpMode() {

        // Connect to HUSKYLENS
        HuskyLens huskyLens =
                hardwareMap.get(HuskyLens.class, "huskylens");

        // Check communication
        if (!huskyLens.knock()) {
            telemetry.addLine("ERROR: HUSKYLENS NOT CONNECTED");
            telemetry.update();
            return;
        }

        // Activate AprilTag recognition
        huskyLens.selectAlgorithm(
                HuskyLens.Algorithm.TAG_RECOGNITION
        );

        telemetry.addLine("HUSKYLENS CONNECTED");
        telemetry.addLine("Looking for two AprilTags");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Read all visible AprilTags
            HuskyLens.Block[] blocks = huskyLens.blocks();

            telemetry.addData(
                    "Tags detected",
                    blocks.length
            );

            // Need at least two detections
            if (blocks.length < 2) {

                telemetry.addLine(
                        "WAITING FOR FIRST AND LAST TAG"
                );

                telemetry.update();
                sleep(50);
                continue;
            }

            // =================================================
            // FIND LEFTMOST AND RIGHTMOST TAGS
            // =================================================

            HuskyLens.Block leftTag = null;
            HuskyLens.Block rightTag = null;

            for (HuskyLens.Block block : blocks) {

                if (leftTag == null || block.x < leftTag.x) {
                    leftTag = block;
                }

                if (rightTag == null || block.x > rightTag.x) {
                    rightTag = block;
                }
            }

            if (leftTag == rightTag) {
                telemetry.addLine("TWO DISTINCT TAGS REQUIRED");
                telemetry.update();
                sleep(50);
                continue;
            }

            // =================================================
            // GET TAG CENTERS
            // =================================================

            double leftX = leftTag.x;
            double leftY = leftTag.y;

            double rightX = rightTag.x;
            double rightY = rightTag.y;

            telemetry.addData(
                    "Left Tag",
                    "X=%.1f Y=%.1f ID=%d",
                    leftX, leftY, leftTag.id
            );

            telemetry.addData(
                    "Right Tag",
                    "X=%.1f Y=%.1f ID=%d",
                    rightX, rightY, rightTag.id
            );

            // =================================================
            // CALCULATE MIDPOINT
            // =================================================

            double middleX = (leftX + rightX) / 2.0;
            double middleY = (leftY + rightY) / 2.0;

            telemetry.addLine("=== TAG MIDPOINT ===");

            telemetry.addData("Midpoint X", "%.2f", middleX);
            telemetry.addData("Midpoint Y", "%.2f", middleY);

            // =================================================
            // CALCULATE PIXEL DISTANCE BETWEEN TAGS
            // =================================================

            double dx = rightX - leftX;
            double dy = rightY - leftY;

            double pixelSeparation =
                    Math.hypot(dx, dy);

            if (pixelSeparation < 5.0) {

                telemetry.addLine("TAGS TOO CLOSE TOGETHER");
                telemetry.update();
                sleep(50);
                continue;
            }

            // Unit direction along the line between tags
            double unitX = dx / pixelSeparation;
            double unitY = dy / pixelSeparation;

            // Perpendicular direction toward the top
            // of the camera image
            double perpendicularX = unitY;
            double perpendicularY = -unitX;

            // =================================================
            // CONVERT OFFSETS FROM MM TO PIXELS
            // =================================================

            telemetry.addLine("=== CELL CENTER ===");

            if (TAG_CENTER_SEPARATION_MM > 0.0) {

                // Estimate pixels per millimeter
                double pixelsPerMM =
                        pixelSeparation
                                / TAG_CENTER_SEPARATION_MM;

                // Apply X and Y offsets
                double offsetPixelX =
                        (CELL_CENTER_OFFSET_X_MM * unitX
                                + CELL_CENTER_OFFSET_Y_MM * perpendicularX)
                                * pixelsPerMM;

                double offsetPixelY =
                        (CELL_CENTER_OFFSET_X_MM * unitY
                                + CELL_CENTER_OFFSET_Y_MM * perpendicularY)
                                * pixelsPerMM;

                // Estimated center of the CELL opening
                double cellCenterX =
                        middleX + offsetPixelX;

                double cellCenterY =
                        middleY + offsetPixelY;

                // Display estimated cell center
                telemetry.addData(
                        "CELL CENTER X",
                        "%.2f pixels",
                        cellCenterX
                );

                telemetry.addData(
                        "CELL CENTER Y",
                        "%.2f pixels",
                        cellCenterY
                );

                telemetry.addData(
                        "Pixels per mm",
                        "%.4f",
                        pixelsPerMM
                );

                // Error relative to the camera center
                double errorX =
                        cellCenterX - IMAGE_CENTER_X;

                double errorY =
                        IMAGE_CENTER_Y - cellCenterY;

                telemetry.addData(
                        "Horizontal error",
                        "%.2f pixels",
                        errorX
                );

                telemetry.addData(
                        "Vertical error",
                        "%.2f pixels",
                        errorY
                );

                // Basic image alignment guidance
                if (errorX < -15) {
                    telemetry.addLine("CELL CENTER: LEFT");
                } else if (errorX > 15) {
                    telemetry.addLine("CELL CENTER: RIGHT");
                } else {
                    telemetry.addLine("CELL CENTER: HORIZONTALLY CENTERED");
                }

                if (errorY < -15) {
                    telemetry.addLine("CELL CENTER: BELOW IMAGE CENTER");
                } else if (errorY > 15) {
                    telemetry.addLine("CELL CENTER: ABOVE IMAGE CENTER");
                } else {
                    telemetry.addLine("CELL CENTER: VERTICALLY CENTERED");
                }

            } else {

                telemetry.addLine(
                        "MIDPOINT FOUND, BUT CALIBRATION NEEDED"
                );

                telemetry.addLine(
                        "Set TAG_CENTER_SEPARATION_MM"
                );
            }

            // =================================================

            telemetry.addLine(
                    "No automatic motor movement"
            );

            telemetry.update();

            sleep(50);
        }
    }
}
