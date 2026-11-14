package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "HuskyLens BIOBUZZ Detector", group = "Vision")
public class ConceptTensorFlowObjectDetection extends LinearOpMode {

    // CALIBRATION SETTINGS
    // Place an AprilTag 300 mm from the camera lens.
    private static final double CALIBRATION_DISTANCE_MM = 300.0;

    // IMPORTANT:
    // Replace 0.0 with the tag width in pixels measured
    // when the tag is exactly 300 mm from the camera.
    private static final double CALIBRATION_WIDTH_PIXELS = 76;
    @Override
    public void runOpMode() {
        HuskyLens huskyLens =
                hardwareMap.get(HuskyLens.class, "huskylens");
        if (!huskyLens.knock()) {
            telemetry.addLine("ERROR: Cannot communicate with HuskyLens!");
            telemetry.addLine("Check wiring and robot configuration.");
            telemetry.update();
            return;
        }
        huskyLens.selectAlgorithm(
                HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.addLine("HuskyLens connected!");


// Read detections.
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Detections", blocks.length);
            if (blocks.length == 0) {
                telemetry.addLine("No objects detected.");
                telemetry.addLine("No distance estimate available.");
            }
            for (int i = 0; i < blocks.length; i++) {
                HuskyLens.Block block = blocks[i];
                telemetry.addData(
                        "Detection " + (i + 1),
                        "ID: %d | Center: (%d, %d)",
                        block.id, block.x, block.y);
                telemetry.addData(
                        "Pixel size " + (i + 1),
                        "%d x %d pixels",
                        block.width, block.height);

// Estimate distance only in AprilTag mode.
                if (block.width > 0) {
                    if (CALIBRATION_WIDTH_PIXELS > 0) {
                        double distanceMM =

                                CALIBRATION_DISTANCE_MM
                                        *
                                        CALIBRATION_WIDTH_PIXELS
                                        / block.width;
                        telemetry.addData(
                                "Estimated distance " + (i + 1),
                                "%.1f mm",
                                distanceMM);
                    } else {
                        telemetry.addData(
                                "Calibration needed",
                                "Measure tag width at %.0f mm",
                                CALIBRATION_DISTANCE_MM);
                    }
                }
            }
            telemetry.update();
            sleep(50);
        }
    }