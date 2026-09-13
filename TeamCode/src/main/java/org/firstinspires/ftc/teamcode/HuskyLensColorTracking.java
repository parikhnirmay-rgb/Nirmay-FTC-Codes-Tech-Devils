package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HuskyLens Color Tracking", group = "Sensor")
public class HuskyLensColorTracking extends LinearOpMode {

    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        // 1. Initialize HuskyLens from the Hardware Map
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        telemetry.addData("Status", "Initializing HuskyLens...");
        telemetry.update();

        // 2. Set the algorithm specifically to COLOR_RECOGNITION
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.addData("Status", "Ready. Train your color on-camera, then press Play!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 3. Get blocks using the updated .blocks() method for your SDK
            HuskyLens.Block[] blocks = huskyLens.blocks();

            telemetry.addData("Objects Found", blocks.length);

            for (HuskyLens.Block block : blocks) {
                // Check for your learned color (ID 1)
                if (block.id == 1) {
                    telemetry.addLine("\n--- Target Color Found! ---");
                    telemetry.addData("ID Number", block.id);
                    telemetry.addData("X Center", block.x); // Screen width is 0 to 320
                    telemetry.addData("Y Center", block.y); // Screen height is 0 to 240
                    telemetry.addData("Width", block.width);

                    // 4. Calculate tracking position relative to screen center (160)
                    if (block.x < 130) {
                        telemetry.addData("Alignment", "Color is to the LEFT 👈");
                    } else if (block.x > 190) {
                        telemetry.addData("Alignment", "Color is to the RIGHT 👉");
                    } else {
                        telemetry.addData("Alignment", "Color is CENTERED 🎯");
                    }
                }
            }

            if (blocks.length == 0) {
                telemetry.addData("Status", "Searching for the trained color...");
            }

            telemetry.update();
            sleep(50); // Prevent I2C bus flooding
        }
    }
}
