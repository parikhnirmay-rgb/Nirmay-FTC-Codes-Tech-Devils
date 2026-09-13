package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HuskyLens Ball Tracking", group = "Sensor")
public class AIBallTrackingWCamera extends LinearOpMode {

    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        // 1. Change algorithm to OBJECT_TRACKING
        // Note: Use HuskyLens.Algorithm.OBJECT_RECOGNITION instead if you want built-in AI classes
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);

        telemetry.addData("Status", "Ready. Press Play!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();

            for (HuskyLens.Block block : blocks) {
                // Check if the detected object matches the ID of your learned ball
                if (block.id == 9) {
                    telemetry.addLine("\n--- Ball Detected! ---");
                    telemetry.addData("X Center", block.x); // X center coordinate (0-320)
                    telemetry.addData("Y Center", block.y); // Y center coordinate (0-240)
                    telemetry.addData("Width", block.width);
                    telemetry.addData("Height", block.height);

                    // Simple size estimation (larger width = closer ball)
                    if (block.width > 150) {
                        telemetry.addLine("Distance: Ball is VERY CLOSE");
                    } else {
                        telemetry.addLine("Distance: Ball is FAR AWAY");
                    }
                }
            }

            if (blocks.length == 0) {
                telemetry.addData("Status", "Looking for the ball...");
            }

            telemetry.update();
            sleep(50);
        }
    }
}
