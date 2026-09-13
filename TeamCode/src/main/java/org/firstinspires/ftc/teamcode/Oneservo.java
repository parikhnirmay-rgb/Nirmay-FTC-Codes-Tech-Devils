package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo; // Import the Servo library

@TeleOp(name="Just a Servo", group="Linear OpMode")
public class Oneservo extends LinearOpMode { // Keeping class name TwoMotors so you don't have to rename your file!

    // 1. Declare the single servo variable
    private Servo clawServo = null;

    @Override
    public void runOpMode() {

        // 2. Hardware Mapping (the string "claw_servo" must match your phone config)
        clawServo = hardwareMap.get(Servo.class, "claw_servo");

        telemetry.addData("Status", "Servo Initialized Successfully!");
        telemetry.update();

        // Wait for the driver to press PLAY on the phone screen
        waitForStart();

        // 3. Main Loop
        while (opModeIsActive()) {

            // Press the 'X' button to rotate to position 0.0 (fully closed)
            if (gamepad1.x) {
                clawServo.setPosition(0.0);
            }
            // Press the 'Y' button to rotate to position 0.5 (halfway open)
            else if (gamepad1.y) {
                clawServo.setPosition(0.5);
            }
            // Press the 'B' button to rotate to position 1.0 (fully open)
            else if (gamepad1.b) {
                clawServo.setPosition(1.0);
            }

            // Display the servo's current position live on your phone screen
            telemetry.addData("Current Servo Position", clawServo.getPosition());
            telemetry.update();
        }
    }
}
