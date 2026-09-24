package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="My First OpMode", group="Linear OpMode")
public class OneMotor extends LinearOpMode {

    // 1. Declare your robot's parts here (Motors, Servos, etc.)
    private DcMotor leftDrive = null;

    @Override
    public void runOpMode() {
        // 1. THIS MUST HAPPEN FIRST: Map the motor hardware name
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");

        // 2. Initialize your telemetry screens
        telemetry.addData("Status", "Initialized Successfully!");
        telemetry.update();

        // 3. Wait for the game to start (Driver presses PLAY)
        waitForStart();

        // 4. Your main operational loop
        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            leftDrive.setPower(power);

            telemetry.addData("Motor Power", power);
            telemetry.update();
        }
    }
}
