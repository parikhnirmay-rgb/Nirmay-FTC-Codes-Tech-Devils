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
        // 2. Map your hardware (the string "left_drive" must match your REV Hub config)
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");

        telemetry.addData("Status", "Initialized Successfully!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY on the Driver Station)
        waitForStart();

        // 3. This loop runs continuously until the driver presses STOP
        while (opModeIsActive()) {
            // Push stick forward to go forward, pull back to go backward
            double power = -gamepad1.left_stick_y;
            leftDrive.setPower(power);

            // Send live data back to your controller screen
            telemetry.addData("Motor Power", power);
            telemetry.update();
        }
    }
}
