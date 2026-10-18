
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@TeleOp(name = "Mecanum Pinpoint Drive", group = "Linear OpMode")
public class testpinpoint extends LinearOpMode {

    // Four drive motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    // Pinpoint Odometry Computer
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        // ==============================
        // INITIALIZE MOTORS
        // ==============================

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Reverse left motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // Brake when power is zero
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ==============================
        // INITIALIZE PINPOINT
        // ==============================

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        // Replace with measured offsets
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

        telemetry.addLine("Robot Initialized!");
        telemetry.addLine("Keep robot still during Pinpoint calibration.");
        telemetry.addLine("Hold Y to move forward at 20% power.");
        telemetry.update();

        waitForStart();

        // ==============================
        // MAIN LOOP
        // ==============================

        while (opModeIsActive()) {

            // --------------------------
            // MOTOR CONTROL
            // --------------------------

            double power = 0.0;

            // Hold Y to move forward
            if (gamepad1.y) {
                power = 0.2;
            }

            // Apply power to all 4 motors
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightFront.setPower(power);
            rightRear.setPower(power);

            // --------------------------
            // PINPOINT ODOMETRY
            // --------------------------

            // Update Pinpoint readings
            pinpoint.update();

            // Read position
            Pose2D position = pinpoint.getPosition();

            double x = position.getX(DistanceUnit.MM);
            double y = position.getY(DistanceUnit.MM);
            double heading = position.getHeading(AngleUnit.DEGREES);

            // --------------------------
            // TELEMETRY
            // --------------------------

            telemetry.addLine("=== MOTOR STATUS ===");

            telemetry.addData("Y Button", gamepad1.y);
            telemetry.addData("Motor Power", "%.2f", power);

            telemetry.addLine("=== PINPOINT STATUS ===");

            telemetry.addData("Status", pinpoint.getDeviceStatus());

            telemetry.addLine("=== ENCODERS ===");

            telemetry.addData("Encoder X", pinpoint.getEncoderX());
            telemetry.addData("Encoder Y", pinpoint.getEncoderY());

            telemetry.addLine("=== POSITION (MM) ===");

            telemetry.addData("X (mm)", "%.2f", x);
            telemetry.addData("Y (mm)", "%.2f", y);
            telemetry.addData("Heading (deg)", "%.2f", heading);

            telemetry.addLine("=== POSITION (INCHES) ===");

            telemetry.addData(
                    "X (in)",
                    "%.2f",
                    position.getX(DistanceUnit.INCH)
            );

            telemetry.addData(
                    "Y (in)",
                    "%.2f",
                    position.getY(DistanceUnit.INCH)
            );

            telemetry.addLine("=== VELOCITY ===");

            telemetry.addData(
                    "X Velocity (mm/s)",
                    "%.2f",
                    pinpoint.getVelX(DistanceUnit.MM)
            );

            telemetry.addData(
                    "Y Velocity (mm/s)",
                    "%.2f",
                    pinpoint.getVelY(DistanceUnit.MM)
            );

            telemetry.update();

            sleep(20);
        }

        // Stop all motors when OpMode ends
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}
