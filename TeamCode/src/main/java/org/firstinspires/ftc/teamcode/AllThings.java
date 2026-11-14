
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Robot Movement Controls", group = "TeleOp")
public class AllThings extends LinearOpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private GoBildaPinpointDriver pinpoint;

    // Your calibrated straight motor powers
    private static final double LF = 0.16;
    private static final double LR = 0.15;
    private static final double RF = 0.26;
    private static final double RR = 0.27;

    // Sideways power (initial calibration)
    private static final double STRAFE_POWER = 0.25;

    // Turning settings
    private static final double TURN_KP = 0.01;
    private static final double MAX_TURN_POWER = 0.25;
    private static final double TURN_TOLERANCE = 2.0;

    @Override
    public void runOpMode() {

        // Initialize motors
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

        // Initialize Pinpoint
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

        telemetry.addLine("Y: Forward | A: Backward");
        telemetry.addLine("X: Strafe Left | B: Strafe Right");
        telemetry.addLine("D-pad Left: -90 | Right: +90 | Up: 0");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                pinpoint.update();
                Pose2D position = pinpoint.getPosition();

                double heading =
                        position.getHeading(AngleUnit.DEGREES);

                double targetHeading = 0;
                double error = 0;
                double turn = 0;

                // ==============================
                // FORWARD
                // ==============================
                if (gamepad1.y) {

                    leftFront.setPower(LF);
                    leftRear.setPower(LR);
                    rightFront.setPower(RF);
                    rightRear.setPower(RR);

                }

                // ==============================
                // BACKWARD
                // ==============================
                else if (gamepad1.a) {

                    leftFront.setPower(-LF);
                    leftRear.setPower(-LR);
                    rightFront.setPower(-RF);
                    rightRear.setPower(-RR);

                }

                // ==============================
                // STRAFE LEFT
                // ==============================
                else if (gamepad1.x) {

                    leftFront.setPower(-STRAFE_POWER);
                    leftRear.setPower(STRAFE_POWER);
                    rightFront.setPower(STRAFE_POWER);
                    rightRear.setPower(-STRAFE_POWER);

                }

                // ==============================
                // STRAFE RIGHT
                // ==============================
                else if (gamepad1.b) {

                    leftFront.setPower(STRAFE_POWER);
                    leftRear.setPower(-STRAFE_POWER);
                    rightFront.setPower(-STRAFE_POWER);
                    rightRear.setPower(STRAFE_POWER);

                }

                // ==============================
                // TURN TO SPECIFIC HEADING
                // ==============================
                else if (gamepad1.dpad_left ||
                        gamepad1.dpad_right ||
                        gamepad1.dpad_up) {

                    if (gamepad1.dpad_left) {
                        targetHeading = -90;
                    } else if (gamepad1.dpad_right) {
                        targetHeading = 90;
                    } else {
                        targetHeading = 0;
                    }

                    error = targetHeading - heading;

                    // Normalize heading error
                    while (error > 180) error -= 360;
                    while (error < -180) error += 360;

                    if (Math.abs(error) <= TURN_TOLERANCE) {
                        stopMotors();

                    } else {

                        turn = Math.max(
                                -MAX_TURN_POWER,
                                Math.min(
                                        MAX_TURN_POWER,
                                        TURN_KP * error
                                )
                        );

                        // Positive turn = CCW
                        leftFront.setPower(-turn);
                        leftRear.setPower(-turn);
                        rightFront.setPower(turn);
                        rightRear.setPower(turn);
                    }
                }

                // ==============================
                // NO BUTTON
                // ==============================
                else {
                    stopMotors();
                }

                // ==============================
                // TELEMETRY
                // ==============================
                telemetry.addData("Heading", "%.2f", heading);
                telemetry.addData("Target Heading", "%.2f", targetHeading);
                telemetry.addData("Turn Error", "%.2f", error);
                telemetry.addData(
                        "X (mm)", "%.1f",
                        position.getX(DistanceUnit.MM)
                );
                telemetry.addData(
                        "Y (mm)", "%.1f",
                        position.getY(DistanceUnit.MM)
                );

                telemetry.update();
                sleep(20);
            }

        } finally {
            stopMotors();
        }
    }

    private void stopMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}
