
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@Autonomous(name = "Strafe 45 Right", group = "Autonomous")
public class Strafe45Right extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        // Brake when stopped
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        // Your measured offsets
        pinpoint.setOffsets(90, 0, DistanceUnit.MM);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();
        Pose2D startPosition = pinpoint.getPosition();

        double startX =
                startPosition.getX(DistanceUnit.MM);

        double startY =
                startPosition.getY(DistanceUnit.MM);
        pinpoint.update();
        Pose2D position = pinpoint.getPosition();
        double x = position.getX(DistanceUnit.MM);
        double y = position.getY(DistanceUnit.MM);
        // Distance traveled from start
        double dx = x - startX;
        double dy = y - startY;
        double traveledDistance =
                Math.hypot(dx, dy);
        double heading = position.getHeading(AngleUnit.DEGREES);

        double angle = Math.toRadians(30);
        double speed = 0;
        double speed1 = 0.35;
        double speed2 = 0.25;

        // Forward and rightward movement
        double forward = Math.cos(angle);
        double strafe = Math.sin(angle);

        // Mecanum mixing
        double lf = forward + strafe;
        double lr = forward - strafe;
        double rf = forward - strafe;
        double rr = forward + strafe;

        // Normalize motor powers
        double max = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(lf), Math.abs(lr)),
                        Math.max(Math.abs(rf), Math.abs(rr))
                )
        );

        lf = lf / max * speed1;
        lr = lr / max * speed;
        rf = rf / max * speed;
        rr = rr / max * speed2;

        telemetry.addLine("Ready to move 45 degrees right");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // Move diagonally
            leftFront.setPower(lf);
            leftRear.setPower(lr);
            rightFront.setPower(rf);
            rightRear.setPower(rr);

            sleep(5000);
            // Stop motors
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            telemetry.addData("Distance:", traveledDistance);
            telemetry.addData("heading:", heading);
            telemetry.update();
            sleep(10000);

            // Stop motors
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
    }
}
