
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "My First Autonomous")
public class Autonomous2Sec extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor leftFront =
                hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront =
                hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftRear =
                hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear =
                hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Ready for Autonomous!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Drive forward at 20% power
        leftFront.setPower(0.16);
        leftRear.setPower(0.15);
        rightFront.setPower(0.26);
        rightRear.setPower(0.27);

        sleep(5000); // Drive for 2 seconds

        // Stop
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}
