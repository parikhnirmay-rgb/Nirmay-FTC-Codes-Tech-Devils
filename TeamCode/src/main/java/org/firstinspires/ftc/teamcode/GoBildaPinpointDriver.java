package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

// ==========================================
// 1. THE HARDWARE DRIVER (MUST BE PUBLIC)
// ==========================================
@I2cSensor(name = "goBILDA Pinpoint Odometry Computer", description = "goBILDA Pinpoint Odometry Computer", xmlTag = "GoBildaPinpointOdometryComputer")
public class GoBildaPinpointDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private final static I2cAddr DEFAULT_ADDRESS = I2cAddr.create7bit(0x31);

    public enum EncoderDirection { FORWARD, REVERSE }
    public enum ReadStatus { NOT_READY, READY }

    public enum GoBildaOdometryPods {
        goBILDA_4_BAR_POD(3.2808398950131233595800524934383);
        public final double countsPerMM;
        GoBildaOdometryPods(double countsPerMM) { this.countsPerMM = countsPerMM; }
    }

    private Pose2D position = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    private Pose2D velocity = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    private ReadStatus status = ReadStatus.NOT_READY;

    public GoBildaPinpointDriver(I2cDeviceSynch deviceSynch, boolean deviceClientIsOwned) {
        super(deviceSynch, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(DEFAULT_ADDRESS);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override protected boolean doInitialize() { return true; }
    @Override public HardwareDevice.Manufacturer getManufacturer() { return HardwareDevice.Manufacturer.Unknown; }
    @Override public String getDeviceName() { return "goBILDA Pinpoint Odometry Computer"; }

    public void setOffsets(double xOffset, double yOffset) {
        byte[] txData = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN)
                .putShort((short) Math.round(xOffset))
                .putShort((short) Math.round(yOffset)).array();
        for (int i = 0; i < txData.length; i++) {
            deviceClient.write8(0x10 + i, txData[i] & 0xFF);
        }
    }

    public void setEncoderResolution(GoBildaOdometryPods podType) { setEncoderResolution(podType.countsPerMM); }
    public void setEncoderResolution(double countsPerMM) {
        byte[] txData = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN)
                .putFloat((float) countsPerMM).array();
        for (int i = 0; i < txData.length; i++) {
            deviceClient.write8(0x14 + i, txData[i] & 0xFF);
        }
    }

    public void setEncoderDirections(EncoderDirection xDirection, EncoderDirection yDirection) {
        byte config = 0;
        if (xDirection == EncoderDirection.REVERSE) config |= 1;
        if (yDirection == EncoderDirection.REVERSE) config |= 2;
        deviceClient.write8(0x18, config);
    }

    public void resetPosAndIMU() { deviceClient.write8(0x19, 1); }
    public void recalibrateIMU() { deviceClient.write8(0x19, 2); }

    public void update() {
        byte[] readWindow = deviceClient.read(0x20, 26);
        ByteBuffer buffer = ByteBuffer.wrap(readWindow).order(ByteOrder.LITTLE_ENDIAN);
        status = (buffer.get() == 1) ? ReadStatus.READY : ReadStatus.NOT_READY;
        buffer.get(); // Padding
        double x = buffer.getInt() / 100.0;
        double y = buffer.getInt() / 100.0;
        double h = buffer.getShort() / 100.0;
        position = new Pose2D(DistanceUnit.MM, x, y, AngleUnit.DEGREES, h);
        double vx = buffer.getShort() / 10.0;
        double vy = buffer.getShort() / 10.0;
        double vh = buffer.getShort() / 10.0;
        velocity = new Pose2D(DistanceUnit.MM, vx, vy, AngleUnit.DEGREES, vh);
    }

    public Pose2D getPosition() { return position; }
    public Pose2D getVelocity() { return velocity; }
    public ReadStatus getStatus() { return status; }
}

// ==========================================
// 2. THE TELEMETRY OPMODE (NOT PUBLIC)
// ==========================================
@TeleOp(name = "Pinpoint Telemetry Test", group = "Calibration")
class PinpointTelemetryTest extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        // Looks for the configured hardware name "pinpoint"
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configuration settings
        pinpoint.setOffsets(0.0, 0.0);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        telemetry.addData("Status", "Calibrating IMU... Keep robot still!");
        telemetry.update();
        pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update(); // Fetch fresh hardware registers

            Pose2D currentPos = pinpoint.getPosition();
            Pose2D currentVel = pinpoint.getVelocity();

            telemetry.addLine("=== SYSTEM STATUS ===");
            telemetry.addData("Device State", pinpoint.getStatus());

            telemetry.addLine("\n=== ESTIMATED POSITION ===");
            telemetry.addData("X (mm)", "%.2f", currentPos.getX(DistanceUnit.MM));
            telemetry.addData("Y (mm)", "%.2f", currentPos.getY(DistanceUnit.MM));
            telemetry.addData("Heading (Deg)", "%.2f°", currentPos.getHeading(AngleUnit.DEGREES));

            telemetry.addLine("\n=== LIVE VELOCITY ===");
            telemetry.addData("X Vel (mm/s)", "%.2f", currentVel.getX(DistanceUnit.MM));
            telemetry.addData("Y Vel (mm/s)", "%.2f", currentVel.getY(DistanceUnit.MM));

            telemetry.update();
        }
    }
}
