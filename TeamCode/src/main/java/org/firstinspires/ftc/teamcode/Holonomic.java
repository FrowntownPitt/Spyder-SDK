package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by frown on 6/30/2017.
 */

@TeleOp(name="Spyder", group="Spyder")
public class Holonomic extends OpMode {

    BNO055IMU imu;

    private DcMotor motorFront;
    private DcMotor motorRear;
    private DcMotor motorLeft;
    private DcMotor motorRight;

    float offsetX = 0;
    float offsetY = 0;
    float offsetZ = 0;

    public void init(){

        motorFront = hardwareMap.dcMotor.get("frontMotor");
        motorRear = hardwareMap.dcMotor.get("rearMotor");
        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        motorRight = hardwareMap.dcMotor.get("rightMotor");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFront.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters imuparams = new BNO055IMU.Parameters();
        imuparams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuparams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuparams.calibrationDataFile = "BNO055IMUCalibration.json";
        imuparams.loggingEnabled = true;
        imuparams.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuparams);

    }

    public void start(){

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        offsetX = angles.firstAngle;
        offsetY = angles.secondAngle;
        offsetZ = angles.thirdAngle;
    }

    public void loop(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("Offset X", offsetX);
        telemetry.addData("Offset Y", offsetY);
        telemetry.addData("Offset Z", offsetZ);
        telemetry.addData("Heading ", angles.firstAngle);
        telemetry.addData("Roll    ", angles.secondAngle);
        telemetry.addData("Pitch   ", angles.thirdAngle);

        double leftX = gamepad1.left_stick_x;
        double leftY = -gamepad1.left_stick_y;

        double rightX = gamepad1.right_stick_x;

        double r = Math.hypot(leftX, leftY);

        double targetAngle = Math.atan2(Math.toRadians(leftY), Math.toRadians(leftX));// - (angles.firstAngle);

        double leftPower  = r * Math.sin(targetAngle) + rightX;
        double rightPower = r * Math.sin(targetAngle) - rightX;
        double frontPower = r * Math.cos(targetAngle) + rightX;
        double rearPower  = r * Math.cos(targetAngle) - rightX;

        telemetry.addData("Target angle", targetAngle);
        telemetry.addData("Left  ", leftPower);
        telemetry.addData("Right ", rightPower);
        telemetry.addData("Front ", frontPower);
        telemetry.addData("Rear ", rearPower);

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
        motorFront.setPower(frontPower);
        motorRear.setPower(rearPower);

        telemetry.update();
    }
}
