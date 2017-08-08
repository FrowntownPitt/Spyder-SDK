package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;

/**
 * Created by frown on 5/1/2017.
 */

@TeleOp(name="Demo Teleop", group="Demo Chassis")
public class DemoTeleop extends OpMode {
    private DcMotor leftDrive, rightDrive;

    public void init(){
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop(){
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickY = -gamepad1.right_stick_y;

        leftDrive.setPower(leftStickY);
        rightDrive.setPower(rightStickY);

        telemetry.addData("Encoder Left", leftDrive.getCurrentPosition());
        telemetry.addData("Encoder Right", rightDrive.getCurrentPosition());
    }
}
