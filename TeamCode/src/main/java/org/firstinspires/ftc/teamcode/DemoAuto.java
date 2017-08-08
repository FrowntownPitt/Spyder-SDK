package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by frown on 5/1/2017.
 */

@Autonomous(name="Demo Auto", group="Demo Chassis")
public class DemoAuto extends OpMode {

    private DcMotor leftDrive, rightDrive;


    public void init(){
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int leftCount = leftDrive.getCurrentPosition();
        int rightCount = rightDrive.getCurrentPosition();

        telemetry.addData("Encoder left", leftCount);
        telemetry.addData("Encoder right", rightCount);
    }

    public void loop(){

    }
}
