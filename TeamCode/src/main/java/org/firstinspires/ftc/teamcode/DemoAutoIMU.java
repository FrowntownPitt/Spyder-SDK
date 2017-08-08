package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by frown on 5/1/2017.
 */

@Autonomous(name="Demo Teleop IMU", group="Demo Chassis")
public class DemoAutoIMU extends OpMode {

    private DcMotor leftDrive, rightDrive;

    private BNO055IMU imu;

    private Orientation imuoffset = new Orientation();

    private enum turnState{POS, NEG, STOP}
    private enum turnSpeed{HIGH, NORM, LOW}

    private turnState currentTurnState = turnState.STOP;
    private turnSpeed currentTurnSpeed = turnSpeed.NORM;

    private float targetAngle;
    private float turnTolerance = 5.f;
    private float turnSlowerTolerance = 0.3f;
    private float currentTolerance;

    private int TurnAngleCheckCounter = 0;
    private int TurnAngleChecks = 5;

    private float turnSpeedNormal = 0.25f;
    private float turnSpeedSlow = 0.15f;
    private float currentSpeed;
    private boolean isTurning;

    private float currentTargetAngle = 0f;

    private enum AutoState{START, F1, T1, F2, T2, DONE}
    private AutoState currentAutoState = AutoState.START;

    public void init(){
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters imuparams = new BNO055IMU.Parameters();
        imuparams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuparams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuparams.calibrationDataFile = "BNO055IMUCalibration.json";
        imuparams.loggingEnabled = true;
        imuparams.loggingTag = "IMU";
        imuparams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuparams);

        //while(imu.isGyroCalibrated()){
        //    telemetry.addData("IMU init", "Calibrating");
        //}

        int leftCount = leftDrive.getCurrentPosition();
        int rightCount = rightDrive.getCurrentPosition();

        telemetry.addData("Encoder left", leftCount);
        telemetry.addData("Encoder right", rightCount);

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        telemetry.addData("IMU Angle X", angles.firstAngle);
        telemetry.addData("IMU Angle Y", angles.secondAngle);
        telemetry.addData("IMU Angle Z", angles.thirdAngle);


        telemetry.update();
    }

    public void start(){

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        imuoffset.firstAngle = angles.firstAngle;
        imuoffset.secondAngle = angles.thirdAngle;
        imuoffset.thirdAngle = angles.thirdAngle;
    }

    public void loop(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        float imuZ = (angles.thirdAngle - imuoffset.thirdAngle);
        if(imuZ < 0){
            imuZ += 360;
        }
        telemetry.addData("IMU Angle X", angles.firstAngle-imuoffset.firstAngle);
        telemetry.addData("IMU Angle Y", angles.secondAngle-imuoffset.secondAngle);
        telemetry.addData("IMU Angle Z", imuZ);
        telemetry.addData("Current Target Angle: ", currentTargetAngle);

        if(gamepad1.x){
            currentAutoState = AutoState.F1;
        }
        if(gamepad1.left_bumper){
            currentTargetAngle += 0.5f;
        }
        if(gamepad1.right_bumper){
            currentTargetAngle -= 0.5f;
        }
        if(currentTargetAngle < 0){
            currentTargetAngle += 360;
        }
        if(currentTargetAngle > 360){
            currentTargetAngle -= 360;
        }

        switch(currentAutoState) {
            case F1: {
                currentTolerance = turnTolerance;
                currentSpeed = turnSpeedNormal;
                TurnAngleCheckCounter = 0;
                currentAutoState = AutoState.T1;

                isTurning = true;
                break;
            }
            case T1: {
                if (!TurnToAngle(currentTargetAngle, imuZ)) {
                    telemetry.addData("T1", "Turning");
                } else {
                    telemetry.addData("T1", "Done!");
                    currentAutoState = AutoState.DONE;
                    break;
                }
                telemetry.update();
            }
        }

        telemetry.update();
    }

    private boolean TurnToAngle(float targetAngle, float imuZ){

        //if(isTurning) {
            if (currentTurnState == turnState.STOP) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            } else if (currentTurnState == turnState.NEG) {
                leftDrive.setPower(-currentSpeed);
                rightDrive.setPower(currentSpeed);
            } else if (currentTurnState == turnState.POS) {
                leftDrive.setPower(currentSpeed);
                rightDrive.setPower(-currentSpeed);
            }

            if (imuZ > targetAngle + currentTolerance && imuZ - 180 < targetAngle + turnTolerance) {
                currentTurnState = turnState.NEG;
            } else if (imuZ < targetAngle - currentTolerance || imuZ - 180 >= targetAngle + turnTolerance) {
                currentTurnState = turnState.POS;
            } else {
                currentTurnState = turnState.STOP;
                //isTurning = false;
                //leftDrive.setPower(0);
                //rightDrive.setPower(0);
                if (currentTurnSpeed == turnSpeed.HIGH) {
                    currentTurnSpeed = turnSpeed.NORM;
                    currentTolerance = turnTolerance;
                    currentSpeed = turnSpeedNormal;
                }
                if (currentTurnSpeed == turnSpeed.NORM) {
                    currentTurnSpeed = turnSpeed.LOW;
                    currentTolerance = turnSlowerTolerance;
                    currentSpeed = turnSpeedSlow;
                }
                if (currentTurnSpeed == turnSpeed.LOW) {
                    if (TurnAngleCheckCounter++ > TurnAngleChecks)
                        return true;
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                }
            }
        //} else {
        //    return true;
        //}
        return false;
    }
}
