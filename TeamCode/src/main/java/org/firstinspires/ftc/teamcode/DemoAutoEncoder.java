package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by frown on 5/1/2017.
 */

@Autonomous(name="Demo Teleop Encoder", group="Demo Chassis")
public class DemoAutoEncoder extends OpMode {

    private DcMotor leftDrive, rightDrive;

    private BNO055IMU imu;

    private Orientation imuoffset = new Orientation();

    private enum turnState{POS, NEG, STOP}
    private enum turnSpeed{HIGH, NORM, LOW}

    private turnState currentTurnState = turnState.STOP;
    private turnSpeed currentTurnSpeed = turnSpeed.HIGH;

    private float targetAngle;
    private float turnHighTolerance = 10.f;
    private float turnTolerance = 5.f;
    private float turnSlowerTolerance = 0.3f;
    private float currentTolerance;

    private int TurnAngleCheckCounter = 0;
    private int TurnAngleChecks = 5;

    private float turnSpeedHigh = 0.6f;
    private float turnSpeedNormal = 0.25f;
    private float turnSpeedSlow = 0.15f;
    private float currentSpeed;
    private boolean isTurning;

    private float currentTargetAngle = 0f;

    private enum AutoState{START,
        F1s, F1,
        T1s, T1,
        F2s, F2,
        T2s, T2,
        DONE {
            @Override
            public AutoState next() {
                return this;
            }
        };

        public AutoState next() {
            return values()[ordinal() + 1];
        }
    }
    private AutoState currentAutoState = AutoState.START;

    /* ---------------------------------------------- */

    private final double MMTOINCH = 0.0393701;
    private final double WHEELDIAMETER = 90; // in mm
    private final int ENCODER_TICKS_PER_REVOLUTION = 1440;
    private final double PI = 3.1415926535;

    private float maxCorrectionOffset = 0.3f;
    private float driveCorrectionPrecision = 2f;
    private float currentDriveCorrectionPrecision = driveCorrectionPrecision;

    private float startAngle;
    private int currentTicks; // in encoder ticks
    private int targetTicks;

    private int startTicks;

    /////////////////////////////////////////////////////

    public void init(){
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
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
        float imuZ = -(angles.thirdAngle - imuoffset.thirdAngle);
        if(imuZ < 0){
            imuZ += 360;
        }
        telemetry.addData("IMU Angle X", angles.firstAngle-imuoffset.firstAngle);
        telemetry.addData("IMU Angle Y", angles.secondAngle-imuoffset.secondAngle);
        telemetry.addData("IMU Angle Z", imuZ);
        telemetry.addData("Current Target Angle: ", currentTargetAngle);
        telemetry.addData("Current Drive Correction Precision", currentDriveCorrectionPrecision);

        if(gamepad1.x){
            currentAutoState = AutoState.T1s;
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

        leftDrive.setPower((-gamepad1.left_stick_y) + gamepad1.right_stick_x);
        rightDrive.setPower((-gamepad1.left_stick_y) - gamepad1.right_stick_x);


        if(gamepad1.y){
            currentAutoState = AutoState.F1s;
        }
        if(gamepad1.left_trigger > 0){
            currentDriveCorrectionPrecision += 1f;
        }
        if(gamepad1.right_trigger > 0){
            currentDriveCorrectionPrecision -= 1f;
            if(currentDriveCorrectionPrecision < 0){
                currentDriveCorrectionPrecision = 0;
            }
        }

        if(gamepad1.b){
            currentAutoState = AutoState.DONE;
        }

        switch(currentAutoState) {
            case F1s: {
                telemetry.addData("Forward!", "Initializing");
                telemetry.update();
                startAngle = imuZ;

                startTicks = leftDrive.getCurrentPosition();
                targetTicks = startTicks + (int)((10.f / (PI * WHEELDIAMETER * MMTOINCH)) * ENCODER_TICKS_PER_REVOLUTION);

                currentAutoState = currentAutoState.next();
                break;
            }
            case F1: {
                telemetry.addData("Forward!", "Moving");
                telemetry.addData("Current Ticks: ", leftDrive.getCurrentPosition());
                telemetry.addData("Target Ticks: ", targetTicks);
                if(DriveToDistance(targetTicks, leftDrive.getCurrentPosition(), 0.6, (imuZ-startAngle)/currentDriveCorrectionPrecision)){
                    currentAutoState = AutoState.DONE;
                }
                telemetry.addData("Left Motor speed:  ", leftDrive.getPower());
                telemetry.addData("Right Motor speed: ", rightDrive.getPower());
                telemetry.update();
                break;
            }
            case T1s: {
                currentTolerance = turnHighTolerance;
                currentSpeed = turnSpeedHigh;
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

    private boolean DriveToDistance(int targetTicks, int currentTicks, double speed, float orientation){

        if(orientation > 180){
            orientation -= 360;
        }
        leftDrive.setPower(speed);
        if(Math.abs(orientation) > maxCorrectionOffset)
            rightDrive.setPower(speed + maxCorrectionOffset * (orientation/Math.abs(orientation)));
        else
            rightDrive.setPower(speed + orientation);

        if(rightDrive.getPower() > 1){
            rightDrive.setPower(1);
        }
        if(rightDrive.getPower() < -1){
            rightDrive.setPower(-1);
        }

        //telemetry.addData("Left Motor speed:  ", speed);
        //telemetry.addData("Right Motor speed: ", speed + orientation);
        //telemetry.update();

        if(currentTicks > targetTicks){
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            return true;
        }

        return false;
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

            imuZ = ((int)(imuZ*10))/10.f;

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
                    //if (TurnAngleCheckCounter++ > TurnAngleChecks)
                    //    return true;
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    return true;
                }
            }
        //} else {
        //    return true;
        //}
        return false;
    }
}
