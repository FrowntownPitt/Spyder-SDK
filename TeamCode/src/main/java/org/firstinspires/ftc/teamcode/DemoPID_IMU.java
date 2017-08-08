package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by frown on 5/1/2017.
 */

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Demo PID - IMU", group="Demo Chassis")
public class DemoPID_IMU extends OpMode {
    private DcMotor leftDrive, rightDrive;

    private BNO055IMU imu;

    private ArrayList<ArrayList<Double>> ErrorList;

    private Orientation imuoffset = new Orientation();

    public void init(){
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters imuparams = new BNO055IMU.Parameters();
        imuparams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuparams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuparams.calibrationDataFile = "BNO055IMUCalibration.json";
        imuparams.loggingEnabled = true;
        imuparams.loggingTag = "IMU";
        //imuparams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuparams);

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        telemetry.addData("IMU Angle X", angles.firstAngle);
        telemetry.addData("IMU Angle Y", angles.secondAngle);
        telemetry.addData("IMU Angle Z", angles.thirdAngle);

        telemetry.update();

        ErrorList = new ArrayList<>();
    }

    public void start(){

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        imuoffset.firstAngle = angles.firstAngle;
        imuoffset.secondAngle = angles.thirdAngle;
        imuoffset.thirdAngle = angles.thirdAngle;

        lastTime = getRuntime();
    }

    private double targetAngle = 90;
    private double maxOutput = 1.0;

    private boolean isHoming = false;

    private double kP = 0.10;
    private double kI = 0.0;
    private double kD = 0.0;

    private double integralTerm = 0.0;
    private double lastError = 0.0;

    private double maxIntegral = 10;

    private double PID_Step = 0.0001;
    private double PID_Step_Step = 0.00005;

    private double lastTime;

    int outputCount = 0;

    public void loop(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        float imuZ = -(angles.thirdAngle - imuoffset.thirdAngle);
        //if(imuZ < 0){
        //    imuZ += 360;
        //}

        telemetry.addData("IMU Angle X", angles.firstAngle-imuoffset.firstAngle);
        telemetry.addData("IMU Angle Y", angles.secondAngle-imuoffset.secondAngle);
        telemetry.addData("IMU Angle Z", imuZ);
        telemetry.addData("Current Target Angle: ", targetAngle);

        telemetry.addData("Current P", kP);
        telemetry.addData("Current I", kI);
        telemetry.addData("Current D", kD);
        telemetry.addData("Current Step", PID_Step);

        if(gamepad1.x){
            isHoming = true;
            integralTerm = 0.0;
            ErrorList = new ArrayList<>();
        } else if(gamepad1.b){
            isHoming = false;
        }

        if(gamepad1.right_bumper){
            kP += PID_Step;
        } else if(gamepad1.right_trigger > 0){
            kP -= PID_Step;
        }
        if(gamepad1.left_bumper){
            kD += PID_Step;
        } else if(gamepad1.left_trigger > 0){
            kD -= PID_Step;
        }
        if(gamepad1.dpad_up){
            kI += PID_Step;
        } else if(gamepad1.dpad_down){
            kI -= PID_Step;
        }

        if(gamepad1.dpad_left){
            PID_Step -= PID_Step_Step;
        } else if(gamepad1.dpad_right){
            PID_Step += PID_Step_Step;
        }

        if(gamepad1.y){
            outputCount++;
            writeToFile("Output"+outputCount+".csv", ErrorList);
        }

        if(isHoming){
            double Output;
            double error = targetAngle - imuZ;

            double currentTime = getRuntime();
            double deltaT = currentTime - lastTime;

            integralTerm += error * deltaT;


            if(kI * integralTerm > maxOutput){
                integralTerm = maxOutput/kI;
            } else if(kI * integralTerm < -maxOutput){
                integralTerm = -maxOutput/kI;
            }

            double dTerm = (error - lastError) / deltaT;


//            if(kP * error > maxOutput){
//                Output = maxOutput + kI * integralTerm + kD * dTerm;
//            } else if(kI * integralTerm < -maxOutput){
//                Output = -maxOutput + kI * integralTerm + kD * dTerm;
//            } else {
//                Output = kP * error + kI * integralTerm + kD * dTerm;
//            }
            Output = kP * error + kI * integralTerm + kD * dTerm;

            telemetry.addData("Error ", error);
            telemetry.addData("Output", Output);

            telemetry.addData("iTerm ", integralTerm);

            if((leftDrive.getPower() < 0 && Output > 0) || (leftDrive.getPower() > 0 && Output < 0)){
                integralTerm = 0;
            }

            leftDrive.setPower(Output);
            rightDrive.setPower(-Output);

            lastError = error;
            lastTime = getRuntime();

            if(gamepad1.left_stick_y > 0){
                maxIntegral += gamepad1.left_stick_y;
            } else if(gamepad1.left_stick_y < 0) {
                maxIntegral += gamepad1.left_stick_y;
            }

            if(integralTerm > maxIntegral){
                integralTerm = maxIntegral;
            } else if(integralTerm < -maxIntegral){
                integralTerm = -maxIntegral;
            }

            telemetry.addData("Max iTerm ", maxIntegral);

            ArrayList<Double> dataPoint = new ArrayList<>();
            dataPoint.add(getRuntime());
            dataPoint.add(error);
            dataPoint.add(Output);
            dataPoint.add(kP * error);
            dataPoint.add(kI * integralTerm);
            dataPoint.add(kD * dTerm);
            ErrorList.add(dataPoint);
        }

        if(!isHoming) {
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickY = -gamepad1.right_stick_y;

            leftDrive.setPower(leftStickY);
            rightDrive.setPower(rightStickY);

            telemetry.addData("Encoder Left", leftDrive.getCurrentPosition());
            telemetry.addData("Encoder Right", rightDrive.getCurrentPosition());
        }

        telemetry.update();
    }

    public void stop(){
        writeToFile("error.csv", ErrorList);
    }

    private void writeToFile(String filename, ArrayList<ArrayList<Double>> list){
        try{
            FileWriter out = new FileWriter(new File(Environment.getExternalStorageDirectory(), filename));
            Log.d("AOPMODE", "count: " + list.size());

            for(ArrayList<Double> l: list) {
                String line = "";

                for(Double d: l){
                    line += d + ",";
                }

                line += "\n";

                out.write(line);
                Log.d("AOPMODE", "Wrote: \"" + line + "\"");
            }

            out.flush();
            out.close();

            Log.d("AOpmode", "File written");

        } catch(IOException e){
            Log.e("OPMODE FILE ERROR", e.getMessage());
            throw new NullPointerException();
        }
    }
}
