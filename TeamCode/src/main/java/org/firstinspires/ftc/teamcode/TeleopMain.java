package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by frown on 5/1/2017.
 */

@TeleOp(name="Main Teleop", group="Demo Chassis")
public class TeleopMain extends OpMode {
    private DcMotor leftDrive, rightDrive;
    private DcMotor collector;
    private DcMotor lift;

    private Servo separator;
    private Servo binLeft, binRight;

    private double right_grab = 0.0;
    private double right_hold = 0.5;
    private double right_release = 1.0;
    private double left_grab = 1.0;
    private double left_hold = 0.5;
    private double left_release = 0.0;
    private double sorter_idle = 0.5;
    private double sorter_left = 0.0;
    private double sorter_right = 1.0;

    private double sorter_timer;

    private enum SORTER_STATE{
        IDLE,
        RIGHT_SORT_A,
        RIGHT_SORT_B,
        RIGHT_HOLD,
        LEFT_SORT_A,
        LEFT_SORT_B,
        LEFT_HOLD;
    }

    private double sorter_idle_time = 0.4;
    private double sorter_pause_time = 0.4;

    private SORTER_STATE current_sorter_state = SORTER_STATE.IDLE;

    private double currentPositionLeftBin = 0.0;
    private double currentPositionRightBin = 0.0;
    private double currentPositionSeparator = 0.0;

    public void init(){
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collector = hardwareMap.dcMotor.get("collector");

        lift = hardwareMap.dcMotor.get("lift");

        separator = hardwareMap.servo.get("separator");
        binLeft = hardwareMap.servo.get("bin_left");
        binRight = hardwareMap.servo.get("bin_right");

        separator.setPosition(sorter_idle);
        binLeft.setPosition(left_grab);
        binRight.setPosition(right_grab);
    }

    public void loop(){
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickY = -gamepad1.right_stick_y;

        leftDrive.setPower(leftStickY+gamepad1.right_stick_x);
        rightDrive.setPower(leftStickY-gamepad1.right_stick_x);

        telemetry.addData("Encoder Left", leftDrive.getCurrentPosition());
        telemetry.addData("Encoder Right", rightDrive.getCurrentPosition());

        if(gamepad1.right_bumper){
            collector.setPower(1);
        } else if(gamepad1.right_trigger > 0){
            collector.setPower(-1);
        } else {
            collector.setPower(0);
        }

        if(gamepad1.left_trigger > 0){
            lift.setPower(-1);
        } else if(gamepad1.left_bumper){
            lift.setPower(1);
        } else {
            lift.setPower(0);
        }

        telemetry.addData("Current state: ", current_sorter_state);

        switch (current_sorter_state){
            case IDLE:{
                if(gamepad1.dpad_left){
                    binLeft.setPosition(left_release);
                } else {
                    binLeft.setPosition(left_hold);
                }
                if(gamepad1.dpad_right){
                    binRight.setPosition(right_release);
                } else {
                    binRight.setPosition(right_hold);
                }
                if(gamepad1.b){
                    current_sorter_state = SORTER_STATE.RIGHT_SORT_A;
                    binRight.setPosition(right_grab);
                    separator.setPosition(sorter_right);
                    sorter_timer = getRuntime();
                } else if(gamepad1.x){
                    binLeft.setPosition(left_grab);
                    separator.setPosition(sorter_left);
                    current_sorter_state = SORTER_STATE.LEFT_SORT_A;
                    sorter_timer = getRuntime();
                }
                break;
            }
            case RIGHT_SORT_A:{
                if(getRuntime()-sorter_timer > sorter_pause_time) {
                    current_sorter_state = SORTER_STATE.RIGHT_SORT_B;
                }
                break;
            }
            case RIGHT_SORT_B:{
                if(!gamepad1.b){
                    binRight.setPosition(right_hold);
                    current_sorter_state = SORTER_STATE.RIGHT_HOLD;
                    sorter_timer = getRuntime();
                }
                break;
            }
            case RIGHT_HOLD:{
                if(getRuntime()-sorter_timer > sorter_idle_time){
                    current_sorter_state = SORTER_STATE.IDLE;
                    separator.setPosition(sorter_idle);
                }
                break;
            }
            case LEFT_SORT_A:{
                if(getRuntime()-sorter_timer > sorter_pause_time) {
                    current_sorter_state = SORTER_STATE.LEFT_SORT_B;
                }
                break;
            }
            case LEFT_SORT_B:{
                if(!gamepad1.x){
                    binLeft.setPosition(left_hold);
                    current_sorter_state = SORTER_STATE.LEFT_HOLD;
                    sorter_timer = getRuntime();
                }
                break;
            }
            case LEFT_HOLD:{
                if(getRuntime()-sorter_timer > sorter_idle_time){
                    current_sorter_state = SORTER_STATE.IDLE;
                    separator.setPosition(sorter_idle);
                }
                break;
            }
        }

        /*if(gamepad2.dpad_left){
            currentPositionLeftBin -= 0.05;
            sleep(200);
        }
        if(gamepad2.dpad_right){
            currentPositionLeftBin += 0.05;
            sleep(200);
        }

        if(gamepad2.x){
            currentPositionRightBin -= 0.05;
            sleep(200);
        }
        if(gamepad2.b){
            currentPositionRightBin += 0.05;
            sleep(200);
        }

        if(gamepad2.left_bumper){
            currentPositionSeparator -= 0.05;
            sleep(200);
        }
        if(gamepad2.right_bumper){
            currentPositionSeparator += 0.05;
            sleep(200);
        }*/

        telemetry.addData("Separator", currentPositionSeparator);
        telemetry.addData("Left bin", currentPositionLeftBin);
        telemetry.addData("Right bin", currentPositionRightBin);

        //binLeft.setPosition(currentPositionLeftBin);
        //binRight.setPosition(currentPositionRightBin);
        //separator.setPosition(currentPositionSeparator);

    }

    void sleep(long t){

        try {
            Thread.sleep(t, 0);
        } catch(InterruptedException e){

        }
    }
}
