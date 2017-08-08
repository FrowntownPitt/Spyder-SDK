package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by frown on 5/1/2017.
 */

@Disabled
@TeleOp(name="Demo Linear Teleop", group="Demo Chassis")
public class FirstOpMode extends LinearOpMode {
    private DcMotor leftMotor, rightMotor;



    private ElapsedTime period = new ElapsedTime();

    private void waitForTick(long periodMs) throws java.lang.InterruptedException {
        long remaining = periodMs - (long)period.milliseconds();

        if(remaining > 0){
            Thread.sleep(remaining);
        }

        period.reset();
    }

    public void runOpMode(){
        double left = 0.0;
        double right = 0.0;

        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        waitForStart();

        try{
            while(opModeIsActive()){
                left = -gamepad1.left_stick_y;
                right = -gamepad1.right_stick_y;
                leftMotor.setPower(left);
                rightMotor.setPower(right);

                waitForTick(40);
            }
        } catch(java.lang.InterruptedException e){
            return;
        } finally {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

    }

}
