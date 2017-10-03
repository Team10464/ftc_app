package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by jonathonmangan on 10/1/17.
 */

public class MechanumProtoBot extends OpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private GyroSensor gyro;

    public void init(){
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft= hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
    }
    public void loop(){
        int heading = gyro.getHeading();

        if (gamepad1.left_trigger > .1 || gamepad1.right_trigger > .1){
            if (gamepad1.left_trigger > gamepad1.right_trigger){
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(-gamepad1.left_trigger);
                motorBackRight.setPower(gamepad1.left_trigger);
                motorBackLeft.setPower(0);
            } else {
                motorFrontRight.setPower(-gamepad1.right_trigger);
                motorFrontLeft.setPower(0);
                motorBackRight.setPower(0);
                motorBackLeft.setPower(gamepad1.right_trigger);
            }
        } else {
            double power = ((Math.abs(gamepad1.left_stick_y)+Math.abs(gamepad1.left_stick_x))/2);
            double head = (Math.PI * heading)/180;
            double headSticks = 1;



        }
    }
}
