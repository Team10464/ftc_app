package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by jonathonmangan on 10/1/17.
 */
@TeleOp(name="Mechanum Protobot Tank", group="Protobot")

public class MechanumProtoBot extends OpMode
{

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private BNO055IMU imu;

    public void init()
    {
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft= hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.getCalibrationStatus();


    }
    public void loop()
    {
        //here is said error
        int reading = imu.readCalibrationData();

        double power = ((Math.abs(gamepad1.left_stick_y)+Math.abs(gamepad1.left_stick_x))/2);
        double head = (Math.PI * reading)/180;
        double headSticks = 2;

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        motorFrontLeft.setPower(v1);
        motorFrontRight.setPower(v2);
        motorBackLeft.setPower(v3);
        motorBackRight.setPower(v4);
        }
    }