package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by emilydkiehl on 10/1/17.
 */
@TeleOp(name="Oriented Protobot Tank", group="Protobot")
public class OrientedProtoBot extends OpMode {

    // State used for updating telemetry
    Orientation angles;

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor top;
    private DcMotor front;
    private Servo franny = null;
    private Servo mobert = null;

    private BNO055IMU imu;

    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backRight");
        franny = hardwareMap.servo.get("franny");
        mobert = hardwareMap.servo.get("mobert");
        top = hardwareMap.dcMotor.get("top");
        front = hardwareMap.dcMotor.get("front");


        BNO055IMU imu;
    }

    public void loop() {
        double r = Math.hypot(gamepad1.right_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_x, gamepad1.left_stick_y) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        motorFrontRight.setPower(v1);
        motorFrontLeft.setPower(v2);
        motorBackRight.setPower(v3);
        motorBackLeft.setPower(v4);

        if (gamepad2.b) {
            mobert.setPosition(.998);
            franny.setPosition(.002);
        } else if (gamepad2.a) {
            mobert.setPosition(.45);
            franny.setPosition(.78);
        }else if (gamepad2.y) {
            mobert.setPosition(.6);
            franny.setPosition(.6);
        } else if (gamepad2.x) {
            mobert.setPosition(.002);
            franny.setPosition(.998);
        } else {
        }


        if (gamepad1.left_stick_button) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES);
            final double v5 = r * Math.cos(robotAngle) + rightX + angles.firstAngle;
            final double v6 = r * Math.sin(robotAngle) + rightX + angles.firstAngle;
            final double v7 = r * Math.sin(robotAngle) + rightX + angles.firstAngle;
            final double v8 = r * Math.cos(robotAngle) + rightX + angles.firstAngle;

            motorFrontRight.setPower(v5);
            motorFrontLeft.setPower(v6);
            motorBackRight.setPower(v7);
            motorBackLeft.setPower(v8);
        }
           // top.setPower(gamepad2.right_stick_x * .5);
           // front.setPower(gamepad2.left_stick_x * .5);

            if (gamepad2.dpad_right) {
                top.setPower(-0.45);
            } else if (gamepad2.dpad_left) {
                top.setPower(0.45);
            } else {
                top.setPower(0);
            }

            if (gamepad2.dpad_up) {
                front.setPower(-0.7);
            } else if (gamepad2.dpad_down) {
                front.setPower(0.7);
            } else {
                front.setPower(0);
            }
        }
    }





