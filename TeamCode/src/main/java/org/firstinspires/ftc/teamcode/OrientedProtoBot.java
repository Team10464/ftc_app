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
    private DcMotor conveyorHorz;
    private DcMotor conveyorVert;
    private Servo franny;
    private Servo mobert;

    private BNO055IMU imu;

    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backRight");
        franny = hardwareMap.servo.get("franny");
        mobert = hardwareMap.servo.get("mobert");
        //  conveyorHorz = hardwareMap.dcMotor.get("conveyorHortz");
        //  conveyorVert = hardwareMap.dcMotor.get("conveyorVert");


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


        if (gamepad2.left_bumper) {
            mobert.setPosition(0);
            franny.setPosition(1);

        } else if (gamepad2.right_bumper) {
            mobert.setPosition(1);
            franny.setPosition(0);
        } else
        {

        }
    }
}

//        if (gamepad2.x){
//            conveyorHorz.setPower(1);
//            }
//            else if (gamepad2.a)
//            {
//            conveyorHorz.setPower(-1);
//            }
//            else
//            {
//            conveyorHorz.setPower(0);
//            }

//        if (gamepad2.y)
//        {
//            conveyorVert.setPower(1);
//            }
//            else if (gamepad2.b)
//            {
//            conveyorVert.setPower(-1);
//            }
//            else
//            {
//            conveyorVert.setPower(0);
//            }
//        }
//    }
