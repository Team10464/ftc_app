package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor conveyorHorz;
    private DcMotor conveyorVert;
    private BNO055IMU imu;

    public void init()
    {
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft= hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backRight");
        conveyorHorz = hardwareMap.dcMotor.get("conveyorHortz");
        conveyorVert = hardwareMap.dcMotor.get("conveyorVert");


    }
    public void loop()
    {


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


        if (gamepad2.x){
            conveyorHorz.setPower(1);
        }else if (gamepad2.a){
            conveyorHorz.setPower(-1);
        }else{
            conveyorHorz.setPower(0);
        }

        if (gamepad2.y){
            conveyorVert.setPower(1);
        }else if (gamepad2.b){
            conveyorVert.setPower(-1);
        }else{
            conveyorVert.setPower(0);
        }
    }

}


