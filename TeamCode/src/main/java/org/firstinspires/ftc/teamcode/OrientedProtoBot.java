package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by Team 10464 on 9/21/16.
 */
@TeleOp(name="Oriented Protobot Tank", group="Protobot")

public class OrientedProtoBot extends OpMode {

    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private double resetTime;
    private int cDist, lDist, dDist, tDist;


    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    public void init(){
        motorFrontRight = hardwareMap.dcMotor.get("front");
        motorBackLeft = hardwareMap.dcMotor.get("back");
        motorBackRight = hardwareMap.dcMotor.get("left");
        motorFrontLeft = hardwareMap.dcMotor.get("right");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
// and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }
    public void loop()
    {

        lDist = cDist;
        dDist = cDist - lDist;
        tDist += dDist;
        // motorspeed = dx/dt * (60 seconds/1 minute) * (1 rotation/1120 encoder degrees) = (rotations/minute)
        double motorSpeed = 60*tDist/(getRuntime() - resetTime)/1120;

        // Drivetrain controls
        if(gamepad1.left_trigger > .1 || gamepad1.right_trigger > .1)
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

        }

        if(gamepad1.left_stick_button){
        //    imu.writeCalibrationData(BNO055IMU.CalibrationData);
        }

        // Put telemetry here
        telemetry.addData("motor speed", motorSpeed);
        telemetry.addData("theta", imu.getAngularOrientation());
        telemetry.addData("motor 1", motorFrontRight.getCurrentPosition());
        telemetry.addData("motor 2", motorFrontLeft.getCurrentPosition());
        telemetry.addData("motor 3", motorBackRight.getCurrentPosition());
        telemetry.addData("motor 4", motorFrontLeft.getCurrentPosition());
    }
}
