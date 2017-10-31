package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.


@Autonomous(name="AutonomousMeukurie", group="ProtoBot")
//@Disabled
public abstract class AutonomousMeurkurie extends OpMode {
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;


    public void init() {

        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backRight");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "imu";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");


                imu.initialize(parameters);
    }





    public void runOpMode()
    {

        //waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while (Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) < 90) { //imu.getAngularOrientation().firstAngle
            motorFrontLeft.setPower(0.6);
            motorBackLeft.setPower(0.6);
            motorFrontRight.setPower(-0.6);
            motorBackRight.setPower(-0.6);
        }

            telemetry.update();
        }

    public void composeIMUTelemetry() {

// At the beginning of each telemetry update, grab a bunch of data
// from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
// Acquiring the angles is relatively expensive; we don't want
// to do that in each of the three items that need that info, as that's
// three times the necessary expense.
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });
    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit , angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}




