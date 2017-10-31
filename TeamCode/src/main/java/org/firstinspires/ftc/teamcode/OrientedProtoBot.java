package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by Team 10464 on 9/21/16.
 */
@TeleOp(name = "Oriented Protobot", group = "Protobot")

class OrientedTeleop extends LinearOpMode
{

    /**
     * In this sample, for illustration purposes we use two interfaces on the one gyro object.
     * That's likely atypical: you'll probably use one or the other in any given situation,
     * depending on what you're trying to do. {@link IntegratingGyroscope} (and it's base interface,
     * {@link Gyroscope}) are common interfaces supported by possibly several different gyro
     * implementations. {@link ModernRoboticsI2cGyro}, by contrast, provides functionality that
     * is unique to the Modern Robotics gyro sensor.
     */
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        boolean lastResetState = false;
        boolean curResetState = false;

        // Get a reference to a Modern Robotics gyro object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
        // A similar approach will work for the Gyroscope interface, if that's all you need.

        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();
        // Wait until the gyro calibration is complete
        timer.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();
        telemetry.log().clear();
        telemetry.log().add("Press A & B to reset heading");

        // Loop until we're asked to stop
        while (opModeIsActive()) {

            // If the A and B buttons are pressed just now, reset Z heading.
            curResetState = (gamepad1.a && gamepad1.b);
            if (curResetState && !lastResetState) {
                modernRoboticsI2cGyro.resetZAxisIntegrator();
            }
            lastResetState = curResetState;

            // The raw() methods report the angular rate of change about each of the
            // three axes directly as reported by the underlying sensor IC.
            int rawX = modernRoboticsI2cGyro.rawX();
            int rawY = modernRoboticsI2cGyro.rawY();
            int rawZ = modernRoboticsI2cGyro.rawZ();
            int heading = modernRoboticsI2cGyro.getHeading();
            int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

            // Read dimensionalized data from the gyro. This gyro can report angular velocities
            // about all three axes. Additionally, it internally integrates the Z axis to
            // be able to report an absolute angular Z orientation.
            AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
            float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // Read administrative information from the gyro
            int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();
            int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();

            telemetry.addLine()
                    .addData("dx", formatRate(rates.xRotationRate))
                    .addData("dy", formatRate(rates.yRotationRate))
                    .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
            telemetry.addData("angle", "%s deg", formatFloat(zAngle));
            telemetry.addData("heading", "%3d deg", heading);
            telemetry.addData("integrated Z", "%3d", integratedZ);
            telemetry.addLine()
                    .addData("rawX", formatRaw(rawX))
                    .addData("rawY", formatRaw(rawY))
                    .addData("rawZ", formatRaw(rawZ));
            telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
            telemetry.update();



            lDist = cDist;
            dDist = cDist - lDist;
            tDist += dDist;
            // motorspeed = dx/dt * (60 seconds/1 minute) * (1 rotation/1120 encoder degrees) = (rotations/minute)
            double motorSpeed = 60 * tDist / (getRuntime() - resetTime) / 1120;

            // Drivetrain controls
            if (gamepad1.left_trigger > .1 || gamepad1.right_trigger > .1) {

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

            if (gamepad1.left_stick_button)
            {
                modernRoboticsI2cGyro.calibrate();

            }


        }
    }

    String formatRaw(int rawValue) {
        return String.format("%d", rawValue);
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatFloat(float rate) {
        return String.format("%.3f", rate);
    }


    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private double resetTime;
    private int cDist, lDist, dDist, tDist;


    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

        }
