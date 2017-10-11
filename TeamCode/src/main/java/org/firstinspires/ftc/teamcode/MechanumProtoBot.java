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
    private DcMotor motorGlyphLift;
    private DcMotor motorTopConveyor;
    private DcMotor motorFourBar;
    private DcMotor motorRelicExtension;

    private GyroSensor gyro;

    public void init(){
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft= hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorGlyphLift = hardwareMap.dcMotor.get("lift");
        motorTopConveyor = hardwareMap.dcMotor.get("conveyor");
        //motorRelicExtensionAngler = hardwareMap.dcMotor.get("angle");
        motorRelicExtension = hardwareMap.dcMotor.get ("extension");

        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
    }
    public void loop(){
        int heading = gyro.getHeading();



        if(gamepad1.left_stick_button){
            gyro.calibrate();
        }

        if (gamepad2.dpad_up){
            motorGlyphLift.setPower(1);
        } else if (gamepad2.dpad_down){
            motorGlyphLift.setPower(-1);
        } else {
            motorGlyphLift.setPower(0);
        }

        if(gamepad2.dpad_right){
            motorTopConveyor.setPower(1);
        } else if (gamepad2.dpad_down){
            motorTopConveyor.setPower(-1);
        } else {
            motorTopConveyor.setPower(0);
        }

        /*if(gamepad2.left_stick_y){
            motorRelicExtension.setPower(gamepad2.left_stick_y);
        } else if (gamepad2.left_stick_y){
            motorRelicExtension.setPower(gamepad2.left_stick_y);
        } else {
            motorRelicExtension.setPower(gamepad2.left_stick_y);
        }

        if(gamepad2.a){
            motorFourBar.setPower();
        } else if (gamepad2.b)






            }*/






    }
}

