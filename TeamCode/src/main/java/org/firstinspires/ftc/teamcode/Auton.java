package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;


/**
 * Created by joannareese on 10/20/17.
 */

public class Auton {


    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

    }

    public void loop() {

        motorFrontRight.setPower(1);
        motorFrontLeft.setPower(1);
        motorBackLeft.setPower(1);
        motorBackRight.setPower(1);
    }

}





