package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by emilydkiehl on 11/1/17
 */




@Autonomous(name="RedFront", group="Red")
public class RedFront extends AutonomousBase {


    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor top;
    private DcMotor front;

    private Servo servo;

    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backRight");
        top = hardwareMap.dcMotor.get("top");
        front = hardwareMap.dcMotor.get("front");
        servo = hardwareMap.servo.get("servo");
    }


    public void gameState() {
        super.gameState();
        switch (gameState) {
            case 0: //Start
                if (actualRuntime() > 1) {
                    gameState = 1;
                    sTime = getRuntime();
                    map.setRobot(10, 2);
                    servo.setPosition(.675);
                }
                break;
            case 1:
                sensorColor = hardwareMap.colorSensor.get("color");
                if (sensorColor.red() > sensorColor.blue()) {
                    motorBackLeft.setTargetPosition(2);
                    motorBackLeft.setPower(.6);
                    motorFrontRight.setTargetPosition(2);
                    motorFrontRight.setPower(-.6);
                }

                if (sensorColor.red() < sensorColor.blue()) {
                    motorBackLeft.setTargetPosition(2);
                    motorBackLeft.setPower(-.6);
                    motorFrontRight.setTargetPosition(2);
                    motorFrontRight.setPower(.6);
                }

                gameState = 3;
                break;

            case 3:
                map.setGoal(10, 5);
                moveState = MoveState.STRAFE_TOWARDS_GOAL;

                break;

        }
    }
}


