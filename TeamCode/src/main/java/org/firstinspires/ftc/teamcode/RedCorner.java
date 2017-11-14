package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name="RedCorner", group="Red")
class RedCorner extends AutonomousBase {

    double xTime;
    int i;
    private OpenGLMatrix lastLocation;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor top;
    private DcMotor front;
    private Servo servo;
    private VuforiaLocalizer vuforia;

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
                if (actualRuntime() > 0) {
                    gameState = 1;
                    sTime = getRuntime();
                    map.setRobot(10,8);
                    servo.setPosition(.675);
                }
                break;

            case 1:
                sensorColor = hardwareMap.colorSensor.get("color");
                if (sensorColor.red() > sensorColor.blue()) {
                    motorBackLeft.setTargetPosition(1);
                    motorBackLeft.setPower(.6);
                    motorFrontRight.setTargetPosition(1);
                    motorFrontRight.setPower(-.6);
                }

                if (sensorColor.red() < sensorColor.blue()) {
                    motorBackLeft.setTargetPosition(1);
                    motorBackLeft.setPower(-.6);
                    motorFrontRight.setTargetPosition(1);
                    motorFrontRight.setPower(.6);
                }

                gameState = 3;
                break;

            case 3: //move to save zone
                map.setGoal(9,10.5);
                moveState = MoveState.STRAFE_TOWARDS_GOAL;


                if (map.distanceToGoal() > DISTANCE_TOLERANCE) {
                    front.setPower(.5);
                    top.setTargetPosition((int) .5);
                }
        }
    }
}