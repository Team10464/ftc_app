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
 * Created by Sean Ovens on 10/26/2016.
 */




@Autonomous(name="Blue Button 6", group="Blue")
public class AutonomousMeurkurie extends AutonomousBase {


    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private Servo servo;
    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backRight");
        servo = hardwareMap.servo.get("port1");
    }


    double xTime;
    int i;




        public void gameState()
        {
            super.gameState();
            switch (gameState) {
                case 0: //Start
                    if (actualRuntime() > 1 && !gyro.isCalibrating()) {
                        gameState = 1;
                        sTime = getRuntime();
                        map.setRobot(10, 2);
                        servo.setPosition(1);
                    }
                    break;
                case 1:
                    sensorColor = hardwareMap.colorSensor.get("color");
                    if (sensorColor.red() > sensorColor.blue())
                        motorBackLeft.setTargetPosition(2);
                        motorBackLeft.setPower(.6);
                        motorFrontRight.setTargetPosition(2);
                        motorFrontRight.setPower(-.6);

                    if (sensorColor.red() < sensorColor.blue())
                        motorBackLeft.setTargetPosition(2);
                        motorBackLeft.setPower(-.6);
                        motorFrontRight.setTargetPosition(2);
                        motorFrontRight.setPower(.6);

                    gameState = 3;
                    break;
                case 3: //Read cryptograph thingy
                    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                    //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId

                    // OR...  Do Not Activate the Camera Monitor View, to save power
                    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
                    parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;


                    parameters.vuforiaLicenseKey = "AXNl2OH/////AAAAGflnH9+TB0cjkiKrzrC40+hq56YtjwvBhyRcjrjKaCjm/UzvB4u1IBT/k5RKhsiiJwoIM4OlMjVVz/xrXIjupQWV7AmH0iUw7iwiE01IwTH1w8xbxgdS/dzzISlVAnAfzqDaAnskEyajrlWhM2OZTuxJ/FeWTIz69IFgk2ArC0ZlbbaUF8g0tbLwvNRjewbIebp81rksnL1KL2s/f8eiq9nb1P6KHMdjGXz7Q2opydkT74X7SJO4GSVmBVrDOumW9DXdOuk82JRPf8HjVjToTQu/zwPLjMr5izEmcs58bb5x2UiPFqzCsmrF65SYCQTShUizvlFKQfdMeu90OPRA0VswWUkvW6Y4OZu35P2+vdUV";


                    parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                    this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

                    /**
                     * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
                     * in this data set: all three of the VuMarks in the game were created from this one template,
                     * but differ in their instance id information.
                     * @see VuMarkInstanceId
                     */
                    VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
                    VuforiaTrackable relicTemplate = relicTrackables.get(0);
                    relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

                    telemetry.addData(">", "Press Play to start");
                    telemetry.update();
                    waitForStart();

                    relicTrackables.activate();

                    while (opModeIsActive()) {

                        /**
                         * See if any of the instances of {@link relicTemplate} are currently visible.
                         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
                         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
                         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
                         */
                        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                            telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                            telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                            if (pose != null) {
                                VectorF trans = pose.getTranslation();
                                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                                double tX = trans.get(0);
                                double tY = trans.get(1);
                                double tZ = trans.get(2);

                                // Extract the rotational components of the target relative to the robot
                                double rX = rot.firstAngle;
                                double rY = rot.secondAngle;
                                double rZ = rot.thirdAngle;
                            }
                        }
                        else {
                            telemetry.addData("VuMark", "not visible");
                        }

                        telemetry.update();
                    }
            }

            String format(OpenGLMatrix transformationMatrix) {
            return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
        }
        }



                    break;
                case 4: //Move parallel to wall
                    map.setGoal(map.getRobotX(), 0);
                    if (linedUp()) {
                        moveState = MoveState.STOP;
                        moveState = MoveState.SERVO_M;
                        moveState = MoveState.SERVO_DEPLOY;
                        gameState = 5;
                    } else {
                        moveState = MoveState.TURN_TOWARDS_GOAL;
                    }
                    break;
                case 5: //Move to wall
                    map.setGoal(12, 6);
                    if (!touchWall.isPressed()) {
                        moveState = MoveState.RIGHT;
                        gameState = 6;
                    } else {
                        moveState = MoveState.STOP;
                        moveState = MoveState.SERVO_DEPLOY;
                    }
                    break;
                case 6: //back up and button press A
                    map.setGoal(11.5, 0); // I need the goal far away so moveState keeps going
                    if (!touchWall.isPressed()) {
                        moveState = MoveState.RIGHT_SLOW;
                        xTime = 0;
                    } else {
                        if (touchRight.isPressed()) {
                            if (xTime == 0) {
                                xTime = getRuntime();
                                moveState = MoveState.STOP;
                            } else if (getRuntime() - xTime > 1) {
                                if (colorRight.blue() > colorRight.red() && colorLeft.blue() < colorLeft.red()) {
                                    moveState = MoveState.SERVO_L;
                                    gameState = 7;
                                    pTime = getRuntime();
                                } else if (colorRight.blue() < colorRight.red() && colorLeft.blue() > colorLeft.red()) {
                                    moveState = MoveState.SERVO_R;
                                    gameState = 7;
                                    pTime = getRuntime();
                                }
                            }
                        } else {
                            if (linedUp()) {
                                moveState = MoveState.BACKWARD_SLOW;
                            } else {
                                moveState = MoveState.TURN_TOWARDS_GOAL;
                            }
                        }
                    }
                    break;
            }
        }
    }
}
