package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;

/**
 * Created by ekbir on 11/8/2017.
 */
//@Autonomous(name="RedCorner", group="Red")
public class RedCorner extends AutonomousBase{


    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor conveyorHorz;
    private DcMotor conveyorVert;

    private Servo servo;

    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backRight");
        conveyorHorz = hardwareMap.dcMotor.get("conveyorHortz");
        conveyorVert = hardwareMap.dcMotor.get("conveyorVert");
        servo = hardwareMap.servo.get("servo");
    }


    double xTime;
    int i;
    VuforiaLocalizer vuforia;
    OpenGLMatrix lastLocation;


    public void gameState() {
        super.gameState();
        switch (gameState) {
            case 0: //Start
                if (actualRuntime() > 1) {
                    gameState = 1;
                    sTime = getRuntime();
                    map.setRobot(10, 8);
                    servo.setPosition(1);
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


                relicTrackables.activate();

                while (1 == 1) {
                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);


                    /**
                     * See if any of the instances of {@link relicTemplate} are currently visible.
                     * {@link RelicRecoveryVuMark} is an enum which can have the following values:
                     * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
                     * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}
                     * RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                     */

                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                        telemetry.addData("VuMark", "%s visible", vuMark);
                        telemetry.addData("null?", 1);
                    }
                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));
                    telemetry.addData("VuMark", "not visible");
                }
        }

        telemetry.update();
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark == RelicRecoveryVuMark.CENTER) {
            map.setGoal(11, 5);
            moveState = AutonomousBase.MoveState.STRAFE_TOWARDS_GOAL;
            conveyorVert.setTargetPosition(2);
            conveyorHorz.setTargetPosition(2);
        }

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            map.setGoal(11, 5.647);
            moveState = AutonomousBase.MoveState.STRAFE_TOWARDS_GOAL;
            conveyorVert.setTargetPosition(2);
            conveyorHorz.setTargetPosition(2);
        }

        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            map.setGoal(11, 4.397);
            moveState = AutonomousBase.MoveState.STRAFE_TOWARDS_GOAL;
            conveyorVert.setTargetPosition(2);
            conveyorHorz.setTargetPosition(2);
        }
    }
}

