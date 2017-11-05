package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by nemo on 10/26/2016.
 */


@Autonomous(name="Blue Button 6", group="Blue")
public class AutonomousMeurkurie extends AutonomousBase{
    double xTime;
    int i;
    @Override
    public void gameState() {
        super.gameState();
        switch(gameState){
            case 0: //Start
                if(actualRuntime() > 1 && !gyro.isCalibrating()) {
                    gameState = 1;
                    sTime = getRuntime();
                    map.setRobot(10,2);
                }
                break;
            case 1:
                sensorColor = hardwareMap.colorSensor.get("color");
                if(sensorColor.red() > sensorColor.blue())


                moveState = MoveState.SERVO_M;
                gameState = 3;
                break;
            case 3: //Move to beacon A push pos.
                map.setGoal(9, 7);
                moveState = MoveState.STRAFE_TOWARDS_GOAL;
                if(map.distanceToGoal()<=.1){
                    moveState = MoveState.STOP;
                    moveState = MoveState.SERVO_DEPLOY;
                    moveState = MoveState.SERVO_M;
                    xTime = getRuntime();
                    gameState = 4;
                }
                break;
            case 4: //Move parallel to wall
                map.setGoal(map.getRobotX(),0);
                if(linedUp()){
                    moveState = MoveState.STOP;
                    moveState = MoveState.SERVO_M;
                    moveState = MoveState.SERVO_DEPLOY;
                    gameState = 5;
                }
                else{
                    moveState = MoveState.TURN_TOWARDS_GOAL;
                }
                break;
            case 5: //Move to wall
                map.setGoal(12, 6);
                if(!touchWall.isPressed()){
                    moveState = MoveState.RIGHT;
                    gameState = 6;
                } else{
                    moveState = MoveState.STOP;
                    moveState = MoveState.SERVO_DEPLOY;
                }
                break;
            case 6: //back up and button press A
                map.setGoal(11.5,0); // I need the goal far away so moveState keeps going
                if(!touchWall.isPressed()){
                    moveState = MoveState.RIGHT_SLOW;
                    xTime = 0;
                }else {
                    if (touchRight.isPressed()) {
                        if(xTime == 0){
                            xTime = getRuntime();
                            moveState = MoveState.STOP;
                        }else if(getRuntime() - xTime > 1){
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
