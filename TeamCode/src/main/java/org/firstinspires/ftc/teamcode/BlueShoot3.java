package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Sean O on 11/23/2016.
 */
@Autonomous(name="Blue Shoot 3", group="Blue")
public class BlueShoot3 extends AutonomousBase {
    @Override
    public void gameState() {
        super.gameState();
        switch(gameState){
            case 0: //Start
                if(actualRuntime() > 3 && !gyro.isCalibrating()) {
                    gameState = 1;
                    map.setRobot(3,11.25);
                }
                break;
            case 1: //moves to shooter post
                map.setGoal(4, 10);
                if(linedUp()){
                    moveState = MoveState.FORWARD;
                }else{
                    moveState = MoveState.TURN_TOWARDS_GOAL;
                }
                if(map.distanceToGoal()<=.1){
                    moveState = MoveState.STOP;
                    gameState = 2;
                }
                break;
            case 2: // turns ...
                desiredAngle = 220;
                if(linedUpAngle()){
                    moveState = MoveState.STOP;
                    gameState = 3;
                    sTime = getRuntime();
                }else{
                    moveState = MoveState.TURN_TOWARDS_ANGLE_SLOW;
                }
                break;
            case 3: // ... and shoots
                moveState = MoveState.SHOOT_WHEEL;
                if(getRuntime() - sTime >= 2) {
                    moveState = MoveState.SHOOT_CONVEYOR;
                }
                if(getRuntime() - sTime >= 5) {
                    moveState = MoveState.SHOOT_STOP;
                    gameState = 100;
                    sTime = getRuntime();
                }
                break;
            case 100:
                if (getRuntime() - sTime > 1) {
                    gameState = 4;
                }
                moveState = MoveState.STOP;
                break;
            case 4: //MOVE TO KNOCK OFF BALL
                map.setGoal(7,7);
                if(linedUp()){
                    moveState = MoveState.FORWARD;
                }else{
                    moveState = MoveState.TURN_TOWARDS_GOAL;
                }
                if(map.distanceToGoal()<=.1){
                    moveState = MoveState.STOP;
                }
                break;
            case 777:
                moveState = MoveState.STOP;
                break;
        }
    }
}
