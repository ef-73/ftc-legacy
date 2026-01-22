package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class DropPurple_PushServo {
    private Servo leftDropPurpleServo = null;
    private Servo rightDropPurpleServo = null;

    private static volatile Servo leftIntakePushServo = null;
    private static volatile Servo rightIntakePushServo = null;

    private LinearOpMode mainTask;

    private final double START_POS = 0.65;
    private final double DROP_POS = 0.95;  //0.95
    private final double RESET_POS =0.46;
    private ArmLeftRight armFlag = ArmLeftRight.LEFT;  // 0 -- left arm, 1 - right arm

    private static final double INTAKE_PUSH_SERVO_RESET_POS = 0.80;
    private static final double INTAKE_PUSH_SERVO_PUSH_POS = 0.1;//0.2; //0.15;
    private static final double INTAKE_PUSH_SERVO_PRE_PUSH_POS = 0.4;
    private static final double INTAKE_PUSH_SERVO_FULLY_IN_POS = 0.0;
    private static final double INTAKE_RIGHT_OFFSET = 0.025;

    private boolean toggleArmMove = false;
    private boolean pushServoMovingFlag = false;

    DropPurple_PushServo(LinearOpMode mainTask, boolean auto, ArmLeftRight armPos){

        this.mainTask = mainTask;
        this.armFlag = armPos;
        leftDropPurpleServo = mainTask.hardwareMap.get(Servo.class, "leftDropPurpleServo");
        rightDropPurpleServo = mainTask.hardwareMap.get(Servo.class, "rightDropPurpleServo");

        leftDropPurpleServo.setDirection(Servo.Direction.FORWARD);
        rightDropPurpleServo.setDirection(Servo.Direction.REVERSE);

        leftIntakePushServo = mainTask.hardwareMap.get(Servo.class, "leftIntakePushServo");
        leftIntakePushServo.setDirection(Servo.Direction.FORWARD);


        rightIntakePushServo = mainTask.hardwareMap.get(Servo.class, "rightIntakePushServo");
        rightIntakePushServo.setDirection(Servo.Direction.REVERSE);




            mainTask.sleep(100);
            if (auto){
                if (armFlag == ArmLeftRight.LEFT){

                    leftDropPurpleServo.setPosition(START_POS);
                    rightDropPurpleServo.setPosition(RESET_POS);
                }
                else{
                    leftDropPurpleServo.setPosition(RESET_POS);
                    rightDropPurpleServo.setPosition(START_POS);
                }
                leftIntakePushServo.setPosition(INTAKE_PUSH_SERVO_RESET_POS);
                rightIntakePushServo.setPosition(INTAKE_PUSH_SERVO_RESET_POS + INTAKE_RIGHT_OFFSET);

            }
            else{
             //  leftDropPurpleServo.setPosition(RESET_POS);
              //  rightDropPurpleServo.setPosition(RESET_POS);
            }

        if (auto){
            resetPushServo();
        }
    }

    public void dropPurple(){
        if (armFlag == ArmLeftRight.LEFT){
            leftDropPurpleServo.setPosition(DROP_POS);
        }
        else{
            rightDropPurpleServo.setPosition(DROP_POS);
        }


    }

    public void resetDropServo(){
        leftDropPurpleServo.setPosition(RESET_POS);
        rightDropPurpleServo.setPosition(RESET_POS);
    }



    public void toggleDropArm(){
        if (!toggleArmMove) {
            new Thread(new Runnable() {
                @Override
                public void run() {
                    toggleArmMove = true;
                    if (leftDropPurpleServo.getPosition() > 0.5) {
                        leftDropPurpleServo.setPosition(RESET_POS);
                    } else {
                        leftDropPurpleServo.setPosition(DROP_POS);
                    }
                    if (rightDropPurpleServo.getPosition() > 0.5) {
                        rightDropPurpleServo.setPosition(RESET_POS);
                    } else {
                        rightDropPurpleServo.setPosition(DROP_POS);
                    }
                    mainTask.sleep(400);
                    toggleArmMove = false;
                }
            }).start();
        }

    }

    public void autoSetArmPos(){
        if (armFlag == ArmLeftRight.LEFT){
            leftDropPurpleServo.setPosition(START_POS);

            rightDropPurpleServo.setPosition(RESET_POS);
        }
        else{
            leftDropPurpleServo.setPosition(RESET_POS);
            rightDropPurpleServo.setPosition(START_POS);
        }
    }

    public enum ArmLeftRight {
        LEFT,
        RIGHT,
    }


    private void resetPushServoThread(){
        pushServoMovingFlag = true;
        new Thread(new Runnable() {
            @Override
            public void run() {
                leftIntakePushServo.setPosition(INTAKE_PUSH_SERVO_RESET_POS);
                rightIntakePushServo.setPosition(INTAKE_PUSH_SERVO_RESET_POS);
                mainTask.sleep(600);
                pushServoMovingFlag = false;
            }
        }).start();
    }

    public void setPushServoReady(){
        leftIntakePushServo.setPosition(INTAKE_PUSH_SERVO_PRE_PUSH_POS);
        rightIntakePushServo.setPosition(INTAKE_PUSH_SERVO_PRE_PUSH_POS + INTAKE_RIGHT_OFFSET);
    }

    public void setPushServoPushIn(){
        leftIntakePushServo.setPosition(INTAKE_PUSH_SERVO_PUSH_POS);
        rightIntakePushServo.setPosition(INTAKE_PUSH_SERVO_PUSH_POS + INTAKE_RIGHT_OFFSET);
    }

    public void resetPushServo(){

        leftIntakePushServo.setPosition(INTAKE_PUSH_SERVO_RESET_POS);
        rightIntakePushServo.setPosition(INTAKE_PUSH_SERVO_RESET_POS + INTAKE_RIGHT_OFFSET);
    }

    public void setPushServoFullyInPos(){
        leftIntakePushServo.setPosition(INTAKE_PUSH_SERVO_FULLY_IN_POS);
        rightIntakePushServo.setPosition(INTAKE_PUSH_SERVO_FULLY_IN_POS + INTAKE_RIGHT_OFFSET);
    }

    private void pushPixelToIntake(){
        pushServoMovingFlag = true;
        new Thread(new Runnable() {
            @Override
            public void run() {

                leftIntakePushServo.setPosition(INTAKE_PUSH_SERVO_PUSH_POS);
                rightIntakePushServo.setPosition(INTAKE_PUSH_SERVO_PUSH_POS + INTAKE_RIGHT_OFFSET);
                mainTask.sleep(400);
                leftIntakePushServo.setPosition(INTAKE_PUSH_SERVO_RESET_POS);
                rightIntakePushServo.setPosition(INTAKE_PUSH_SERVO_RESET_POS + INTAKE_RIGHT_OFFSET);
                pushServoMovingFlag = false;
            }
        }).start();
    }

    public void teleOp_pushServo(){
        if (mainTask.gamepad1.x  && !pushServoMovingFlag){
            //resetPushServo();
            pushPixelToIntake();
        }
        if (mainTask.gamepad1.right_stick_button){
            setPushServoReady();
        }
    }




}
