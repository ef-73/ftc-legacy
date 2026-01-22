/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.List;


@Autonomous(name="Red_Left3White_Outside")
//@Disabled
//@Config
public class AutoRed_Left3White extends LinearOpMode {
    private boolean oneWhite = false;
    private int waitTime = 8000;

    private int firstDropLevel = 2;  // 2 -- in case teammate put pixel on it
    private final double YELLOW_POS_OFFSET = 0.035 ;//


    ElapsedTime runTime = new ElapsedTime();
    ElapsedTime timeOutTimer = new ElapsedTime();

    public DigitalChannel redLed = null;
    public DigitalChannel greenLed = null;
    private AirPlaneLaunch airPlaneLaunch = null;

    private DropPurple_PushServo dropPurplePushServo = null;

    ChassisSystem chassis = null;
    Odometry odometry = null;
    MecanumDriveLib chassisController = null;
    RobotPosition robotPos = new RobotPosition(0,0,0);
    RobotPosition pickUpPos = new RobotPosition(0,0,0);

    public static double targetVel = 1.0;
    public static double turnVel = 1.0;

    AprilTagTeamObjectDetector aprilTagTeamObjectDetector = null;
    private SliderAndTraySystem_Servo sliderTrayController = null;




    // for red side
    private AllianceColor teamColor = AllianceColor.RED;
    RobotPosition startPosition = new RobotPosition(-0.885,-1.62 + 0.03,90);  // start position for red side
    private final double FACING_TAG_ANGLE = 180;
    RobotPosition leftDropPosition = new RobotPosition(startPosition.x - 0.52, startPosition.y + 0.78, startPosition.angle);
    RobotPosition rightDropPosition = new RobotPosition(startPosition.x + 0.03, startPosition.y + 0.51, startPosition.angle);
    RobotPosition middleDropPosition = new RobotPosition(startPosition.x - 0.38, startPosition.y + 0.79, startPosition.angle);


    RobotPosition whitePilePosition = new RobotPosition(-1.45, -0.305, FACING_TAG_ANGLE);  // y=-0.27

    RobotPosition leftTagPos = new RobotPosition(1.55 -0.4,-0.74 , FACING_TAG_ANGLE); //tag4
    RobotPosition middleTagPos = new RobotPosition(1.55 - 0.4,-0.90 , FACING_TAG_ANGLE); //tag5
    RobotPosition rightTagPos = new RobotPosition(1.55 - 0.4,-1.06  , FACING_TAG_ANGLE); // tag6  1.04


    RobotPosition camPos = new RobotPosition(0,0,0);

    RobotPosition yellowDeliverPosition = new RobotPosition(0,0,0);

    RobotPosition whiteDeliverPosition = new RobotPosition(0,0,0);

    RobotPosition deliverPosition = new RobotPosition(0,0,0);

    TeamObjectPosition teamObjectPosition = TeamObjectPosition.NULL;
    boolean deliverWhite = false;


    DistanceSensor leftDisSensor = null;
    DistanceSensor rightDisSensor = null;

    double costTime = 0;


    int moveStep = 0;

    @Override
    public void runOpMode(){
        auto_init();
/////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        aprilTagTeamObjectDetector.startTeamObjectDetection();
        odometry.setRobotPosition( startPosition);  // in case move robot
        sliderTrayController.enableTrayServo();
        dropPurplePushServo.resetPushServo();
        sliderTrayController.startResetCheck();
        sliderTrayController.trayCloseBothLock();
        runTime.reset();

        sleep(100);
        while ( runTime.milliseconds() < 100 || aprilTagTeamObjectDetector.getTeamObjectPosition() == TeamObjectPosition.NULL) {
            if (runTime.milliseconds() > 500){
                teamObjectPosition = TeamObjectPosition.RIGHT;
                break;
            }
            sleep(10);  // wait for detecting 5 frames
        }
        telemetry.addLine("Time = " + runTime.milliseconds());
        teamObjectPosition = aprilTagTeamObjectDetector.getTeamObjectPosition();
        telemetry.update();
        aprilTagTeamObjectDetector.stopTeamObjectDetect();


        aprilTagTeamObjectDetector.doCameraSwitching(AprilTagTeamObjectDetector.WebCamPos.BackCam);  //switch to back camera
        aprilTagTeamObjectDetector.startAprilTagDetect();

        aprilTagTeamObjectDetector.aprilTagTeamObject_Detect_Thread_Task();
        aprilTagTeamObjectDetector.enablePosUpdate();

        if (opModeIsActive()) {

            if (teamObjectPosition == TeamObjectPosition.LEFT) {
                telemetry.addLine("Left Position");

            } else if (teamObjectPosition == TeamObjectPosition.MIDDLE) {
                telemetry.addLine("Middle Position");
            } else {
                //right
                telemetry.addLine("Right Position");
            }
            telemetry.update();


            if (teamObjectPosition == TeamObjectPosition.RIGHT) {
                runTime.reset();
                rightTagPos.y = rightTagPos.y + 0.105;  // move to right, outside
                yellowDeliverPosition.setPosition(rightTagPos);
                whiteDeliverPosition.setPosition(middleTagPos);



                chassisController.p2pDrive(rightDropPosition, 1.0, 0.3, 0, 0.02, 2, 0, true, 2000);
                moveStep = 0;
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    sleep(10);
                }
                // drop preload here
                dropPurplePushServo.dropPurple();
                sleep(500);

                dropPurplePushServo.setPushServoReady();
                //move to pile
                chassisController.p2pDrive(whitePilePosition, 1.0, 1.0, 0, 0.01, 2, 0, true, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {

                    sleep(10);
                }
                dropPurplePushServo.resetDropServo();
                pickDeliverPixel();

                costTime = runTime.milliseconds();


            } else if (teamObjectPosition == TeamObjectPosition.MIDDLE) {
                // move to middle

                runTime.reset();
                middleTagPos.y = middleTagPos.y + 0.095; // move close to rightTag
                yellowDeliverPosition.setPosition(middleTagPos);
                whiteDeliverPosition.setPosition(rightTagPos);

                chassisController.p2pDrive(middleDropPosition, 1.0, 1.0, 0, 0.02, 2, 0, true, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {

                    sleep(10);
                }
                dropPurplePushServo.setPushServoReady();
                dropPurplePushServo.dropPurple();
                sleep(500);
                //move to pile
                //chassisController.setP2PXPid(1.0, 0.1, 8);
                //chassisController.setP2PYPid(1.5, 0.1, 5);
                chassisController.resetP2PPID();
                chassisController.setP2PTurnPID(0.01, 0.1, 0.1);
                chassisController.p2pDrive(new RobotPosition(whitePilePosition.x, whitePilePosition.y - 0.0, whitePilePosition.angle), 0.25, 0.2, 0, 0.01, 1, 0, true, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    sleep(10);
                }
                dropPurplePushServo.resetDropServo();
                pickDeliverPixel();
                costTime = runTime.milliseconds();

            } else if (teamObjectPosition == TeamObjectPosition.LEFT) {
                // move to left
                runTime.reset();
                leftTagPos.y = leftTagPos.y - 0.06; // move a little bit to outside
                yellowDeliverPosition.setPosition(leftTagPos);
                whiteDeliverPosition.setPosition(middleTagPos);


                chassisController.p2pDrive(new RobotPosition(startPosition.x - 0.53, startPosition.y + 0.45, startPosition.angle), 1.0, 1.0, 0, 0.04, 4, 0, false, 2000);
                moveStep = 0;
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {

                    sleep(10);
                }
                chassisController.resetP2PPID();

                chassisController.p2pDrive(leftDropPosition, 1.0, 1.0, 0, 0.01, 2, 0, true, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    sleep(10);
                }
                //drop cone here
                dropPurplePushServo.dropPurple();
                sleep(500);

                // move forward a little bit first
                chassisController.p2pDrive(new RobotPosition(whitePilePosition.x + 0.025, whitePilePosition.y, startPosition.angle), 1.0, 1.0, 0, 0.01, 2, 0.0, false, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    sleep(10);
                }
                dropPurplePushServo.resetDropServo();

                chassisController.turnToAngle(FACING_TAG_ANGLE, 1.0,MecanumDriveLib.TURN_WHEEL.WHEELS_TURN,1.0,true,1500);
                while (opModeIsActive() && !gamepad1.back && !chassisController.turnIsSettled()) {
                    sleep(10);
                }
                dropPurplePushServo.setPushServoReady();
                sleep(100);
                chassisController.resetP2PPID();


                pickDeliverPixel();

                costTime = runTime.milliseconds();
            }
        }
        chassis.stopRobot();
        while(opModeIsActive()){
            sleep(10);
        }
        chassis.stopRobot();

        odometry.stopThread();
        if (aprilTagTeamObjectDetector != null){
            aprilTagTeamObjectDetector.stopAprilTagDetect();
            aprilTagTeamObjectDetector.closeDetector();
        }


    }



    public void moveToPixel(double pwr, boolean sensorFlag){
        setLedGreen();
        sleep(200);
        double leftOffset  = 0;  // 20mm
        //average ????????? 2 times
        double leftDis = leftDisSensor.getDistance(DistanceUnit.MM) + leftOffset; //mm
        double rightDis = rightDisSensor.getDistance(DistanceUnit.MM);
        sleep(10);
        leftDis = 0.3 * leftDis + 0.7 * (leftDisSensor.getDistance(DistanceUnit.MM) + leftOffset); //mm
        rightDis = 0.3 * rightDis + 0.7 * (rightDisSensor.getDistance(DistanceUnit.MM));

        if (sensorFlag) {
            if (Math.abs(leftDis) < 400 && Math.abs(rightDis) < 400) {

                double deltaDis = leftDis - rightDis;

                int tryCnt = 0;
                while (Math.abs(deltaDis) > 20 && tryCnt < 2) {  // only correct twice ? once? to save time
                    setLedGreen();
                    tryCnt++;
                    robotPos.setPosition(odometry.getRobotPosition());
                    if (deltaDis > 0) {
                        chassisController.p2pDrive(new RobotPosition(robotPos.x, robotPos.y + 0.05, FACING_TAG_ANGLE), 1.0, 1.0, 0, 0.01,
                                2, 0.0, true, 1000);


                    } else {
                        chassisController.p2pDrive(new RobotPosition(robotPos.x, robotPos.y - 0.05, FACING_TAG_ANGLE), 1.0, 1.0, 0, 0.01,
                                2, 0.0, true, 1000);
                    }
                    while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                        sleep(10);
                    }
                    sleep(150);
                    leftDis = leftDisSensor.getDistance(DistanceUnit.MM);
                    rightDis = rightDisSensor.getDistance(DistanceUnit.MM);
                    deltaDis = leftDis - rightDis;


                }
                chassis.stopRobot();

            }
        }
        chassisController.stopAllThread();
        sleep(50);
        pickUpPos.setPosition(odometry.getRobotPosition());  // remember current position as next pick up position
        sliderTrayController.setToIntakePos();

        /*
        robotPos = odometry.getRobotPosition();
        chassisController.p2pDrive(new RobotPosition(robotPos.x - 0.2, robotPos.y, FACING_TAG_ANGLE), pwr, 1.0, 0, 0.01,
                2, pwr, true, 300);
        while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
            sleep(10);
        }
        */
        chassis.driveRobot(pwr, pwr,pwr,pwr);
        timeOutTimer.reset();
        double  minDis = Math.min(leftDisSensor.getDistance(DistanceUnit.MM), rightDisSensor.getDistance(DistanceUnit.MM));
        while(opModeIsActive() && timeOutTimer.milliseconds() < 450 && minDis > 55){
            chassis.driveRobot(pwr, pwr,pwr,pwr);
            sleep(10);
            minDis = Math.min(leftDisSensor.getDistance(DistanceUnit.MM), rightDisSensor.getDistance(DistanceUnit.MM));
            if (minDis <= 55){
                setLedRed();
            }

        }
        chassis.stopRobot();

    }


    private void deliverPixel(boolean whitePixel) {

        if (!whitePixel){
            deliverPosition.setPosition(middleTagPos);
        }else {
            deliverPosition.setPosition(whiteDeliverPosition);
        }

        //chassisController.setP2PYPid(1.0,0.1,5.0);
        chassisController.setP2PYPid(2.0,MecanumDriveLib.P2PY_KI,MecanumDriveLib.P2PY_KD);
        // offset center line a little bit - 0.03
        chassisController.p2pDrive(new RobotPosition(0.8, pickUpPos.y , FACING_TAG_ANGLE), targetVel, turnVel, 0.0, 0.1, 5, 0.2, true, 3000);
        boolean armNotUp = true;
        while (opModeIsActive() && !chassisController.p2pIsSettled()) {

            if (odometry.getRobotPosition().x > 0.3 && armNotUp) {
                armNotUp = false;
                if (!whitePixel) {
                    if (teamObjectPosition == TeamObjectPosition.LEFT) {
                        sliderTrayController.autoToDropLevelPos(firstDropLevel, -90);
                    }
                    else if(teamObjectPosition == TeamObjectPosition.RIGHT){
                        sliderTrayController.autoToDropLevelPos(firstDropLevel, 90);
                    }
                    else{
                        sliderTrayController.autoToDropLevelPos(firstDropLevel, 90);
                    }
                }
                else{
                    if (teamObjectPosition == TeamObjectPosition.LEFT) {
                        sliderTrayController.autoToDropLevelPos(1, 0);
                    }
                    else if(teamObjectPosition == TeamObjectPosition.RIGHT){
                        sliderTrayController.autoToDropLevelPos(1, 0);
                    }
                    else{
                        sliderTrayController.autoToDropLevelPos(1, -10);
                    }
                }

            }
            sleep(10);
        }
        chassisController.resetP2PPID();


        chassisController.p2pDrive(new RobotPosition(deliverPosition.x - 0.13, deliverPosition.y, FACING_TAG_ANGLE), targetVel, turnVel, 0.2, 0.02, 1, 0.0, true, 2000);



        while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {

            sleep(10);
        }

        if (!whitePixel) {
            sleep(300);  // give some time for camera to detect april tag
            if (aprilTagTeamObjectDetector.getTagIDOrNot()) {
                robotPos.setPosition(odometry.getRobotPosition());
                camPos.setPosition(aprilTagTeamObjectDetector.getRobotPosition());

                if (camPos.dis2Pos(robotPos) < 0.15) {  // make sure camPos is right
            //        if (camPos.dis2Pos(deliverPosition) > 0.02) {

                        //just for debug
                        double deltaX = camPos.x - robotPos.x;
                        double deltaY = camPos.y - robotPos.y;
                        // saveToFile("CamPos.txt", new double[]{deltaX,deltaY});
                        pickUpPos.x = pickUpPos.x + deltaX;
                        pickUpPos.y = pickUpPos.y +  deltaY;


                        odometry.setRobotPositionXY(camPos);
                        setLedGreen();

                        chassisController.resetP2PPID();
                        chassisController.p2pDrive(new RobotPosition(yellowDeliverPosition.x, yellowDeliverPosition.y, FACING_TAG_ANGLE), targetVel, turnVel, 0.2, 0.02, 1, 0.0, true, 1000);
                        while (opModeIsActive() && !chassisController.p2pIsSettled()) {
                            sleep(10);
                        }
      //              } else {
       //                 odometry.setRobotPositionXY(camPos);

      //              }

                }
                else{
                    chassisController.resetP2PPID();
                    chassisController.p2pDrive(new RobotPosition(yellowDeliverPosition.x, yellowDeliverPosition.y, FACING_TAG_ANGLE), targetVel, turnVel, 0.2, 0.02, 1, 0.0, true, 1000);
                    while (opModeIsActive() && !chassisController.p2pIsSettled()) {
                        sleep(10);
                    }
                }
            }
            else{
                chassisController.resetP2PPID();
                chassisController.p2pDrive(new RobotPosition(yellowDeliverPosition.x, yellowDeliverPosition.y, FACING_TAG_ANGLE), targetVel, turnVel, 0.2, 0.02, 1, 0.0, true, 1000);
                while (opModeIsActive() && !chassisController.p2pIsSettled()) {
                    sleep(10);
                }
            }
        }

        sleep(100);
        chassisController.stopAllThread();



        chassis.driveRobot(-0.4, -0.4, -0.4, -0.4);
        timeOutTimer.reset();
        while(opModeIsActive() && sliderTrayController.touchBoardSensor.getState() && timeOutTimer.milliseconds() < 350){
            chassis.driveRobot(-0.4, -0.4, -0.4, -0.4);
            sleep(10);
        }

        chassis.stopRobot();

        sliderTrayController.trayOpenBothLock();
        sleep(300);





    }


    public void gotoPickupPos(){
        dropPurplePushServo.setPushServoReady();
        setLedRed();
        aprilTagTeamObjectDetector.stopAprilTagDetect();

   //     sliderTrayController.setArmTiltAngle(sliderTrayController.getArmTiltAngle() + 20);
   //     sleep(500);
        chassis.driveRobot(0.3,0.3,0.3,0.3);
        sleep(300);
        chassis.stopRobot();
        sliderTrayController.resetArmToPickupPos();
        chassisController.p2pDrive(new RobotPosition(1.0, pickUpPos.y + 0.02, FACING_TAG_ANGLE), 1.0, turnVel, 0.0, 0.1, 5, 0.0, true, 2000);

        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){

            sleep(10);
        }
        chassisController.resetP2PPID();
       // robotPos = odometry.getRobotPosition();
       // saveToFile("middlePos.txt", new double[]{robotPos.x, robotPos.y});

     //   chassisController.setP2PYPid(1.5,0.1,5);
     //   chassisController.setP2PYPid(2.0,0.1,8);

        chassisController.setP2PYPid(2.0,MecanumDriveLib.P2PY_KI,MecanumDriveLib.P2PY_KD);
        if (teamObjectPosition == TeamObjectPosition.RIGHT){
            chassisController.p2pDrive(new RobotPosition(pickUpPos.x + 0.00,pickUpPos.y + 0.01,FACING_TAG_ANGLE),targetVel,turnVel,0.2,0.02,4,0.0,true,5000);
        }else if(teamObjectPosition == TeamObjectPosition.MIDDLE) {
            chassisController.p2pDrive(new RobotPosition(pickUpPos.x + 0.00,pickUpPos.y + 0.01,FACING_TAG_ANGLE),targetVel,turnVel,0.2,0.02,4,0.0,true,5000);
        }
        else{// left
            chassisController.p2pDrive(new RobotPosition(pickUpPos.x + 0.00,pickUpPos.y + 0.01,FACING_TAG_ANGLE),targetVel,turnVel,0.2,0.02,4,0.0,true,5000);
        }

        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){
            sleep(10);
        }


        chassisController.resetP2PPID();

        telemetry.addLine(String.format("Pos: %3.3f, %3.3f,%3.3f", odometry.getRobotPosition().x, odometry.getRobotPosition().y, odometry.getRobotPosition().angle));
        telemetry.update();
        robotPos = odometry.getRobotPosition();
       // saveToFile("PickUpPos.txt", new double[]{robotPos.x, robotPos.y});

    }

    public void pickDeliverPixel(){
        sliderTrayController.intakeIn();
        moveToPixel(0.35, false);
        if (!oneWhite) {

            // chassis.driveRobot(-0.1,-0.1,-0.1,-0.1); // move back a little bit by time 100ms
            sliderTrayController.setToIntakePos();
            dropPurplePushServo.setPushServoPushIn();
            sleep(400);
            dropPurplePushServo.resetPushServo();
            sleep(400);

            deliverPixel(false);
            dropPurplePushServo.setPushServoReady();


            gotoPickupPos();

            sliderTrayController.intakeIn();
            dropPurplePushServo.setPushServoReady();
            moveToPixel(0.35, true);
            //chassis.driveRobot(-0.1,-0.1,-0.1,-0.1); // move back a little bit by time 100ms
            sliderTrayController.setToIntakePos();
            dropPurplePushServo.setPushServoPushIn();
            sleep(500);
            dropPurplePushServo.setPushServoReady();
            sleep(400);
            dropPurplePushServo.setPushServoPushIn();
            sleep(500);
            dropPurplePushServo.resetPushServo();
            sleep(400);


            if (teamObjectPosition != TeamObjectPosition.LEFT) {
                leftTagPos.y = leftTagPos.y - 0.03;
                whiteDeliverPosition.setPosition(leftTagPos);
            } else {
                whiteDeliverPosition.setPosition(middleTagPos);
            }

            deliverPixel(true);

        }
        else{
            // chassis.driveRobot(-0.1,-0.1,-0.1,-0.1); // move back a little bit by time 100ms
            sliderTrayController.setToIntakePos();
            dropPurplePushServo.setPushServoPushIn();
            sleep(600);
            dropPurplePushServo.resetPushServo();
            sleep(800);
            sliderTrayController.intakeStop();
            sleep(waitTime);
            deliverPixel(false);

        }
        dropPurplePushServo.resetPushServo();
        chassis.driveRobot(0.4, 0.4, 0.4, 0.4);
        sleep(150);
        chassis.driveRobot(0.4, 0.4, 0.4, 0.4);
        sliderTrayController.resetArmToPickupPos();
        sleep(50);
        chassis.stopRobot();
    }


    public void initLed(){
        redLed = hardwareMap.get(DigitalChannel.class, "red2");
        greenLed = hardwareMap.get(DigitalChannel.class, "green2");


        redLed.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed.setMode(DigitalChannel.Mode.OUTPUT);

        redLed.setState(true);
        greenLed.setState(false);
    }

    public void setLedRed(){
        redLed.setState(true);
        greenLed.setState(false);
    }

    public void setLedGreen(){

        redLed.setState(false);
        greenLed.setState(true);
    }


    public void waitForTouch(){
        while(opModeIsActive() && !gamepad1.back){
            RobotPosition currentPosition  = odometry.getRobotPosition();
            telemetry.addLine(String.format("Pos %3.3f, %3.3f, %3.3f", currentPosition.x, currentPosition.y, currentPosition.angle));
            RobotPosition camPosition  = aprilTagTeamObjectDetector.getRobotPosition();
            telemetry.addLine(String.format("Cam Pos %3.3f, %3.3f, %3.3f", camPosition.x, camPosition.y, camPosition.angle));
            telemetry.update();
            sleep(10);
        }

    }

    private void saveToFile(String FileName, double[] value)
    {
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FIRST/debug");
        dir.mkdirs();
        File file = new File(dir, FileName);
        try {
            FileOutputStream f = new FileOutputStream(file, false);
            final PrintStream printStream = new PrintStream(f);

            //red lower threshold first
            for (int i = 0; i < value.length; i++) {
                printStream.println(Double.toString(value[i]));
            }

            printStream.close();
            f.close();
        }
        catch(Exception e){

        }

    }

    public void auto_init(){
        initLed();
        leftDisSensor = hardwareMap.get(DistanceSensor.class, "leftDis");
        rightDisSensor = hardwareMap.get(DistanceSensor.class, "rightDis");

        airPlaneLaunch = new AirPlaneLaunch(this);


        // could save time for reading encoder
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        chassis = new ChassisSystem(this);
        sleep(500);

/*
        telemetry.addLine("Please lift the tray and put in the yellow pixel under back lock, then press the gamepad1 y button");
        telemetry.update();

        while(!gamepad1.y){
            sleep(100);
        }
        sliderTrayController.trayCloseBothLock();
        sleep(500);
        sliderTrayController.trayCloseBothLock();
        sleep(500);
        telemetry.addLine("Please make sure the back lock hold the yellow pixel, if not restart the program, if yes, push the slide back, put robot to right position, then press the gamepad1 Y button again");
        telemetry.update();
        while(!gamepad1.y){
            sleep(100);
        }
        sleep(500);


 */
        dropPurplePushServo = new DropPurple_PushServo(this, true, DropPurple_PushServo.ArmLeftRight.RIGHT);
        sleep(500);


        odometry = new Odometry(this,chassis, startPosition, true);
        sleep(2000);
        chassisController = new MecanumDriveLib(this, odometry,chassis);
        sleep(500);



        sliderTrayController = new SliderAndTraySystem_Servo(this, chassis,true);
        aprilTagTeamObjectDetector = new AprilTagTeamObjectDetector(this, teamColor, odometry, "WebcamFront", "WebcamBack");

        sleep(2000);

        // set start position

        odometry.setRobotPosition( startPosition);
        sleep(2000);
        dropPurplePushServo.autoSetArmPos();

        sliderTrayController.trayLockHoldPos();
        telemetry.addData("Working Mode", "waiting for start, make sure the purple is close to the rack");
        telemetry.update();
    }


}
