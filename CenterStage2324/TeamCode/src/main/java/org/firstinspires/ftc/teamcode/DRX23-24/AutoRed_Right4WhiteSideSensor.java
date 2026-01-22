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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;


@TeleOp(name="Red_Right4WhiteSideSensor")
//@Disabled
//@Config
public class AutoRed_Right4WhiteSideSensor extends LinearOpMode {

    private int firstDropLevel = 0;  // 2 -- if with 19836  0 --  normal
    private final double YELLOW_POS_OFFSET =  0.035 ;//

    ElapsedTime runTime = new ElapsedTime();
    ElapsedTime timeOutTimer = new ElapsedTime();

    public DigitalChannel redLed = null;
    public DigitalChannel greenLed = null;



    AirPlaneLaunch airPlaneLaunch = null;
    ChassisSystem chassis = null;
    Odometry odometry = null;
    MecanumDriveLib chassisController = null;
    DropPurple_PushServo dropPurplePushServo = null;

    RobotPosition robotPos = new RobotPosition(0,0,0);
    RobotPosition pickUpPos = new RobotPosition(0,0,0);

    public static double targetVel = 1.0;
    public static double turnVel = 1.0;

    AprilTagTeamObjectDetector aprilTagTeamObjectDetector = null;
    private SliderAndTraySystem_Servo sliderTrayController = null;


    //red side
    private AllianceColor teamColor = AllianceColor.RED;
    private final double FACING_TAG_ANGLE = 180;
    RobotPosition startPosition = new RobotPosition(0.2925,-1.62,90);  // start position for red side

    RobotPosition leftDropPosition = new RobotPosition(startPosition.x - 0.03, startPosition.y + 0.51, startPosition.angle);
    RobotPosition rightDropPosition = new RobotPosition(startPosition.x + 0.52, startPosition.y + 0.53, startPosition.angle);
    RobotPosition middleDropPosition = new RobotPosition(startPosition.x + 0.40, startPosition.y + 0.81, startPosition.angle);



    RobotPosition rightTagPos = new RobotPosition(1.55 -0.4,-1.05  + YELLOW_POS_OFFSET, FACING_TAG_ANGLE); //tag4
    RobotPosition middleTagPos = new RobotPosition(1.55 - 0.4,-0.90 + YELLOW_POS_OFFSET , FACING_TAG_ANGLE); //tag5
    RobotPosition leftTagPos = new RobotPosition(1.55 - 0.4,-0.75  + YELLOW_POS_OFFSET, FACING_TAG_ANGLE); // tag6  1.04

    RobotPosition parkPosition = new RobotPosition(1.55 - 0.4, -1.54, FACING_TAG_ANGLE);
    RobotPosition whitePilePosition = new RobotPosition(-1.45, -0.30, FACING_TAG_ANGLE);  // y=-0.27

    RobotPosition camPos = new RobotPosition(0,0,0);



    RobotPosition yellowDeliverPosition = new RobotPosition(0,0,0);
    RobotPosition whiteDeliverPosition = new RobotPosition(0,0,0);

    RobotPosition deliverPosition = new RobotPosition(0,0,0);

    TeamObjectPosition teamObjectPosition = TeamObjectPosition.NULL;
    boolean deliverWhite = false;


    DistanceSensor leftDisSensor = null;
    DistanceSensor rightDisSensor = null;
    DistanceSensor rightSideDisSensor = null;
    DistanceSensor leftSideDisSensor = null;



    double costTime = 0;

    // for blue side
    //RobotPosition startPosition = new RobotPosition(-0.9,1.57,-90);  // start position for blue side
    // for red side

    int moveStep = 0;

    @Override
    public void runOpMode(){
        initLed();

        leftSideDisSensor = hardwareMap.get(DistanceSensor.class, "leftSideDis");
        rightSideDisSensor = hardwareMap.get(DistanceSensor.class, "rightSideDis");

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

        sliderTrayController = new SliderAndTraySystem_Servo(this, chassis,true);
        aprilTagTeamObjectDetector = new AprilTagTeamObjectDetector(this, teamColor, odometry, "WebcamFront", "WebcamBack");

        sleep(2000);


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
        dropPurplePushServo = new DropPurple_PushServo(this, true, DropPurple_PushServo.ArmLeftRight.LEFT);
        sleep(500);


        odometry = new Odometry(this,chassis, startPosition, true);
        sleep(2000);
        chassisController = new MecanumDriveLib(this, odometry,chassis);
        sleep(500);
        sliderTrayController.trayLockHoldPos();

        // set start position

        odometry.setRobotPosition( startPosition);
        sleep(2000);
        dropPurplePushServo.autoSetArmPos();


        telemetry.addData("Working Mode", "waiting for start, make sure the purple is close to the rack");
        telemetry.update();
/////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        aprilTagTeamObjectDetector.startTeamObjectDetection();
        runTime.reset();
        odometry.setRobotPosition( startPosition);  // in case move robot
        dropPurplePushServo.resetPushServo();
        sliderTrayController.startResetCheck();
        sliderTrayController.trayCloseBothLock();

        if (aprilTagTeamObjectDetector != null){
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
        }
        else{
            sleep(200);
            teamObjectPosition = TeamObjectPosition.RIGHT;
        }



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


            if (teamObjectPosition == TeamObjectPosition.LEFT) {
                whitePilePosition.y = whitePilePosition.y + 0.0;
                whitePilePosition.x = whitePilePosition.x - 0.01;
                pickUpPos.setPosition(whitePilePosition);
                yellowDeliverPosition.setPosition(leftTagPos);
                whiteDeliverPosition.setPosition(middleTagPos);

                // move to left
                chassisController.p2pDrive(leftDropPosition, 1.0, 0.3, 0, 0.02, 2, 0, true, 2000);
                moveStep = 0;
                while (opModeIsActive() && !chassisController.p2pIsSettled()) {
                    if (odometry.getRobotPosition().y > -1.3 && moveStep == 0){
                        moveStep = 1;
                        sliderTrayController.autoToDropLevelPos(0,90);
                    }
                    sleep(10);
                }
                // drop preload here
                dropPurplePushServo.dropPurple();
                sleep(300);
                dropYellowPixel();
                pickWhitePixel();

                costTime = runTime.milliseconds();


            } else if (teamObjectPosition == TeamObjectPosition.MIDDLE) {
                // move to middle
                whitePilePosition.y = whitePilePosition.y + 0.0;
                whitePilePosition.x = whitePilePosition.x - 0.01;
                pickUpPos.setPosition(whitePilePosition);
                yellowDeliverPosition.setPosition(middleTagPos);

                leftTagPos.y = leftTagPos.y - YELLOW_POS_OFFSET;
                whiteDeliverPosition.setPosition(leftTagPos);

                chassisController.p2pDrive(middleDropPosition, 1.0, 1.0, 0, 0.02, 2, 0, true, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    if (odometry.getRobotPosition().y > -1.3 && moveStep == 0){
                        moveStep = 1;
                        sliderTrayController.autoToDropLevelPos(0,90);
                    }
                    sleep(10);
                }

                dropPurplePushServo.dropPurple();
                sleep(400);

                dropYellowPixel();
                pickWhitePixel();

                costTime = runTime.milliseconds();

            } else if (teamObjectPosition == TeamObjectPosition.RIGHT) {
                // move to right
                whitePilePosition.y = whitePilePosition.y + 0.015;
                whitePilePosition.x = whitePilePosition.x - 0.01;
                pickUpPos.setPosition(whitePilePosition);
                yellowDeliverPosition.setPosition(rightTagPos);

                leftTagPos.y = leftTagPos.y - YELLOW_POS_OFFSET;
                whiteDeliverPosition.setPosition(leftTagPos);
                chassisController.resetP2PPID();

                chassisController.p2pDrive(new RobotPosition(startPosition.x + 0.54, startPosition.y + 0.45, startPosition.angle), 1.0, 1.0, 0, 0.04, 4, 0, false, 2000);
                moveStep = 0;
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    if (odometry.getRobotPosition().y > -1.3 && moveStep == 0){
                        moveStep = 1;
                        sliderTrayController.autoToDropLevelPos(0,90);
                    }
                    sleep(10);
                }


                chassisController.p2pDrive(rightDropPosition, 1.0, 1.0, 0, 0.01, 2, 0, true, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    sleep(10);
                }
                //drop cone here
                dropPurplePushServo.dropPurple();
                sleep(400);

                dropYellowPixel();
                pickWhitePixel();

                costTime = runTime.milliseconds();
            }
        }

        while(opModeIsActive() ){
            telemetry.addLine(String.format("Time= %4.4f", costTime));
            telemetry.update();
            sleep(10);
        }
        odometry.stopThread();
        if (aprilTagTeamObjectDetector != null){
            aprilTagTeamObjectDetector.stopAprilTagDetect();
            aprilTagTeamObjectDetector.closeDetector();
        }



    }



    private void dropYellowPixel() {

        deliverPosition.setPosition(yellowDeliverPosition);

        chassisController.setP2PTurnPID(MecanumDriveLib.turnKp, MecanumDriveLib.turnKi,MecanumDriveLib.turnKd);
        chassisController.p2pDrive(new RobotPosition(deliverPosition.x - 0.1 , deliverPosition.y, FACING_TAG_ANGLE), 1.0, 0.3, 0, 0.02, 2, 0, true, 5000);
        moveStep = 0;
        while (opModeIsActive() && !chassisController.p2pIsSettled()) {
            sleep(10);
        }
        chassisController.resetP2PPID();
        if (aprilTagTeamObjectDetector != null) {
            if (aprilTagTeamObjectDetector.getTagIDOrNot()) {
                robotPos.setPosition(odometry.getRobotPosition());
                camPos.setPosition(aprilTagTeamObjectDetector.getRobotPosition());
                if (camPos.dis2Pos(robotPos) < 0.2) {  // make sure camPos is right
                    if (camPos.dis2Pos(deliverPosition) > 0.02) {
                        odometry.setRobotPositionXY(camPos);
                        setLedGreen();


                    }
                }
            }
        }

        chassisController.resetP2PPID();
        chassisController.p2pDrive(new RobotPosition(deliverPosition.x, deliverPosition.y, FACING_TAG_ANGLE), targetVel, turnVel, 0.2, 0.02, 1, 0.0, true, 1000);
        while (opModeIsActive() && !chassisController.p2pIsSettled()) {
            sleep(10);
        }
        sleep(50);
        dropPurplePushServo.resetDropServo();
        chassisController.stopAllThread();

        chassis.driveRobot(-0.35, -0.35, -0.35, -0.35);
        timeOutTimer.reset();
        while(opModeIsActive() && sliderTrayController.touchBoardSensor.getState() && timeOutTimer.milliseconds() < 400){
            sleep(10);
        }

        chassis.stopRobot();

        sliderTrayController.trayOpenBothLock();
        sleep(300);
        chassis.stopRobot();

        dropPurplePushServo.resetPushServo();





    }

    public void pickWhitePixel(){
        //gotoPickupPos();
        gotoPickupPosWithSideSensor();

        dropPurplePushServo.setPushServoPushIn();
        sliderTrayController.setToIntakePos();
        //    sleep(100);
        //     chassis.stopRobot();
        //    sliderTrayController.setToIntakePos();
        sleep(300);
        dropPurplePushServo.setPushServoReady();
        sleep(300);
        dropPurplePushServo.setPushServoFullyInPos();
        sleep(300);
        dropPurplePushServo.resetPushServo();
         sleep(400);
        sliderTrayController.trayCloseBothLock();
        sliderTrayController.intakeStop();
        deliverPixel(true);

        //gotoPickupPos();
        gotoPickupPosWithSideSensor();


        dropPurplePushServo.setPushServoPushIn();
        sliderTrayController.setToIntakePos();
        //     sleep(100);
        //    chassis.stopRobot();
        //    sliderTrayController.setToIntakePos();
        sleep(300);
        dropPurplePushServo.setPushServoReady();
        sleep(300);
        dropPurplePushServo.setPushServoFullyInPos();
        sleep(300);
        dropPurplePushServo.resetPushServo();
        sleep(400);
        sliderTrayController.trayCloseBothLock();
        sliderTrayController.intakeStop();
        if (teamObjectPosition != TeamObjectPosition.LEFT){
            whiteDeliverPosition.y = whiteDeliverPosition.y - 0.03;
        }
        deliverPixel(true);


        dropPurplePushServo.resetPushServo();
        sliderTrayController.resetArmToPickupPos();
        chassis.driveRobot(0.4, 0.4, 0.4, 0.4);
        sleep(250);
        chassis.stopRobot();


    }

    public void moveToPixel(boolean sensorFlag){
        if (sensorFlag) {
            sleep(200);
            double leftOffset = 0;  // 20mm
            double leftDis = leftDisSensor.getDistance(DistanceUnit.MM) + leftOffset; //mm
            double rightDis = rightDisSensor.getDistance(DistanceUnit.MM);

            if (sensorFlag) {
                if (Math.abs(leftDis) < 400 && Math.abs(rightDis) < 400) {

                    double deltaDis = leftDis - rightDis;

                    int tryCnt = 0;
                    while (Math.abs(deltaDis) > 20 && tryCnt < 1) {  // only correct twice ? once? to save time
                        setLedGreen();
                        tryCnt++;
                        robotPos.setPosition(odometry.getRobotPosition());
                        if (deltaDis > 0) {
                            chassisController.p2pDrive(new RobotPosition(robotPos.x, robotPos.y + 0.04, FACING_TAG_ANGLE), 1.0, 1.0, 0, 0.01,
                                    2, 0.0, true, 1000);


                        } else {
                            chassisController.p2pDrive(new RobotPosition(robotPos.x, robotPos.y - 0.04, FACING_TAG_ANGLE), 1.0, 1.0, 0, 0.01,
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
        }

        // by timeout stop robot
        robotPos = odometry.getRobotPosition();
        chassisController.p2pDrive(new RobotPosition(robotPos.x - 0.2,robotPos.y,FACING_TAG_ANGLE),0.4,turnVel,0.4,0.02,1,0.4,true,300);
        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){
            sleep(10);
        }
        chassisController.stopAllThread();

    }


    private void deliverPixel(boolean whitePixel) {

        deliverPosition.setPosition(whiteDeliverPosition);

        chassisController.setP2PYPid(1.0,0.1,5.0);
        // offset center line a little bit - 0.03
        chassisController.p2pDrive(new RobotPosition(0.8, pickUpPos.y , FACING_TAG_ANGLE), targetVel, turnVel, 0.0, 0.1, 5, 0.2, false, 3000);
        boolean armNotUp = true;
        while (opModeIsActive() && !chassisController.p2pIsSettled()) {

            if (odometry.getRobotPosition().x > 0.3 && armNotUp) {
                armNotUp = false;
                if (!whitePixel) {
                    sliderTrayController.autoToDropLevelPos(firstDropLevel,90);
                }
                else{
                    sliderTrayController.autoToDropLevelPos(1,90);
                }

            }
            sleep(10);
        }
        chassisController.resetP2PPID();

        chassisController.p2pDrive(new RobotPosition(deliverPosition.x - 0.1, deliverPosition.y, FACING_TAG_ANGLE), targetVel, turnVel, 0.2, 0.02, 1, 0.0, true, 2000);

        while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {

            sleep(10);
        }
        sleep(50);
        chassisController.stopAllThread();

        chassis.driveRobot(-0.35, -0.35, -0.35, -0.35);
        timeOutTimer.reset();
        while(opModeIsActive() && sliderTrayController.touchBoardSensor.getState() && timeOutTimer.milliseconds() < 400){
            sleep(10);
        }

        chassis.stopRobot();
        sliderTrayController.trayOpenBothLock();
        sleep(300);




    }
    public void gotoPickupPos(){
        dropPurplePushServo.setPushServoReady();
        setLedRed();
        if (aprilTagTeamObjectDetector != null) {
            aprilTagTeamObjectDetector.stopAprilTagDetect();
        }

        //     sliderTrayController.setArmTiltAngle(sliderTrayController.getArmTiltAngle() + 20);
        //     sleep(500);
        chassis.driveRobot(0.3,0.3,0.3,0.3);
        sleep(300);
        chassis.stopRobot();
        sliderTrayController.resetArmToPickupPos();
        chassisController.p2pDrive(new RobotPosition(1.0, pickUpPos.y, FACING_TAG_ANGLE), 1.0, turnVel, 0.0, 0.1, 5, 0.0, false, 2000);

        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){

            sleep(10);
        }
        chassisController.resetP2PPID();
        sliderTrayController.intakeIn();
        dropPurplePushServo.setPushServoReady();

        chassisController.setP2PYPid(1.5,MecanumDriveLib.P2PY_KI,MecanumDriveLib.P2PY_KD);

        chassisController.p2pDrive(new RobotPosition(pickUpPos.x + 0.00,pickUpPos.y + 0.0,FACING_TAG_ANGLE),targetVel,turnVel,0.2,0.02,1,0.0,true,5000);


        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){
            sleep(10);
        }

        waitForTouch();
        chassisController.resetP2PPID();
        robotPos = odometry.getRobotPosition();
        // saveToFile("PickUpPos.txt", new double[]{robotPos.x, robotPos.y});

    }

    public void gotoPickupPosWithSideSensor(){
        dropPurplePushServo.setPushServoReady();
        setLedRed();
        if (aprilTagTeamObjectDetector != null) {
            aprilTagTeamObjectDetector.stopAprilTagDetect();
        }

        //     sliderTrayController.setArmTiltAngle(sliderTrayController.getArmTiltAngle() + 20);
        //     sleep(500);
        chassis.driveRobot(0.3,0.3,0.3,0.3);
        sleep(300);
        chassis.stopRobot();
        sliderTrayController.resetArmToPickupPos();
        chassisController.p2pDrive(new RobotPosition(1.0, pickUpPos.y, FACING_TAG_ANGLE), 1.0, turnVel, 0.0, 0.1, 5, 0.0, false, 2000);

        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){

            sleep(10);
        }
        chassisController.resetP2PPID();
        sliderTrayController.intakeIn();
        dropPurplePushServo.setPushServoReady();

        chassisController.setP2PYPid(1.5,MecanumDriveLib.P2PY_KI,MecanumDriveLib.P2PY_KD);

        chassisController.p2pDrive(new RobotPosition(-0.3,pickUpPos.y + 0.0,FACING_TAG_ANGLE),targetVel,turnVel,0.0,0.02,1,0.4,false,5000);
        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){
            sleep(10);
        }
        detectBar();
        chassisController.p2pDrive(new RobotPosition(-0.9,pickUpPos.y + 0.0,FACING_TAG_ANGLE),0.4,turnVel,0.4,0.02,1,0.4,false,5000);
        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){
            sleep(10);
        }
        modifyOdometry();
        chassisController.p2pDrive(new RobotPosition(pickUpPos.x,pickUpPos.y + 0.0,FACING_TAG_ANGLE),targetVel,turnVel,0.4,0.02,1,0.0,true,5000);
        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){
            sleep(10);
        }

        chassisController.resetP2PPID();
        robotPos = odometry.getRobotPosition();
        // saveToFile("PickUpPos.txt", new double[]{robotPos.x, robotPos.y}

        moveToPixel(true);




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


    private boolean modifyOrNot = false;
    boolean record = true;
    int recordNum = 100;
    double minDis = 8192;
    private double[] recordDis = new double[100];
    private final double SIDE_SENSOR_DIS = 120 ;  // mm

    private double odometryDeltaY = 0;
    private void detectBar(){

        new Thread(new Runnable() {


            @Override
            public void run() {
                int cnt = 0;

                minDis = 8192;
                modifyOrNot = false;
                odometryDeltaY = 0;
                while(cnt < recordNum){
                    //    recordDis1[cnt] = leftDisSensor[0].getDistance(DistanceUnit.MM);
                    //     recordDis2[cnt] = leftDisSensor[1].getDistance(DistanceUnit.MM);
                    robotPos = odometry.getRobotPosition();
                    if (teamColor == AllianceColor.RED) {
                        recordDis[cnt] = leftSideDisSensor.getDistance(DistanceUnit.MM);
                    }
                    else{
                        recordDis[cnt] = rightSideDisSensor.getDistance(DistanceUnit.MM);
                    }
                    if (recordDis[cnt] > 40 && recordDis[cnt] < 200) {
                        if (recordDis[cnt] < minDis) {
                            minDis = recordDis[cnt];

                            //compare the odometry reading
                            odometryDeltaY = (pickUpPos.y - robotPos.y) - (minDis - SIDE_SENSOR_DIS) / 1000;
                            //odometryDeltaY =  - (minDis - 120) / 1000;
                            modifyOrNot = true;


                        }
                    }



                    cnt ++;
                }

                record = false;
            }
        }).start();


    }


    public void modifyOdometry(){
        if (modifyOrNot){
            odometry.modifyY(odometryDeltaY);
        }
    }



}
