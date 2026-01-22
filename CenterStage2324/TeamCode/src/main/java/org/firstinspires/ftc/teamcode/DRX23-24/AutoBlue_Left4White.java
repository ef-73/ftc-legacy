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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;


@Autonomous(name="BlueLeft4White", preselectTeleOp = "TeleOpTest")
//@Disabled
//@Config
public class AutoBlue_Left4White extends LinearOpMode {

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


    //blue side
    private AllianceColor teamColor = AllianceColor.BLUE;
    private final double FACING_TAG_ANGLE = -180;
    RobotPosition startPosition = new RobotPosition(0.3,1.60 - 0.04,-90);  // start position for red side  1.57

    RobotPosition rightDropPosition = new RobotPosition(startPosition.x - 0.03, startPosition.y - 0.52, startPosition.angle);
    RobotPosition leftDropPosition = new RobotPosition(startPosition.x + 0.53, startPosition.y - 0.51, startPosition.angle);
    RobotPosition middleDropPosition = new RobotPosition(startPosition.x + 0.39, startPosition.y - 0.80, startPosition.angle);

    RobotPosition whitePilePosition = new RobotPosition(-1.47, 0.28, FACING_TAG_ANGLE);

    RobotPosition leftTagPos = new RobotPosition(1.55 - 0.4,1.05 + 0.02, FACING_TAG_ANGLE); //tag4
    RobotPosition middleTagPos = new RobotPosition(1.55 - 0.4,0.90 + 0.015 , FACING_TAG_ANGLE); //tag5
    RobotPosition rightTagPos = new RobotPosition(1.55 - 0.4,0.75 + 0.02 , FACING_TAG_ANGLE); // tag6  1.04

    RobotPosition parkPosition = new RobotPosition(1.55 - 0.4, 1.56, FACING_TAG_ANGLE);


    RobotPosition camPos = new RobotPosition(0,0,0);



    RobotPosition yellowDeliverPosition = new RobotPosition(0,0,0);
    RobotPosition whiteDeliverPosition = new RobotPosition(0,0,0);

    RobotPosition deliverPosition = new RobotPosition(0,0,0);

    TeamObjectPosition teamObjectPosition = TeamObjectPosition.NULL;
    boolean deliverWhite = false;


    DistanceSensor leftDisSensor = null;
    DistanceSensor rightDisSensor = null;

    double costTime = 0;

    // for blue side
    //RobotPosition startPosition = new RobotPosition(-0.9,1.57,-90);  // start position for blue side
    // for red side

    int moveStep = 0;

    @Override
    public void runOpMode(){
        auto_init();
/////////////////////////////////////////////////////////////////////////////////
        waitForStart();

        runTime.reset();
        aprilTagTeamObjectDetector.startTeamObjectDetection();
        odometry.setRobotPosition( startPosition);  // in case move robot
        dropPurplePushServo.resetPushServo();
        sliderTrayController.startResetCheck();
        sliderTrayController.trayCloseBothLock();

        if (aprilTagTeamObjectDetector != null){

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


            if (teamObjectPosition == TeamObjectPosition.RIGHT) {
                whitePilePosition.y = whitePilePosition.y - 0.00;
                whitePilePosition.x = whitePilePosition.x + 0.00;
                pickUpPos.setPosition(whitePilePosition);
                yellowDeliverPosition.setPosition(rightTagPos);
                middleDropPosition.y = middleTagPos.y - YELLOW_POS_OFFSET;
                whiteDeliverPosition.setPosition(middleTagPos);

                // move to right

                chassisController.p2pDrive(rightDropPosition, 1.0, 0.3, 0, 0.02, 2, 0, true, 2000);
                moveStep = 0;
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    if (odometry.getRobotPosition().y < 1.3 && moveStep == 0){
                        moveStep = 1;
                        sliderTrayController.autoToDropLevelPos(0, 90);
                    }
                    sleep(10);
                }
                // drop preload here
                dropPurplePushServo.dropPurple();
                sleep(400);
                dropYellowPixel();
                pickWhitePixel();

                costTime = runTime.milliseconds();


            } else if (teamObjectPosition == TeamObjectPosition.MIDDLE) {
                // move to middle
                whitePilePosition.y = whitePilePosition.y - 0.00;
                whitePilePosition.x = whitePilePosition.x + 0.00;
                pickUpPos.setPosition(whitePilePosition);
                yellowDeliverPosition.setPosition(middleTagPos);

                rightTagPos.y = rightTagPos.y + 0.0;
                whiteDeliverPosition.setPosition(rightTagPos);

                chassisController.p2pDrive(middleDropPosition, 1.0, 1.0, 0, 0.02, 2, 0, true, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    if (odometry.getRobotPosition().y < 1.3 && moveStep == 0){
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

            } else if (teamObjectPosition == TeamObjectPosition.LEFT) {
                // move to right
                whitePilePosition.y = whitePilePosition.y - 0.00;
                whitePilePosition.x = whitePilePosition.x + 0.00;
                pickUpPos.setPosition(whitePilePosition);
                yellowDeliverPosition.setPosition(leftTagPos);

                whiteDeliverPosition.setPosition(middleTagPos);
                chassisController.resetP2PPID();

                chassisController.p2pDrive(new RobotPosition(startPosition.x + 0.54, startPosition.y - 0.45, startPosition.angle), 1.0, 1.0, 0, 0.04, 4, 0, false, 2000);
                moveStep = 0;
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    if (odometry.getRobotPosition().y < 1.3 && moveStep == 0){
                        moveStep = 1;
                        sliderTrayController.autoToDropLevelPos(0,-90);
                    }
                    sleep(10);
                }


                chassisController.p2pDrive(leftDropPosition, 1.0, 1.0, 0, 0.01, 2, 0, true, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    sleep(10);
                }

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
        deliverPosition.setPosition(middleTagPos);
        chassisController.setP2PTurnPID(MecanumDriveLib.turnKp, MecanumDriveLib.turnKi,MecanumDriveLib.turnKd);
        chassisController.p2pDrive(new RobotPosition(deliverPosition.x - 0.13 , deliverPosition.y, FACING_TAG_ANGLE), 1.0, 0.3, 0, 0.02, 2, 0, true, 5000);
        moveStep = 0;
        while (opModeIsActive() && !chassisController.p2pIsSettled()) {
            sleep(10);
        }

        chassisController.resetP2PPID();
        if (aprilTagTeamObjectDetector != null) {
            sleep(400);
            if (aprilTagTeamObjectDetector.getTagIDOrNot()) {
                robotPos.setPosition(odometry.getRobotPosition());
                camPos.setPosition(aprilTagTeamObjectDetector.getRobotPosition());
                if (camPos.dis2Pos(robotPos) < 0.2) {  // make sure camPos is right
                    double deltaX = camPos.x - robotPos.x;
                    double deltaY = camPos.y - robotPos.y;
                    // saveToFile("CamPos.txt", new double[]{deltaX,deltaY});
                    pickUpPos.x = pickUpPos.x + deltaX;
                    pickUpPos.y = pickUpPos.y + deltaY;
                    odometry.setRobotPositionXY(camPos);
                    setLedGreen();
                }
            }
        }


        chassisController.resetP2PPID();
        chassisController.p2pDrive(new RobotPosition(yellowDeliverPosition.x, yellowDeliverPosition.y, FACING_TAG_ANGLE), targetVel, turnVel, 0.2, 0.02, 1, 0.0, true, 1000);
        while (opModeIsActive() && !chassisController.p2pIsSettled()) {
            sleep(10);
        }


        dropPurplePushServo.resetDropServo();
        chassisController.stopAllThread();

        chassis.driveRobot(-0.35, -0.35, -0.35, -0.35);
        timeOutTimer.reset();
        while(opModeIsActive() && sliderTrayController.touchBoardSensor.getState() && timeOutTimer.milliseconds() < 300){
            chassis.driveRobot(-0.35, -0.35, -0.35, -0.35);
            sleep(10);
        }

        chassis.stopRobot();

        sliderTrayController.trayOpenBothLock();
        sleep(300);
        chassis.stopRobot();
        dropPurplePushServo.resetPushServo();

    }


    public void pickWhitePixel(){
        gotoPickupPos();
        sliderTrayController.intakeIn();
        dropPurplePushServo.setPushServoReady();
        moveToPixel(0.35,true );

        dropPurplePushServo.setPushServoPushIn();
        sliderTrayController.setToIntakePos();
    //    sleep(100);
   //     chassis.stopRobot();
    //    sliderTrayController.setToIntakePos();
        sleep(400);
        dropPurplePushServo.setPushServoReady();
        sleep(300);
        dropPurplePushServo.setPushServoFullyInPos();
        sleep(400);
        dropPurplePushServo.resetPushServo();
        sleep(300);

        if (teamObjectPosition == TeamObjectPosition.LEFT){
            whiteDeliverPosition.y = whiteDeliverPosition.y - 0.02;
        }else if(teamObjectPosition == TeamObjectPosition.MIDDLE){
            whiteDeliverPosition.y = whiteDeliverPosition.y - 0.02;
        }else{
            whiteDeliverPosition.y = whiteDeliverPosition.y + 0.04;
        }

        if (teamObjectPosition == TeamObjectPosition.MIDDLE){
            deliverPixel(true, 0, 1);
        }else {
            deliverPixel(true, 90, 2);
        }

        gotoPickupPos();

        moveToPixel(0.35,true );

        dropPurplePushServo.setPushServoPushIn();
        sliderTrayController.setToIntakePos();
   //     sleep(100);
    //    chassis.stopRobot();
    //    sliderTrayController.setToIntakePos();
        sleep(400);
        dropPurplePushServo.setPushServoReady();
        sleep(300);
        dropPurplePushServo.setPushServoFullyInPos();
        sleep(400);
        dropPurplePushServo.resetPushServo();
        sleep(300);


        if (teamObjectPosition == TeamObjectPosition.LEFT){
            whiteDeliverPosition.y = whiteDeliverPosition.y - 0.04;
        }else if(teamObjectPosition == TeamObjectPosition.MIDDLE){
            whiteDeliverPosition.y = whiteDeliverPosition.y - 0.00;
        }else{
            whiteDeliverPosition.y = whiteDeliverPosition.y + 0.02;
        }


        if (teamObjectPosition == TeamObjectPosition.MIDDLE){
            deliverPixel(true, 10, 1);
        }
        else if (teamObjectPosition == TeamObjectPosition.LEFT){
            deliverPixel(true, 30, 1);
        }
        else{
            deliverPixel(true, 30, 1);
        }

        dropPurplePushServo.resetPushServo();
        chassis.driveRobot(0.4, 0.4, 0.4, 0.4);
        sleep(150);
        sliderTrayController.resetArmToPickupPos();
        sleep(50);
        chassis.stopRobot();


    }

    public void moveToPixel(double pwr, boolean sensorFlag){
        setLedGreen();
        if(sensorFlag) {
            sleep(100);
            double leftOffset = 0;  // 20mm
            //average ????????? 2 times
            double leftDis = leftDisSensor.getDistance(DistanceUnit.MM) + leftOffset; //mm
            double rightDis = rightDisSensor.getDistance(DistanceUnit.MM);
            sleep(10);
            leftDis = 0.3 * leftDis + 0.7 * (leftDisSensor.getDistance(DistanceUnit.MM) + leftOffset); //mm
            rightDis = 0.3 * rightDis + 0.7 * (rightDisSensor.getDistance(DistanceUnit.MM));

                if (Math.abs(leftDis) < 400 && Math.abs(rightDis) < 400) {

                    double deltaDis = leftDis - rightDis;

                    int tryCnt = 0;
                    while (Math.abs(deltaDis) > 20 && tryCnt < 1) {  // only correct twice ? once? to save time
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
            chassisController.stopAllThread();
            pickUpPos.setPosition(odometry.getRobotPosition());
        }
        // by timeout stop robot
        /*
        robotPos = odometry.getRobotPosition();
        chassisController.p2pDrive(new RobotPosition(robotPos.x - 0.2,robotPos.y,FACING_TAG_ANGLE),pwr,turnVel,0.4,0.02,1,0.4,true,300);
        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){
            sleep(10);
        }*/
        chassisController.stopAllThread();
        chassis.driveRobot(pwr, pwr,pwr,pwr);
        timeOutTimer.reset();
        double  minDis = Math.min(leftDisSensor.getDistance(DistanceUnit.MM), rightDisSensor.getDistance(DistanceUnit.MM));
        while(opModeIsActive() && timeOutTimer.milliseconds() < 450 && minDis > 50){
            chassis.driveRobot(pwr, pwr,pwr,pwr);
            sleep(10);
            minDis = Math.min(leftDisSensor.getDistance(DistanceUnit.MM), rightDisSensor.getDistance(DistanceUnit.MM));
            if (minDis <= 50){
                setLedRed();
            }

        }
        chassis.stopRobot();

    }


    private void deliverPixel(boolean whitePixel, int angle, int dropLevel) {

       deliverPosition.setPosition(whiteDeliverPosition);

        chassisController.setP2PYPid(1.0,0.1,5.0);
        // offset center line a little bit - 0.03
        chassisController.p2pDrive(new RobotPosition(0.8, pickUpPos.y - 0.03, FACING_TAG_ANGLE), targetVel, turnVel, 0.0, 0.1, 5, 0.2, false, 3000);
        boolean armNotUp = true;
        while (opModeIsActive() && !chassisController.p2pIsSettled()) {

            if (odometry.getRobotPosition().x > 0.3 && armNotUp) {
                sliderTrayController.autoToDropLevelPos(dropLevel, angle);
            }
            sleep(10);
        }
        chassisController.resetP2PPID();

        chassisController.p2pDrive(new RobotPosition(deliverPosition.x - 0.13, deliverPosition.y, FACING_TAG_ANGLE), targetVel, turnVel, 0.2, 0.02, 1, 0.0, true, 2000);

        while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {

            sleep(10);
        }
        sleep(50);
        chassisController.stopAllThread();
        chassisController.stopRobot();

        chassis.driveRobot(-0.42, -0.42, -0.42, -0.42);
        timeOutTimer.reset();
        while(opModeIsActive() && sliderTrayController.touchBoardSensor.getState() && timeOutTimer.milliseconds() < 350){
            chassis.driveRobot(-0.42, -0.42, -0.42, -0.42);
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

        chassisController.p2pDrive(new RobotPosition(1.0, pickUpPos.y, FACING_TAG_ANGLE), 1.0, turnVel, 0.0, 0.1, 5, 0.0, false, 2000);

        timeOutTimer.reset();
        boolean firstMove = false;
        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){
            if (timeOutTimer.milliseconds() > 200 && !firstMove){
                firstMove = true;
                sliderTrayController.resetArmToPickupPos();
            }
            sleep(10);
        }
        sliderTrayController.intakeIn();
        dropPurplePushServo.setPushServoReady();
        chassisController.resetP2PPID();


        chassisController.setP2PYPid(1.5,MecanumDriveLib.P2PY_KI,MecanumDriveLib.P2PY_KD);

        chassisController.p2pDrive(new RobotPosition(pickUpPos.x + 0.00,pickUpPos.y + 0.0,FACING_TAG_ANGLE),targetVel,turnVel,0.2,0.02,4,0.0,true,5000);


        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){
            sleep(10);
        }

        chassisController.resetP2PPID();
        robotPos = odometry.getRobotPosition();
        // saveToFile("PickUpPos.txt", new double[]{robotPos.x, robotPos.y});

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
            if (aprilTagTeamObjectDetector != null) {
                RobotPosition camPosition = aprilTagTeamObjectDetector.getRobotPosition();
                telemetry.addLine(String.format("Cam Pos %3.3f, %3.3f, %3.3f", camPosition.x, camPosition.y, camPosition.angle));
            }
            telemetry.update();
            sleep(10);
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
