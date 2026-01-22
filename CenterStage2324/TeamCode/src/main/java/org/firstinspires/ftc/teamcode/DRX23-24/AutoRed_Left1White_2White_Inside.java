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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.List;


@Autonomous(name="Red_Left3White_Inside")
//@Disabled
@Config
public class AutoRed_Left1White_2White_Inside extends LinearOpMode {

    private boolean oneWhite = false;
    private int waitTime = 8000;

    private int firstDropLevel = 2;
    private final double YELLOW_POS_OFFSET =  0.035 ;//


    ElapsedTime runTime = new ElapsedTime();
    ElapsedTime timeOutTimer = new ElapsedTime();


    public DigitalChannel redLed = null;
    public DigitalChannel greenLed = null;
    private AirPlaneLaunch airPlaneLaunch = null;

    private DropPurple_PushServo autoDropPurple = null;

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
    RobotPosition leftDropPosition = new RobotPosition(startPosition.x - 0.51, startPosition.y + 0.51, startPosition.angle );
    RobotPosition rightDropPosition = new RobotPosition(startPosition.x + 0.03, startPosition.y + 0.5, startPosition.angle);
    RobotPosition middleDropPosition = new RobotPosition(startPosition.x - 0.06, startPosition.y + 0.71 - 0.04, startPosition.angle + Math.signum(startPosition.angle) * 68);


    RobotPosition whitePilePosition = new RobotPosition(-1.45, -0.905, FACING_TAG_ANGLE);  // y=-0.27

    RobotPosition leftTagPos = new RobotPosition(1.55 -0.4,-0.74 , FACING_TAG_ANGLE); //tag4
    RobotPosition middleTagPos = new RobotPosition(1.55 - 0.4,-0.90 , FACING_TAG_ANGLE); //tag5
    RobotPosition rightTagPos = new RobotPosition(1.55 - 0.4,-1.06  , FACING_TAG_ANGLE); // tag6  1.04


    RobotPosition camPos = new RobotPosition(0,0,0);

    RobotPosition yellowDeliverPosition = new RobotPosition(0,0,0);

    RobotPosition whiteDeliverPosition = new RobotPosition(0,0,0);

    RobotPosition deliverPosition = new RobotPosition(0,0,0);
    RobotPosition parkPosition = new RobotPosition(1.55 - 0.41, -1.54, FACING_TAG_ANGLE);

    TeamObjectPosition teamObjectPosition = TeamObjectPosition.NULL;
    boolean deliverWhite = false;


    DistanceSensor leftDisSensor = null;
    DistanceSensor rightDisSensor = null;

    double costTime = 0;


    int moveStep = 0;

    @Override
    public void runOpMode(){
        initLed();

        leftDisSensor = hardwareMap.get(DistanceSensor.class, "leftDis");
        rightDisSensor = hardwareMap.get(DistanceSensor.class, "rightDis");

        airPlaneLaunch = new AirPlaneLaunch(this);
        autoDropPurple = new DropPurple_PushServo(this, true, DropPurple_PushServo.ArmLeftRight.RIGHT);
        // could save time for reading encoder
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        chassis = new ChassisSystem(this);

        sleep(1000);
        telemetry.addLine("odometry is setup");
        telemetry.update();

        odometry = new Odometry(this,chassis, startPosition, true);


        sleep(1000);
        chassisController = new MecanumDriveLib(this, odometry,chassis);
        sliderTrayController = new SliderAndTraySystem_Servo(this, chassis,true);
        aprilTagTeamObjectDetector = new AprilTagTeamObjectDetector(this, teamColor, odometry, "WebcamFront", "WebcamBack");

        sleep(1000);


        // set start position

        odometry.setRobotPosition( startPosition);
        sleep(2000);

        telemetry.addData("Working Mode", "waiting for start");
        telemetry.update();
/////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        aprilTagTeamObjectDetector.startTeamObjectDetection();
        odometry.setRobotPosition( startPosition);  // in case move robot
        sliderTrayController.enableTrayServo();
        autoDropPurple.resetPushServo();
        sliderTrayController.startResetCheck();
        sliderTrayController.trayOpenBothLock();
        runTime.reset();
        aprilTagTeamObjectDetector.resetTeamObjectDetectCnt();
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
                rightTagPos.y = rightTagPos.y + 0.11;  // move to right, outside
                yellowDeliverPosition.setPosition(rightTagPos);
                whiteDeliverPosition.setPosition(middleTagPos);



                chassisController.p2pDrive(rightDropPosition, 1.0, 0.3, 0, 0.02, 2, 0, true, 2000);
                moveStep = 0;
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    sleep(10);
                }
                // drop preload here
                autoDropPurple.dropPurple();
                autoDropPurple.setPushServoReady();
                sleep(400);


                autoDropPurple.setPushServoReady();
                //move to pile
                chassisController.p2pDrive(whitePilePosition, 1.0, 1.0, 0, 0.01, 1, 0, true, 3000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {

                    sleep(10);
                }
                autoDropPurple.resetDropServo();
                pickDeliverPixel();

                costTime = runTime.milliseconds();


            } else if (teamObjectPosition == TeamObjectPosition.MIDDLE) {
                // move to middle

                runTime.reset();
                middleTagPos.y = middleTagPos.y - 0.06; // move close to rightTag
                yellowDeliverPosition.setPosition(middleTagPos);
                whiteDeliverPosition.setPosition(rightTagPos);

                chassisController.p2pDrive(new RobotPosition(middleDropPosition.x - 0.1, middleDropPosition.y - 0.3, middleDropPosition.angle), 1.0, 1.0, 0, 0.05, 5, 1.0, false, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {

                    sleep(10);
                }

                chassisController.p2pDrive(middleDropPosition, 1.0, 1.0, 0, 0.02, 2, 0, true, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {

                    sleep(10);
                }
                autoDropPurple.setPushServoReady();
                autoDropPurple.dropPurple();
                autoDropPurple.setPushServoReady();
                sleep(400);

                //move to pile
                //chassisController.setP2PXPid(1.0, 0.1, 8);
                //chassisController.setP2PYPid(1.5, 0.1, 5);
                chassisController.resetP2PPID();
                chassisController.p2pDrive(new RobotPosition(whitePilePosition.x, whitePilePosition.y - 0.0, whitePilePosition.angle), 1.0, 1.0, 0, 0.01, 1, 0, true, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    sleep(10);
                }
                autoDropPurple.resetDropServo();
                pickDeliverPixel();
                costTime = runTime.milliseconds();

            } else if (teamObjectPosition == TeamObjectPosition.LEFT) {
                // move to left
                runTime.reset();
                leftTagPos.y = leftTagPos.y - 0.06; // move a little bit to outside
                yellowDeliverPosition.setPosition(leftTagPos);
                whiteDeliverPosition.setPosition(middleTagPos);



                chassisController.resetP2PPID();

                chassisController.p2pDrive(leftDropPosition, 1.0, 1.0, 0, 0.01, 2, 0, true, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    sleep(10);
                }
                //drop cone here
                autoDropPurple.dropPurple();
                sleep(400);

                chassisController.turnToAngle(FACING_TAG_ANGLE, 1.0,MecanumDriveLib.TURN_WHEEL.WHEELS_TURN,1.0,true,1500);
                while (opModeIsActive() && !gamepad1.back && !chassisController.turnIsSettled()) {
                    sleep(10);
                }

                chassisController.p2pDrive(new RobotPosition(whitePilePosition.x , whitePilePosition.y, whitePilePosition.angle), 1.0, 1.0, 0, 0.01, 2, 0.0, false, 2000);
                while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
                    sleep(10);
                }
                autoDropPurple.resetDropServo();


                autoDropPurple.setPushServoReady();
                sleep(100);
                chassisController.resetP2PPID();

                pickDeliverPixel();

                costTime = runTime.milliseconds();
            }
        }
        chassis.stopRobot();
        while(opModeIsActive()){
            telemetry.addLine(String.format("Time = %3.3f", costTime/ 1000.0));
            telemetry.update();
            sleep(10);
        }
        chassis.stopRobot();

        odometry.stopThread();
        if (aprilTagTeamObjectDetector != null){
            aprilTagTeamObjectDetector.stopAprilTagDetect();
            aprilTagTeamObjectDetector.closeDetector();
        }


    }



    public void moveToPixel(double pwr,boolean sensorFlag){
        setLedGreen();
        if (sensorFlag) {
            sleep(200);
            double leftOffset = 0;  // 20mm
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
            }
            chassisController.stopAllThread();

            sleep(100);
        }
        robotPos = odometry.getRobotPosition();
        if (!sensorFlag) {
            pickUpPos.setPosition(robotPos);  // remember current position as next pick up position
        }
        sliderTrayController.setToIntakePos();

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


    private void deliverPixel(boolean whitePixel) {
        if (!whitePixel) {
            deliverPosition.setPosition(middleTagPos);
        }
        else{
            deliverPosition.setPosition(parkPosition);
        }

        chassisController.p2pDrive(new RobotPosition(pickUpPos.x, pickUpPos.y, FACING_TAG_ANGLE), 0.4, 1.0, 0, 0.01,
                2, 0.0, true, 1000);

        while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
            sleep(10);
        }

        chassisController.p2pDrive(new RobotPosition(pickUpPos.x, pickUpPos.y - 0.57, FACING_TAG_ANGLE), 1.0, 1.0, 0, 0.01,
                2, 0.0, false, 2000);

        while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {
            sleep(10);
        }

        chassisController.setP2PYPid(1.0,0.1,5.0);
        // offset center line a little bit - 0.03
        chassisController.p2pDrive(new RobotPosition(0.8, pickUpPos.y - 0.59 , FACING_TAG_ANGLE), targetVel, turnVel, 0.0, 0.1,
                5, 0.2, false, 3000);
        boolean armNotUp = true;
        while (opModeIsActive() && !chassisController.p2pIsSettled()) {

            if (odometry.getRobotPosition().x > 0.1 && armNotUp) {
                armNotUp = false;
                if (!whitePixel) {
                    if (teamObjectPosition == TeamObjectPosition.RIGHT){
                        sliderTrayController.autoToDropLevelPos(2,90);
                    }else{
                        sliderTrayController.autoToDropLevelPos(2,-90);
                    }

                }
                else{
                    sliderTrayController.autoToDropLevelPos(1,90);
                }

            }
            sleep(10);
        }
        sleep(50);
        chassisController.resetP2PPID();
        if (!whitePixel) {
            chassisController.p2pDrive(new RobotPosition(deliverPosition.x - 0.14, deliverPosition.y, FACING_TAG_ANGLE), targetVel, turnVel, 0.2, 0.02, 1, 0.0, true, 2500);
        }
        else{
            chassisController.p2pDrive(new RobotPosition(deliverPosition.x, deliverPosition.y, FACING_TAG_ANGLE), targetVel, turnVel, 0.2, 0.02, 1, 0.0, true, 2500);
        }
        while (opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()) {

            sleep(10);
        }

        if (!whitePixel) {
            sleep(400);

            if (aprilTagTeamObjectDetector.getTagIDOrNot()) {
                robotPos.setPosition(odometry.getRobotPosition());
                camPos.setPosition(aprilTagTeamObjectDetector.getRobotPosition());
                if (camPos.dis2Pos(robotPos) < 0.15) {  // make sure camPos is right
                    //if (camPos.dis2Pos(deliverPosition) < 0.3) {

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
                    //}

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

        if (!whitePixel) {
            chassis.driveRobot(-0.35, -0.35, -0.35, -0.35); // push against board
            timeOutTimer.reset();
            while (opModeIsActive() && timeOutTimer.milliseconds() < 300){
                chassis.driveRobot(-0.35, -0.35, -0.35, -0.35); // push against board
                sleep(10);
            }

            chassis.stopRobot();

            sliderTrayController.trayOpenBothLock();
            sleep(300);  // drop first white picked from pile
        }
        else{

            sliderTrayController.trayOpenBothLock();
            sleep(300);

            autoDropPurple.resetPushServo();
            sliderTrayController.resetArmToPickupPos();



        }



    }
    public void gotoPickupPos(){

        setLedRed();
        aprilTagTeamObjectDetector.stopAprilTagDetect();

   //     sliderTrayController.setArmTiltAngle(sliderTrayController.getArmTiltAngle() + 20);
   //     sleep(500);
       // chassis.driveRobot(0.3,0.3,0.3,0.3);
       // sleep(300);
       // chassis.stopRobot();

        chassisController.p2pDrive(new RobotPosition(1.0, pickUpPos.y - 0.6, FACING_TAG_ANGLE), 1.0, turnVel, 0.0, 0.1, 5, 0.0, false, 2000);
        timeOutTimer.reset();
        boolean resetFlag = false;
        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){
            if (timeOutTimer.milliseconds() > 200 && !resetFlag){
                sliderTrayController.resetArmToPickupPos();
                resetFlag = true;
            }
            sleep(10);
        }
        chassisController.resetP2PPID();
       // robotPos = odometry.getRobotPosition();
       // saveToFile("middlePos.txt", new double[]{robotPos.x, robotPos.y});

     //   chassisController.setP2PYPid(1.5,0.1,5);
     //   chassisController.setP2PYPid(2.0,0.1,8);

        chassisController.setP2PYPid(2.0,MecanumDriveLib.P2PY_KI,MecanumDriveLib.P2PY_KD);
        chassisController.p2pDrive(new RobotPosition(pickUpPos.x + 0.02,pickUpPos.y - 0.57,FACING_TAG_ANGLE),targetVel,turnVel,0.0,0.02,1,0.0,false,5000);

        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){
            sleep(10);
        }


        chassisController.resetP2PPID();

        chassisController.p2pDrive(new RobotPosition(pickUpPos.x + 0.02,pickUpPos.y + 0.01,FACING_TAG_ANGLE),0.75,turnVel,0.0,0.02,2,0.0,false,5000);

        while(opModeIsActive() && !gamepad1.back && !chassisController.p2pIsSettled()){
            sleep(10);
        }
        autoDropPurple.setPushServoReady();
        robotPos = odometry.getRobotPosition();
       // saveToFile("PickUpPos.txt", new double[]{robotPos.x, robotPos.y});

    }

    public void pickDeliverPixel(){
        sliderTrayController.intakeIn();
        moveToPixel(0.32, false);

        if (!oneWhite) {
            sliderTrayController.setToIntakePos();
            autoDropPurple.setPushServoPushIn();
            sleep(400);
            autoDropPurple.resetPushServo();
            sleep(300);


            deliverPixel(false);
            autoDropPurple.setPushServoReady();


            gotoPickupPos();

            sliderTrayController.intakeIn();
            autoDropPurple.setPushServoReady();
            moveToPixel(0.35, true);

            autoDropPurple.setPushServoPushIn();
            sleep(500);
            autoDropPurple.setPushServoReady();
            sleep(400);
            autoDropPurple.setPushServoPushIn();
            sleep(800);

            autoDropPurple.resetPushServo();
            sleep(400);

            if (teamObjectPosition == TeamObjectPosition.MIDDLE) {
                whiteDeliverPosition.y = whiteDeliverPosition.y + 0.03;
            }


            deliverPixel(true);
            autoDropPurple.resetPushServo();
        }else{
            sliderTrayController.setToIntakePos();
            autoDropPurple.setPushServoPushIn();
            sleep(500);
            autoDropPurple.resetPushServo();
            sleep(800);

            sliderTrayController.intakeStop();
            sleep(waitTime);

            deliverPixel(false);
            // reset arm
            autoDropPurple.resetPushServo();
            chassis.driveRobot(0.4, 0.4, 0.4, 0.4);
            sleep(150);
            sliderTrayController.resetArmToPickupPos();
            sleep(50);
            chassis.stopRobot();
        }
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


}
