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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.List;


@Autonomous(name="Auto Substation High Pole")
//@Disabled

public class AutoSubStationHigh extends LinearOpMode {
    private double HPICKUP_5_LEN = 0.73 + 0.02;
    private double HPICKUP_PAN_ANGLE = -2.8;

    private double[] hPickup5Tilt1Angle = {-51.31 - 4,-39.71 - 4.5,-25.38 - 5.0,-13.47 - 5.0,2.43 + 0.0};
    private double[] hPickup5Tilt2Angle = {-8.01 + 4,-15.58 + 4.5,-29.88 + 5.0,-41.59 + 5.0,-60.83 - 2.0};
    private double sensorDetectTilt1 = - 12.69;
    private double sensorDetectTilt2 = -54.09;
    private double vSlideStopHeight = 0.3;

    private int sideChoice = 1;  // 1 -- left, -1 -- right
    private ElapsedTime timeOut = new ElapsedTime();
    AprilTagSignalDetect cameraObjectDetector = null;
    ChassisSystem chassis = null;
    Odometry odometry = null;
    MecanumDriveLib chassisController = null;

    HSliderSystem hSliderSystem = null;
    VSliderSystem vSliderSystem = null;
    RobotPosition robotPos = new RobotPosition(0,0,0);

    RobotPosition parkPosition = new RobotPosition(0,0,0);
    // the start position and final target position
    private static double STARTX = -1.52;
    private static double STARTY = 0.9 ;  // middle of tile
    private static double STARTA = 0;

    public static double targetX = STARTX + 1.25;
    public static double targetY = STARTY - 0.48 ;
    public static double targetA = -90;
    public static double targetVel = 1.0;
    public static double turnVel = 1.0;



    //for pick up


    //handover

    private static final double HSLIDER_HANDOVER_LEN = 0.097;   //0.01

    private static final double HHANDOVER_TILT1_ANGLE = 52.47 ;//44.82;
    private static final double HHANDOVER_TILT2_ANGLE = 66.5 ;//67.32; //53.64;


    private static final double HHANDOVER_PAN_ANGLE = 3.4;

    private static final double VSLIDER_HANDOVER_LEN = 0.002; //0.139
    private static final double VHANDOVER_TILT_ANGLE = 2.8;
    private static final double VHANDOVER_PAN_ANGLE = 0;
    private static final double VHANDOVER_ROTATE_ANGLE = 0.0;
    private static final double VHANDOVER_CLAW_ANGLE = 35;  // 45



    private static final double VSLIDELEN_LPOLE = 0.10;  // for low pole

    private static final double VSLIDELEN_MPOLE = 0.18;  // for medium pole
    private static final double VSLIDELEN_HPOLE = 0.42 + 0.02;  // 0.44  for high pole

    private static final double VSLIDE_TILTANGLE = 5.5;  // drop tilt angle



    private static final double HSLIDEPICKUPLEN_1TILE = 0.24;
    private static final double HSLIDEPICKUPLEN_2TILES = 0.84;
    private static final double HSLIDEPICKUPLEN_3TILES = 1.402;

    private static final double HPICKUP_TILT1 = -81;
    private static final double HPICKUP_TILT2 = 4.05;
    private static final double HSLIDEPICKUP_PANANGLE = 0;
    private static final double HSLIDERPICKUP_PANANGLE = -2.0;

    // for pickup, need re-initialize in the code
    private static volatile double hPickupLen = 0;   // for level-5 cone
    private static volatile double hPickupTilt1Angle = 0;
    private static volatile double hPickupTilt2Angle = 0;
    private static volatile double hPickUpPanAngle = HSLIDERPICKUP_PANANGLE;
    // for v slider drop
    private static volatile double vDropLen = VSLIDELEN_HPOLE;
    private static volatile double vDropTiltAngle = VSLIDE_TILTANGLE;  //12
    private static volatile double vDropPanAngle = 120;
    private static volatile double vDropRotateAngle = 98;
    private final double VSLIDE_PAN_PUSH_ANGLE = 40;  // 30

    private int pickLevel = 5;
    private int coneCnt = 0;


    ElapsedTime timer = new ElapsedTime();

    int parkingPos = 1;  // for parking location, by camera
    private static volatile int robotPosValue = 0;
    private static volatile boolean pickUpConeDone = false;
    private static volatile  boolean handOverCone = false;
    private static volatile boolean pickUpConeTaskRunning = false;
    private static volatile boolean vSliderHoldConeFlag = false;
    private static volatile boolean dropConeTaskRunning = false;
    private static int dropPosValue = 0;

    private static volatile boolean foundCone = false;
    private static volatile boolean detectCone = true;


    @Override
    public void runOpMode(){
        telemetry.addLine("GamePad1: Please choose Left(X)/Right(B) side");
        telemetry.update();
        while(!isStopRequested() ){
            if (gamepad1.x){
                sideChoice = 1;  //left
                break;
            }else if(gamepad1.b){
                sideChoice = -1;  // right
                break;
            }
        }

        if (sideChoice == 1){
            telemetry.addLine("You choose Left Side");
            STARTX = -1.54;
            STARTY = 0.9 ;  // middle of tile
            STARTA = 0;
            targetX = STARTX + 1.24;
            targetY = STARTY - 0.48 + 0.02;
            targetA = -90;
            vDropPanAngle = 120;
            vDropRotateAngle = 98;

        }else{
            telemetry.addLine("You Choose Right Side");
            STARTX = -1.54;
            STARTY = -0.9 ;  // middle of tile
            STARTA = 0;
            targetX = STARTX + 1.24;
            targetY = STARTY + 0.48 - 0.02;
            targetA = 90;
            vDropPanAngle = -120;
            vDropRotateAngle = -98;

        }
        telemetry.addLine(" Start Initialize ......");
        telemetry.update();



        targetVel = 1.0;
        turnVel = 1.0;

        cameraObjectDetector = new AprilTagSignalDetect(this);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        chassis = new ChassisSystem(hardwareMap, telemetry);


        robotPos = new RobotPosition(STARTX,STARTY,STARTA);
        odometry = new Odometry(this, hardwareMap, chassis,robotPos, telemetry, true);
        writeSDCardFile(robotPos);
        sleep(1000);
        telemetry.addLine("odometry is setup");
        telemetry.update();


        chassisController = new MecanumDriveLib(this, odometry,chassis, telemetry);

        hSliderSystem = new HSliderSystem(this, hardwareMap, telemetry, true);
        sleep(1000);

        vSliderSystem = new VSliderSystem(this, hardwareMap, telemetry, true);



        hPickupLen = HPICKUP_5_LEN;   // for level-5 cone
        hPickupTilt1Angle = hPickup5Tilt1Angle[4] ;
        hPickupTilt2Angle = hPickup5Tilt2Angle[4]; // first the value is not same,
        hPickUpPanAngle = HPICKUP_PAN_ANGLE;

        if (sideChoice == 1){
            telemetry.addData("Left Side", "waiting for start....");
        }
        else{
            telemetry.addData("Right Side", "waiting for start....");
        }

        telemetry.update();
        hSliderSystem.initConeDetectValue();

        /////////////////////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();

        if(opModeIsActive()) {
            hSliderSystem.sliderLenCtrl(0.005,1.0);
            hSliderSystem.initConeDetectValue();
            timer.reset();
            vSliderSystem.setClawAngle(vSliderSystem.CLAW_CLOSE_ANGLE);
            vSliderSystem.setTiltAngle(-60);
            vSliderSystem.setRotateAngle(-28.79);
            pickLevel = 5;

            parkingPos = cameraObjectDetector.detectID(150);//  id start from 1,2,3

            if (parkingPos < 0 ) parkingPos = 3;

            if (sideChoice == 1){
                if (parkingPos == 1) {
                    parkPosition.x = -0.36;
                    parkPosition.y = 1.5;
                    parkPosition.angle = -90;
                } else if (parkingPos == 2) {
                    parkPosition.x = -0.36;
                    parkPosition.y = 0.9;
                    parkPosition.angle = -90;
                } else {
                    parkPosition.x = -0.48;
                    parkPosition.y = 0.28;
                    parkPosition.angle = 0;
                }
            }
            else{
                if (parkingPos == 1) {
                    parkPosition.x = -0.48;
                    parkPosition.y = -0.28;
                    parkPosition.angle = 0;

                } else if (parkingPos == 2) {
                    parkPosition.x = -0.24;
                    parkPosition.y = -0.9 + 0.02;
                    parkPosition.angle = 90;
                } else {
                    parkPosition.x = -0.24;
                    parkPosition.y = -1.47;
                    parkPosition.angle = 90;
                }
            }


            writeSDCardFile(parkPosition);

            robotPos = odometry.getRobotPosition();

            //go forward, then move forward a little bit
            int stepCnt = 0;
            chassisController.p2pDrive(new RobotPosition(STARTX + 0.6, STARTY, STARTA), targetVel, turnVel, 0.05, 5, 0.5, false, 3000);
            while (opModeIsActive() && !chassisController.p2pIsSettled()) {
                if (robotPos.x > STARTX + 0.25  && stepCnt == 0 ){
                    stepCnt = 1;
                    vSliderSystem.sliderLenCtrl(vDropLen, 1.0);
                    vSliderSystem.setTiltAngle(vDropTiltAngle);
                    vSliderSystem.setPanAngle(vDropPanAngle);
                    vSliderSystem.setRotateAngle(vDropRotateAngle);
                }
                sleep(10);
            }
            stepCnt = 0;
            hSliderSystem.setTilt1Angle(63.05);
            hSliderSystem.setTilt2Angle(65.0);
            chassisController.p2pDrive(new RobotPosition(targetX , STARTY, targetA), targetVel, turnVel, 0.02, 1, 0.1, false, 3000);
            while (opModeIsActive() && !chassisController.p2pIsSettled()) {



                sleep(10);
            }
            chassisController.stopP2PTask();

            hSlideOut_DetectCone_Thread_Task();


            // then move forward a little bit

            chassisController.p2pDrive(new RobotPosition(targetX, targetY, targetA), targetVel, turnVel, 0.01, 1, 0.1, true, 2000);
            while (opModeIsActive() && !chassisController.p2pIsSettled()) {
                sleep(10);
            }
            chassisController.stopP2PTask();
            // vSlide drop pre-load cone
            vSlideDropPreLoadCone();
            chassisController.setHoldPositionTaskStart(odometry.getRobotPosition());

            // make sure the cone detect task done
            timeOut.reset();
            while(opModeIsActive() && timeOut.milliseconds() < 4500 && detectConeTaskRunning){
                sleep(10);
            }
            sleep(10);

            detectConeTaskRunning = false; // stop detect task;
            hSliderSystem.stopDetectCone();
            HPICKUP_PAN_ANGLE = hSliderSystem.getPanAngle();
            hPickUpPanAngle = HPICKUP_PAN_ANGLE;
            hPickupLen = hSliderSystem.getSliderLen();



            while(opModeIsActive() && timer.milliseconds()< 26000 && pickLevel > 0){
                PickUp_HandOver_Cone_Thread_Task();
                pickLevel --;
                handOverCone = false;
                sleep(200);  // wait handOverCone Flag is set by pickup_handOver_cone_thread
                // wait for handOVerCone Flag is set by pickup_handover_cone_thread
                while(opModeIsActive() && !handOverCone){
                    sleep(10);
                }
                sleep(200);
                Drop_Cone_Thread_Task(2);
                vSliderHoldConeFlag = true;
                sleep(200);  // wait vSliderHoldConeFlag is set by Drop_Cone_Thread_Task

                while(opModeIsActive() && vSliderHoldConeFlag){
                    sleep(10);
                }

            }

            chassisController.setHoldPositionTaskOver();
            resetHorizontalSlide();
            if (opModeIsActive()){
                timeOut.reset();
                while(opModeIsActive() && dropConeTaskRunning && timeOut.milliseconds() < 1500){
                    sleep(10);
                }
                resetVerticalSlide();

                turnVel = 1.0;
                targetVel = 1.0;

                timeOut.reset();

                if (sideChoice == 1){
                    if (parkingPos == 1 && opModeIsActive()) {
                        chassisController.p2pDrive(new RobotPosition(parkPosition.x, parkPosition.y, parkPosition.angle), targetVel, turnVel, 0.01, 1, 0, true, 2500);
                        while (opModeIsActive() && !chassisController.p2pIsSettled()) {

                            sleep(10);
                        }
                        chassisController.stopP2PTask();


                    }
                    else if (parkingPos == 2 && opModeIsActive()) {

                        chassisController.p2pDrive(new RobotPosition(parkPosition.x, parkPosition.y, parkPosition.angle), targetVel, turnVel, 0.10, 1, 0.1, true, 2500);
                        while (opModeIsActive() && !chassisController.p2pIsSettled()) {

                            sleep(10);
                        }
                        chassisController.stopP2PTask();

                    }
                    else if (opModeIsActive()) {
                        // turn first
                        chassisController.setP2PTurnPID(0.04,0.02,0.05);
                        chassisController.p2pDrive(new RobotPosition(-0.2, parkPosition.y, parkPosition.angle), targetVel, turnVel, 0.10, 5, 0.2, true, 1500);
                        while (opModeIsActive() && !chassisController.p2pIsSettled()) {
                            sleep(10);
                        }
                        chassisController.stopP2PTask();
                        //position 3
                        chassisController.p2pDrive(new RobotPosition(parkPosition.x, parkPosition.y, parkPosition.angle), targetVel, turnVel, 0.10, 1, 0.1, true, 2500);
                        while (opModeIsActive() && !chassisController.p2pIsSettled()) {

                            telemetry.update();
                            sleep(10);
                        }
                        chassisController.stopP2PTask();

                    }
                }
                else{
                    if (parkingPos == 3 && opModeIsActive()) {
                        chassisController.p2pDrive(new RobotPosition(parkPosition.x, parkPosition.y, parkPosition.angle), targetVel, turnVel, 0.01, 1, 0, true, 2500);
                        while (opModeIsActive() && !chassisController.p2pIsSettled()) {

                            sleep(10);
                        }
                        chassisController.stopP2PTask();


                    }
                    else if (parkingPos == 2 && opModeIsActive()) {

                        chassisController.p2pDrive(new RobotPosition(parkPosition.x, parkPosition.y, parkPosition.angle), targetVel, turnVel, 0.10, 1, 0.1, true, 2500);
                        while (opModeIsActive() && !chassisController.p2pIsSettled()) {

                            sleep(10);
                        }
                        chassisController.stopP2PTask();

                    }
                    else if (opModeIsActive()) {
                        chassisController.setP2PTurnPID(0.04,0.02,0.05);
                        chassisController.p2pDrive(new RobotPosition(-0.2, parkPosition.y, parkPosition.angle), targetVel, turnVel, 0.10, 5, 0.2, true, 1500);
                        while (opModeIsActive() && !chassisController.p2pIsSettled()) {
                            sleep(10);
                        }
                        chassisController.stopP2PTask();
                        //position 3
                        chassisController.p2pDrive(new RobotPosition(parkPosition.x, parkPosition.y, parkPosition.angle), targetVel, turnVel, 0.10, 1, 0.1, true, 2500);
                        while (opModeIsActive() && !chassisController.p2pIsSettled()) {

                            telemetry.update();
                            sleep(10);
                        }
                        chassisController.stopP2PTask();

                    }
                }

                if (sideChoice == 1){
                    if (parkingPos == 2 || parkingPos == 1 ){
                        sleep(400); // wait for vertical slide reset done
                    }
                }
                else{
                    if (parkingPos == 2 || parkingPos == 3 ){
                        sleep(400); // wait for vertical slide reset done
                    }
                }



                updateTelemetry();

                //finish, stop all
                chassisController.stopAllThread();

                chassis.stopRobot();

                //write current position, may only use with tracking wheel
                robotPos = odometry.getRobotPosition();
                writeSDCardFile(robotPos);

                odometry.stopThread();
            }

        }
        ////////////////////////////
    }



    public void updateTelemetry()
    {
        telemetry.addData("Position", "%.2f %.2f %.2f", robotPos.x, robotPos.y, robotPos.angle);
        telemetry.addData("Timer:", "%f", timer.milliseconds());
        telemetry.addData("ParkingPos", " %d", parkingPos);
        telemetry.update();
    }





    private void writeSDCardFile(RobotPosition currentPos)
    {
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FIRST/ParkPos");
        dir.mkdirs();
        File file = new File(dir, "parkingPos.txt");
        try {
            FileOutputStream f = new FileOutputStream(file, false);
            final PrintStream printStream = new PrintStream(f);


            printStream.println(Double.toString(currentPos.x));
            printStream.println(Double.toString(currentPos.y));
            printStream.println(Double.toString(currentPos.angle));
            printStream.println("L");   // Right
            printStream.close();
            f.close();
        }
        catch(Exception e){

        }

    }

    private void resetVerticalSlide() {
        if (sideChoice == 1){
            if (opModeIsActive() && (parkingPos == 1 || parkingPos == 2)) {
                vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
                vSliderSystem.setRotateAngle(-90);
                vSliderSystem.setPanAngle(-20);
                sleep(150);
                vSliderSystem.setTiltAngle(-50);

                sleep(200);
                vSliderSystem.sliderLenCtrl(0.01, 0.2);

                hSliderSystem.setSlideBlockServoRelease(); // must before the h-slide moving in
            }
            else if(opModeIsActive()){
                vSliderSystem.setPanAngle(0);
                vSliderSystem.setTiltAngle(0);
                vSliderSystem.setRotateAngle(0);
                vSliderSystem.sliderLenCtrl(0.01, 1.0);
                vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
                hSliderSystem.setSlideBlockServoRelease(); // must before the h-slide moving in
            }
        }
        else{
            if (opModeIsActive() && (parkingPos == 2 || parkingPos == 3)) {
                vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
                vSliderSystem.setRotateAngle(-90);
                vSliderSystem.setPanAngle(-20);
                sleep(150);

                vSliderSystem.setTiltAngle(-50);

                sleep(200);
                vSliderSystem.sliderLenCtrl(0.01, 0.2);

                hSliderSystem.setSlideBlockServoRelease(); // must before the h-slide moving in
            }
            else if(opModeIsActive()){
                vSliderSystem.setPanAngle(0);
                vSliderSystem.setTiltAngle(0);
                vSliderSystem.setRotateAngle(0);
                vSliderSystem.sliderLenCtrl(0.01, 1.0);
                vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
                hSliderSystem.setSlideBlockServoRelease(); // must before the h-slide moving in
            }
        }

    }

    private void resetHorizontalSlide(){
        if (opModeIsActive()){

            if (opModeIsActive()){

                hSliderSystem.sliderLenCtrl(0.01, 1.0);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_CLOSE_ANGLE);


                hSliderSystem.setTilt1Angle(81);
                hSliderSystem.setTilt2Angle(17);
                if ((sideChoice == 1 && parkingPos == 3) || (sideChoice == -1 && parkingPos == 1)){
                    hSliderSystem.setPanAngle(0);
                }
                else{
                    hSliderSystem.setPanAngle(-35);
                }
                
            }

        }

    }








    void PickUp_HandOver_Cone_Thread_Task() {
        pickUpConeTaskRunning = true;
        new Thread(new Runnable() {
            @Override
            public void run() {

                if (opModeIsActive()) {
                    pickUpConeTaskRunning = true;
                    ElapsedTime timeOut = new ElapsedTime();
                    pickUpConeDone = false;

                    if (opModeIsActive() && !pickUpConeDone) {
                        handOverCone = false;  // hold cone
                        hSliderSystem.setTilt1Angle(hPickupTilt1Angle);  //hPickupTiltAngle
                        hSliderSystem.setTilt2Angle(hPickupTilt2Angle);  //hPickupTiltAngle
                        hSliderSystem.setPanAngle(hPickUpPanAngle);
                        hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE + 20);
                        hSliderSystem.sliderLenCtrl(hPickupLen, 1.0);

                        timeOut.reset();
                        while (opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - hPickupLen) > 0.02 && timeOut.milliseconds() < 2000) {
                            sleep(10);
                        }


                        // grab cone
                        //set block here
                        hSliderSystem.setSlideBlock();
                        hSliderSystem.setClawAngle(HSliderSystem.CLAW_CLOSE_ANGLE);
                        sleep(400);  // wait for claw hold cone

                        if (pickLevel == -2){

                            hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN, 1.0);  // may need wait for tilt servo, if yes, reduce the power
                            sleep(150);
                            hSliderSystem.setTilt1Angle(HHANDOVER_TILT1_ANGLE);
                            hSliderSystem.setTilt2Angle(HHANDOVER_TILT2_ANGLE);
                            sleep(100);
                            hSliderSystem.setPanAngle(HHANDOVER_PAN_ANGLE);
                        }
                        else{
                            // lift a little bit
                            hSliderSystem.setTilt1Angle(HHANDOVER_TILT1_ANGLE);
                            hSliderSystem.setTilt2Angle(HHANDOVER_TILT2_ANGLE);
                            sleep(250);
                            hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN, 1.0);  // may need wait for tilt servo, if yes, reduce the power
                            hSliderSystem.setPanAngle(HHANDOVER_PAN_ANGLE);
                        }

                        timeOut.reset();
                        while (opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - HSLIDER_HANDOVER_LEN) > 0.02 && timeOut.milliseconds() < 3000) {
                            sleep(10);
                        }

                        // need wait for tilt servo at right position
                        sleep(100);

                        handOverCone = true;
                    }
                }

                pickUpConeTaskRunning = false;
            }
        }).start();


    }


    private void stopPickUpTask(){
        pickUpConeTaskRunning = false;
    }


    void Drop_Cone_Thread_Task(int posFlag) {
        dropPosValue = posFlag;
        dropConeTaskRunning = true;
        new Thread(new Runnable() {
            @Override
            public void run() {
                if(opModeIsActive()) {
                    dropConeTaskRunning = true;
                    ElapsedTime timeOut = new ElapsedTime();
                    if (opModeIsActive() && dropConeTaskRunning) {
                        gotoDropPos_DropCone(dropPosValue);

                        if (coneCnt == 6){
                            timeOut.reset();
                            while (opModeIsActive() && timeOut.milliseconds() < 2000 && dropConeTaskRunning && Math.abs(vSliderSystem.getSliderLen() - vSlideStopHeight) > 0.02) {
                                sleep(10);
                            }
                        }
                        else{
                            timeOut.reset();
                            while (opModeIsActive() && timeOut.milliseconds() < 2000 && dropConeTaskRunning && Math.abs(vSliderSystem.getSliderLen() - VSLIDER_HANDOVER_LEN) > 0.02) {
                                sleep(10);
                            }
                        }

                        dropConeTaskRunning = false;
                    }
                    dropConeTaskRunning = false;
                }
            }
        }).start();
    }

    private boolean detectConeTaskRunning = false;

    void hSlideOut_DetectCone_Thread_Task() {
        detectConeTaskRunning = true;
        new Thread(new Runnable() {
            @Override
            public void run() {
                detectConeTaskRunning = true;
                if(opModeIsActive()) {
                    hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
                    hSliderSystem.sliderLenCtrl(hPickupLen, 0.4);
                    hSliderSystem.setTilt1Angle(sensorDetectTilt1);  // lower a little bit for detect one
                    hSliderSystem.setTilt2Angle(sensorDetectTilt2);  // lower the tilt2 first
                    hSliderSystem.setPanAngle(hPickUpPanAngle);

                    // make sure the slide out
                    ElapsedTime timeOut = new ElapsedTime();
                    timeOut.reset();
                    while (opModeIsActive() && detectConeTaskRunning && timeOut.milliseconds() < 2000  && Math.abs(hSliderSystem.getSliderLen() - hPickupLen) > 0.01) {
                        sleep(10);
                    }
                    boolean foundCone = false;
                    int bothFoundCnt = 0, leftFoundCnt = 0, rightFoundCnt = 0;
                    double deltaAngle = 8;
                    hSliderSystem.slowMoveSliderMtr(0.5); //0.35
                    timeOut.reset();
                    while(opModeIsActive() && !foundCone && timeOut.milliseconds() < 4000){
                        if (hSliderSystem.leftCone.getVoltage() > 2.0 && hSliderSystem.rightCone.getVoltage() > 2.0){
                            bothFoundCnt ++;
                            if (bothFoundCnt > 1){
                                foundCone = true;
                            }
                        }else if (hSliderSystem.leftCone.getVoltage() > 2.0){
                            leftFoundCnt ++;
                            if (leftFoundCnt > 1){
                                foundCone = true;
                            }
                        }else if (hSliderSystem.rightCone.getVoltage() > 2.0){
                            rightFoundCnt ++;
                            if (rightFoundCnt > 1){
                                foundCone = true;
                            }
                        }

                        if (foundCone){
                            hSliderSystem.slowMoveSliderMtr(0); // stop slide first
                            if (bothFoundCnt < 2){
                                if (leftFoundCnt > rightFoundCnt){
                                    hSliderSystem.setPanAngle(hPickUpPanAngle - deltaAngle);

                                }
                                else if(leftFoundCnt < rightFoundCnt){
                                    hSliderSystem.setPanAngle(hPickUpPanAngle + deltaAngle);

                                }
                            }

                        }else{
                            sleep(100);
                        }

                    }
                    hSliderSystem.slowMoveSliderMtr(0);
                    sleep(200);

                    hSliderSystem.setTilt1Angle(hPickupTilt1Angle);
                    hSliderSystem.setTilt2Angle(hPickupTilt2Angle);
                    hSliderSystem.sliderLenCtrl(hSliderSystem.getSliderLen() - 0.02, 0.6);
                    sleep(250);
                    detectConeTaskRunning = false;

                }
            }
        }).start();
    }


    void stopDropConeTask(){

        dropConeTaskRunning = false;
    }


    void gotoDropPos_DropCone(int polePosFlag){  // A

        if(opModeIsActive()) {


            ElapsedTime timeOut = new ElapsedTime();


            hSliderSystem.setSlideBlockServoRelease();
            vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
            sleep(200);

            vSliderHoldConeFlag = true;
            //open hSlideClaw
            hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
            sleep(150);
            hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN + 0.05, 1.0);
            sleep(100);
            hSlide_GotoPickUp_Thread_Task();


            double targetPanAngle = 200;
            double targetRoteAngle = 100;


            vDropTiltAngle = VSLIDE_TILTANGLE;
            vSliderSystem.setTiltAngle(vDropTiltAngle);
            ///////////// robot  -- pole ////////////////////////////////////////
            //      Pole(1)       (Front)   Pole(2)
            //              (Left)Robot(Right)
            //      Pole(0)       (Back)    Pole(3)

            // now only for left/right front pole 1,1


            if (sideChoice == -1) { // left back pole
                vDropPanAngle = -150 + VSLIDE_PAN_PUSH_ANGLE;
                vDropRotateAngle = -90;
                vDropLen = VSLIDELEN_HPOLE;
            } else { // right back pole
                vDropPanAngle = 150.44 - VSLIDE_PAN_PUSH_ANGLE;
                vDropRotateAngle = 90;
                vDropLen = VSLIDELEN_HPOLE;
            }
            targetRoteAngle = vDropRotateAngle;
            vSliderSystem.setRotateAngle(targetRoteAngle);
            // rise slide
            vSliderSystem.sliderLenCtrl(vDropLen, 1.0);
            // then pan
            targetPanAngle = vDropPanAngle;
            vSliderSystem.setPanAngle(targetPanAngle, 300);



            timeOut.reset();
            while (opModeIsActive() && timeOut.milliseconds() < 2500) {
                if (Math.abs(vSliderSystem.getPanAngle() - targetPanAngle) < 2.0 ){//&& Math.abs(vSliderSystem.getSliderLen() - vDropLen) < 0.05) {
                    break;
                }
                sleep(10);
            }
            targetPanAngle = targetPanAngle + Math.signum(targetPanAngle) * (VSLIDE_PAN_PUSH_ANGLE);
            vSliderSystem.setPanAngle(targetPanAngle, 250);
            timeOut.reset();
            while (opModeIsActive() && timeOut.milliseconds() < 1500) {
                if (Math.abs(vSliderSystem.getPanAngle() - targetPanAngle) < 2.0 && Math.abs(vSliderSystem.getSliderLen() - vDropLen) < 0.02) {
                    break;
                }
                sleep(10);
            }
            //drop cone
            vSliderSystem.setRotateAngle(Math.signum(targetRoteAngle) * 185);
            sleep(150);

            /////////////////////// ???????????????????????????????
           // vSliderHoldConeFlag = false;  // set here, timing is Ok?  too early now

            //
            vSliderSystem.setTiltAngle(-80); // down
            sleep(150);
            vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
            coneCnt ++;
            sleep(150);

           //  vSliderHoldConeFlag = false;  // set here, timing is Ok?  too early now

            //

            vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
            if (vSliderSystem.getSliderLen() < 0.35) {
                vSliderSystem.sliderLenCtrl(0.45, 1.0);
            }


            if (coneCnt != 6) {
                vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 0.2);
            }


            vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE);
            timeOut.reset();
            while (opModeIsActive() && timeOut.milliseconds() < 2000 && Math.abs(vSliderSystem.getPanAngle()) > 30) {
               // if (vSliderSystem.getPanAngle() < 90){
               //     vSliderHoldConeFlag = false;  // timing OK???
              //  }
                sleep(10);
            }

            vSliderHoldConeFlag = false;  // timing???
            if (coneCnt == 6){
                vSliderSystem.setTiltAngle(-45);
                vSliderSystem.setPanAngle(0);
                vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
            }
            else{
                vSliderSystem.setTiltAngle(VHANDOVER_TILT_ANGLE);
                vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
            }

            vSliderSystem.setRotateAngle(VHANDOVER_ROTATE_ANGLE);
            sleep(200);
            vSliderHoldConeFlag = false;  //timing

            sleep(200);
         //   vSliderHoldConeFlag = false;  //timing

            // then down

            if (coneCnt == 6){
                vSliderSystem.sliderLenCtrl(vSlideStopHeight, 1.0);
                timeOut.reset();
                while (opModeIsActive() && Math.abs(vSliderSystem.getSliderLen() - vSlideStopHeight) > 0.02 && timeOut.milliseconds() < 2000) {
                    sleep(10);
                }
            }
            else {
                vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 1.0);
                timeOut.reset();
                while (opModeIsActive() && Math.abs(vSliderSystem.getSliderLen() - VSLIDER_HANDOVER_LEN) > 0.02 && timeOut.milliseconds() < 2000) {
                    sleep(10);
                }
            }

            vSliderHoldConeFlag = false;

        }

    }

    void hSlide_GotoPickUp_Thread_Task(){

        new Thread(new Runnable() {
            @Override
            public void run() {
                if(opModeIsActive()) {
                    if (pickLevel > 0) {
                        if (pickLevel == 5) {
                            hPickupLen = hPickupLen;
                            hPickupTilt1Angle = hPickup5Tilt1Angle[4];
                            hPickupTilt2Angle = hPickup5Tilt2Angle[4];
                            hPickUpPanAngle = HPICKUP_PAN_ANGLE;
                        } else if (pickLevel == 4) {
                            hPickupLen = hPickupLen - 0.015;
                            hPickupTilt1Angle = hPickup5Tilt1Angle[3];
                            hPickupTilt2Angle = hPickup5Tilt2Angle[3];
                            hPickUpPanAngle = HPICKUP_PAN_ANGLE;
                        } else if (pickLevel == 3) {
                            hPickupLen = hPickupLen - 0.005;
                            hPickupTilt1Angle = hPickup5Tilt1Angle[2];
                            hPickupTilt2Angle = hPickup5Tilt2Angle[2];
                            hPickUpPanAngle = HPICKUP_PAN_ANGLE;
                        } else if (pickLevel == 2) {
                            hPickupLen = hPickupLen + 0.015;
                            hPickupTilt1Angle = hPickup5Tilt1Angle[1];
                            hPickupTilt2Angle = hPickup5Tilt2Angle[1];
                            hPickUpPanAngle = HPICKUP_PAN_ANGLE;
                        } else {  //if(pickLevel == 1)
                            hPickupLen = hPickupLen + 0.025; //0.035
                            hPickupTilt1Angle = hPickup5Tilt1Angle[0];
                            hPickupTilt2Angle = hPickup5Tilt2Angle[0];
                            hPickUpPanAngle = HPICKUP_PAN_ANGLE;
                        }
                        hSliderSystem.sliderLenCtrl(hPickupLen, 0.6);

                        hSliderSystem.setTilt1Angle(hPickupTilt1Angle);
                        hSliderSystem.setTilt2Angle(hPickupTilt2Angle);
                        hSliderSystem.setPanAngle(hPickUpPanAngle);
                        hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE + 10);
                    }
                    else{
                        hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN + 0.1, 0.6);  // keep the claw out of substation, no need for auto

                        hSliderSystem.setTilt1Angle(0);
                        hSliderSystem.setTilt2Angle(0);
                    }
                }

            }
        }).start();
    }

    private void vSlideDropPreLoadCone(){
        if(opModeIsActive()) {
            double targetPanAngle = vDropPanAngle + Math.signum(vDropPanAngle) * (VSLIDE_PAN_PUSH_ANGLE);
            vSliderSystem.setPanAngle(targetPanAngle, 200);  // 160
            timeOut.reset();
            while (opModeIsActive() && timeOut.milliseconds() < 1500 && Math.abs(vSliderSystem.getPanAngle() - targetPanAngle) > 2) {
                sleep(10);
            }
            //drop cone
            vSliderSystem.setRotateAngle(Math.signum(vDropRotateAngle) * 185);
            sleep(150);


            vSliderSystem.setTiltAngle(-80); // down
            sleep(100);
            vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
            sleep(300);
            vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE + 30);

            coneCnt ++;
            if (vSliderSystem.getSliderLen() < 0.35) {
                vSliderSystem.sliderLenCtrl(0.45, 1.0);
            }


            vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE);
            timeOut.reset();
            while (opModeIsActive() && timeOut.milliseconds() < 2000 && Math.abs(vSliderSystem.getPanAngle()) > 30) {
                sleep(10);
            }
            timeOut.reset();
            while (opModeIsActive() && timeOut.milliseconds() < 2000 && vSliderSystem.getSliderLen() < 0.4) {
                sleep(10);
            }
            vSliderSystem.setTiltAngle(VHANDOVER_TILT_ANGLE);
            vSliderSystem.setRotateAngle(VHANDOVER_ROTATE_ANGLE);
            sleep(200);
            vSliderHoldConeFlag = false;
            vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
            sleep(200);


            // then down
            vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 1.0);
            vSliderHoldConeFlag = false;
        }
    }




}