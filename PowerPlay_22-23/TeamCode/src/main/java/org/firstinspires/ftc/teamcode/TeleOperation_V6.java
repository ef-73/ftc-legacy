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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;


@TeleOp(name="TeleOp_V6")
//@Disabled

public class TeleOperation_V6 extends LinearOpMode {
    private static volatile ElapsedTime timeOut = new ElapsedTime();
    private double[] hPickup5Tilt1Angle = {-51.31 + 2.0,-39.71 + 0.0,-25.38 - 3.0,-13.47 + 0.0,2.43 + 0.0};
    private double[] hPickup5Tilt2Angle = {-8.01 - 2.0,-15.58 - 0.0,-29.88 + 3.0,-41.59 - 0.0,-60.83 - 0.0};
    private static volatile boolean autoMove = true;
    private static volatile ChassisSystem chassis = null;
    private static volatile Odometry odometry = null;
    //MecanumXYTurnDriveLib mecanumDrive = null;
    private static volatile MecanumDriveLib chassisController = null;

    private static volatile HSliderSystem hSliderSystem = null;
    private static volatile VSliderSystem vSliderSystem = null;
    private static volatile RobotPosition robotPos = null;

    //-------------------------------------------------------------------------------------

    private static final double targetX = -0.37;
    private static final double targetY = 0.30;
    private static final double targetA = 0;


    //reset arm
    public final double HSLIDE_TILT1_RESET_ANGLE = 75;
    public final double HSLIDE_TILT2_RESET_ANGLE = 50;
    public final double VSLIDE_TILT_UP_ANGLE = 75;

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




    //--------------------------------------------------------------------------------
    //init

    private static boolean firstPickupFlag = true;
    private static final double hPickUpBackLen = 0.2;

    private static volatile double hPickupLen = 0.77;
    private static volatile double hPickupTilt1Angle = 0;
    private static volatile double hPickupTilt2Angle = 0;
    private static volatile double hPickUpPanAngle = 0;

    private static volatile double vDropLen = 0.43;
    private static double vDropTiltAngle = 10;  //12




// some preset value here
    private final double VSLIDE_PAN_PUSH_ANGLE = 35;
    private final double VSLIDELEN_LPOLE = 0.10;  // for low pole

    private final double VSLIDELEN_MPOLE = 0.18;  // for medium pole
    private final double VSLIDELEN_HPOLE = 0.44;  // for high pole

    private final double VSLIDE_TILTANGLE = 0;  // drop tilt angle


    private final double HSLIDEPICKUPLEN_1TILE = 0.19;
    private final double HSLIDEPICKUPLEN_2TILES = 0.77;
    private final double HSLIDEPICKUPLEN_3TILES = 1.38;

    private final double HPICKUP_TILT1 = -50.31;
    private final double HPICKUP_TILT2 = -8.01;
    private final double HSLIDEPICKUP_PANANGLE = 0;

    //---------------------------------------------------------------------------------

    private boolean isConeStack = false;

    private static final int dropLevel = 3;  //high junction

    private static int polePosition = 2; //middle pole

    String posFlag = "R";
    private int posIndex = 0;
    private int posNum = 16;
    private TeleOpPresetData[] preSetPos = new TeleOpPresetData[posNum];
    private static volatile int vClawDelayCnt = 0;
    private static volatile int hClawDelayCnt = 0;


    private static volatile int robotPosValue = 0;
    private static volatile boolean pickUpConeDone = false;
    private static volatile  boolean handOverCone = false;
    private static volatile boolean pickUpConeTaskRunning = false;

    private boolean resetPosFlag = false;

    // for left 5 cone
    private boolean left5Cone = false;
    private int fakePosIndex = 0;
    private static volatile int pickLevel = 4;  // 0 ~ 4, 5 level

    private int gamepad1YCnt = 0;
    private double hpickup5ConeLen = 0.72;


    private static volatile double hPickup5Cone_Pan_Angle = 0;

    private static boolean left5ConeFirst = false;
    private boolean autoCircle = false;
    @Override
    public void runOpMode(){
        pickLevel = 4;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        chassis = new ChassisSystem(hardwareMap, telemetry);

        //hard code robot position, must be at D3 center
        robotPos = new RobotPosition(0,0.0,0);
        //read back the robot position from autonomous

        if (!readSDCardFile()){
            robotPos = new RobotPosition(0,0,0);
        }


        telemetry.addData("Pos ( ", "%2.2f, %2.2f, %2.2f )", robotPos.x, robotPos.y, robotPos.angle);


        odometry = new Odometry(this, hardwareMap, chassis,robotPos, telemetry, false);
        telemetry.addLine("odometry is setup");

        //mecanumDrive = new MecanumXYTurnDriveLib(this, chassis, odometry,telemetry);
        chassisController = new MecanumDriveLib(this, odometry,chassis, telemetry);

        vSliderSystem = new VSliderSystem( this, hardwareMap, telemetry, false);
        hSliderSystem = new HSliderSystem(this, hardwareMap, telemetry, false);





        //initialize preset position data with HSLIDER and VSlider Data
        initializePosData();

        telemetry.addData("Working Mode", "waiting for start " + posFlag);

        telemetry.update();
       robotPosValue = 0;
       pickUpConeDone = false;
       handOverCone = false;
       pickUpConeTaskRunning = false;


        waitForStart();


        // reset arm position here

        teleFirstResetArms();

        hSliderSystem.enableTouchCheck();
        vSliderSystem.enableTouchCheck();


        while (opModeIsActive()){

            robotPos = odometry.getRobotPosition();


            if (chassisController.p2pIsSettled()) {
                chassis.chassisTeleOp(gamepad1);
            }
            /*
            if (gamepad1.back && !autoCircle)
            {
                // v-slide using gamepad2 dpad go to preset high position
                if (gamepad2.dpad_up){
                    vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
                    sleep(200);
                    hSliderSystem.setSlideBlockServoRelease();
                    hSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN + 0.5, 1.0);
                    vSliderSystem.setTiltAngle(VSLIDE_TILT_UP_ANGLE);
                    vSliderSystem.sliderLenCtrl(VSLIDELEN_HPOLE, 1.0);
                    vSliderSystem.setPanAngle(180);
                    sleep(200);
                    vSliderSystem.setRotateAngle(180);
                    vSliderSystem.setTiltAngle(20);
                    hSliderSystem.sliderLenCtrl(0.01,1.0);
                }
                else

                if (gamepad2.dpad_left){

                    vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
                    sleep(200);
                    hSliderSystem.setSlideBlockServoRelease();
                    hSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN + 0.5, 1.0);
                    vSliderSystem.setTiltAngle(VSLIDE_TILT_UP_ANGLE);
                    vSliderSystem.sliderLenCtrl(VSLIDELEN_MPOLE, 1.0);
                    vSliderSystem.setPanAngle(180);
                    sleep(200);
                    vSliderSystem.setRotateAngle(180);
                    vSliderSystem.setTiltAngle(30);
                    hSliderSystem.sliderLenCtrl(0.01,1.0);
                }
                else if (gamepad2.dpad_right){
                    vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
                    sleep(200);
                    hSliderSystem.setSlideBlockServoRelease();
                    hSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN + 0.5, 1.0);
                    vSliderSystem.setTiltAngle(VSLIDE_TILT_UP_ANGLE);
                    vSliderSystem.sliderLenCtrl(VSLIDELEN_LPOLE, 1.0);
                    vSliderSystem.setPanAngle(180);
                    sleep(200);
                    vSliderSystem.setRotateAngle(180);
                    vSliderSystem.setTiltAngle(30);
                    hSliderSystem.sliderLenCtrl(0.01,1.0);
                    vSliderSystem.sliderLenCtrl(0.01,1.0);
                }
            }
            else*/

            if (gamepad2.dpad_up)
            {
                if (!left5Cone){
                    if (!dropConeTaskRunning){
                        Drop_Cone_Thread_Task(1);
                    }
                }
                else if(left5Cone && fakePosIndex == 1){
                    if (!dropConeTaskRunning){
                        Drop_5Cone_Thread_Task(1);
                    }
                }


            }else if(gamepad2.dpad_right)
            {
                if (!left5Cone){
                    if (!dropConeTaskRunning){
                        Drop_Cone_Thread_Task(2);
                    }
                }
                else if(left5Cone && fakePosIndex == 6){
                    if (!dropConeTaskRunning){
                        Drop_5Cone_Thread_Task(2);
                    }

                }

            }
            else if(gamepad2.dpad_down)
            {
                if (!left5Cone){
                    if (!dropConeTaskRunning){
                        Drop_Cone_Thread_Task(3);
                    }
                }
                else if(left5Cone){
                    if (!dropConeTaskRunning){
                        Drop_5Cone_Thread_Task(3);
                    }
                }

            }
            else if(gamepad2.dpad_left)
            {

                if (!left5Cone){
                    if (!dropConeTaskRunning){
                        Drop_Cone_Thread_Task(0);
                    }
                }
                else if(left5Cone){
                    if(!dropConeTaskRunning) {
                        Drop_5Cone_Thread_Task(0);
                    }
                }

            }

            if ((gamepad2.left_bumper) && (gamepad2.left_trigger > 0.5)){
                // sweep cone in the substation
                updateHSlideData();
                hSliderSystem.sliderLenCtrl(hPickupLen - hPickUpBackLen, 1.0);
                hSliderSystem.setTilt1Angle(-52);
                hSliderSystem.setTilt2Angle(-3.45);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_CLOSE_ANGLE);
                sleep(150);
                if (robotPos.y <= 0) {
                    hSliderSystem.setPanAngle(HSliderSystem.PAN_MIN);
                    sleep(200);
                    hSliderSystem.sliderLenCtrl(hPickupLen + 0.03, 1.0);
                    sleep(250);
                    hSliderSystem.setPanAngle(HSliderSystem.PAN_MAX);
                    sleep(400);
                }
                else{
                    hSliderSystem.setPanAngle(HSliderSystem.PAN_MAX);
                    sleep(200);
                    hSliderSystem.sliderLenCtrl(hPickupLen + 0.03, 1.0);
                    sleep(250);
                    hSliderSystem.setPanAngle(HSliderSystem.PAN_MIN);
                    sleep(400);
                }
                /*
                hSliderSystem.setPanAngle(HSliderSystem.PAN_MAX);
                sleep(400);
                hSliderSystem.setPanAngle(HSliderSystem.PAN_MIN);
                sleep(400);*/
                hSliderSystem.setPanAngle(hPickUpPanAngle);
                hSliderSystem.sliderLenCtrl(hPickupLen - hPickupLen, 1.0);
                sleep(200);

            }
            else{
                hSliderSystem.joystickCtrl(gamepad1, gamepad2);

            }

            if (gamepad2.right_bumper && gamepad2.right_trigger > 0.5){

            }
            else{
                vSliderSystem.joystickCtrl(gamepad1, gamepad2);
            }




            //v-slide claw open-close control
            if (gamepad2.right_stick_button && vClawDelayCnt == 0)
            {
                if (vSliderSystem.getClawAngle() <=vSliderSystem.CLAW_OPEN_ANGLE + 10)
                {
                    vSliderSystem.setClawAngle(vSliderSystem.CLAW_CLOSE_ANGLE);
                    vClawDelayCnt  = 10;
                }
                else{
                    vSliderSystem.setClawAngle(vSliderSystem.CLAW_OPEN_ANGLE);
                    vClawDelayCnt  = 10;
                    vSliderHoldConeFlag = false;
                }
            }else{
                vClawDelayCnt --;
                vClawDelayCnt = vClawDelayCnt < 0 ? 0 : vClawDelayCnt;
            }
            // h-slide claw open-close control
            if (gamepad2.left_stick_button && hClawDelayCnt == 0)
            {
                if (hSliderSystem.getClawAngle() <= hSliderSystem.CLAW_OPEN_ANGLE + 10)
                {
                    hSliderSystem.setClawAngle(hSliderSystem.CLAW_CLOSE_ANGLE);
                    hClawDelayCnt  = 10;
                }
                else{
                    hSliderSystem.setClawAngle(hSliderSystem.CLAW_OPEN_ANGLE);
                    hClawDelayCnt  = 10;
                }
            }else{
                hClawDelayCnt --;
                hClawDelayCnt = hClawDelayCnt < 0 ? 0 : hClawDelayCnt;
            }






            if (gamepad2.y)
            {
                if (gamepad2.back) {
                    hPickupTilt1Angle = HPICKUP_TILT1;
                    hPickupTilt2Angle = HPICKUP_TILT2;

                    hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
                    hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN + 0.05, 1.0);
                    hSliderSystem.setTilt1Angle(hPickupTilt1Angle);
                    hSliderSystem.setTilt2Angle(hPickupTilt2Angle);
                    hSliderSystem.setPanAngle(hPickUpPanAngle);

                    // vSlide ready
                    vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
                    vSliderSystem.setTiltAngle(VHANDOVER_TILT_ANGLE);
                    vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 1.0);
                    vSliderSystem.setRotateAngle(VHANDOVER_ROTATE_ANGLE);


                }
                else {
                    if (!left5Cone){
                        getReadyPickup();
                    }
                    else if (pickLevel >= 0){
                        // vSlide ready
                        hSliderSystem.setSlideBlockServoRelease();
                        hPickupLen = hpickup5ConeLen;
                        hPickupTilt1Angle = hPickup5Tilt1Angle[pickLevel];
                        hPickupTilt2Angle = hPickup5Tilt2Angle[pickLevel];
                        hPickUpPanAngle = hPickup5Cone_Pan_Angle;

                        hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);

                        hSliderSystem.sliderLenCtrl(hPickupLen, 1.0);
                        hSliderSystem.setTilt1Angle(hPickupTilt1Angle);
                        hSliderSystem.setTilt2Angle(hPickupTilt2Angle);
                        hSliderSystem.setPanAngle(hPickUpPanAngle);
                        // vSlide ready
                        vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
                        vSliderSystem.setTiltAngle(VHANDOVER_TILT_ANGLE);
                        vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 1.0);
                        vSliderSystem.setRotateAngle(VHANDOVER_ROTATE_ANGLE);
                    }
                }


            }

            if (gamepad2.b) {
                if (gamepad2.back){

                    if (!pickUpConeTaskRunning) {
                        pickUpNow_HandOver_Cone_Thread_Task();
                    }
                }
                else{
                    if (!left5Cone){
                        if (!pickUpConeTaskRunning){
                            if (hSliderSystem.getTilt2Angle() > HHANDOVER_TILT2_ANGLE - 15){
                                hSliderSystem.sliderLenCtrl(hSliderSystem.getSliderLen() + 0.1, 1.0);
                                sleep(100);
                            }
                            pickUp_HandOver_Cone_Thread_Task();
                        }
                    }
                    else if (pickLevel >= 0 ){
                        // pick up left 5 cone
                        // based on tilt1 angle to decide which level should go

                        if (!pickUpConeTaskRunning) {
                            if (left5ConeFirst) {
                                left5ConeFirst = false;
                                telemetry.addData("Before PickupLevel", " %d", pickLevel);
                                double tilt2A = hSliderSystem.getTilt2Angle();

                                for (int i = 4; i >= 0; i--) {
                                    if (Math.abs(tilt2A - hPickup5Tilt2Angle[i]) < 10) {
                                        pickLevel = i;
                                        break;
                                    }
                                }
                                telemetry.addData("PickupLevel", " %d", pickLevel);
                                telemetry.update();
                            }

                            hPickupTilt1Angle = hSliderSystem.getTilt1Angle();
                            hPickupTilt2Angle = hSliderSystem.getTilt2Angle();
                            hPickup5Cone_Pan_Angle = hSliderSystem.getPanAngle();
                            hpickup5ConeLen = hSliderSystem.getSliderLen();
                            hPickupLen = hpickup5ConeLen;
                            hPickUpPanAngle = hPickup5Cone_Pan_Angle;
                            pickUpConeTaskRunning = true;
                            pickLevel --;
                            pickUp5_HandOver_Cone_Thread_Task();


                        }
                    }
                }



            }










            if (gamepad1.x){
                //set global position at D3, angle????
                robotPos = new RobotPosition(targetX, targetY,0);
                odometry.setRobotPosition(robotPos);
                resetPosFlag = true;
                left5Cone = false;
            }
            if (gamepad1.b){
                // D4
                robotPos = new RobotPosition(targetX, -targetY,0);
                odometry.setRobotPosition(robotPos);
                resetPosFlag = true;
                left5Cone = false;
            }


             if (gamepad1.a){
                // goes to preset D3 or D4 tile
                 if(Math.abs(robotPos.angle) > 20 && Math.abs(robotPos.y) > 0.6) {


                     if (robotPos.y > 0) {
                        chassisController.setP2PTurnPID(0.04,0.02,0.05);
                         // auto stop at D1 or D2 tile, left side
                         chassisController.p2pDrive(new RobotPosition(-0.25, 0.34, -90), 1.0, 1.0, 0.05, 5, 0.1, false, 2000);
                         while (opModeIsActive() && !chassisController.p2pIsSettled() && !gamepad1.back) {
                             sleep(10);
                         }
                         chassisController.stopP2PTask();
                         // then turn and move back
                         if (!gamepad1.back) {
                             chassisController.p2pDrive(new RobotPosition(targetX, targetY, targetA), 1.0, 1.0, 0.01, 1, 0.1, true, 2000);
                             while (opModeIsActive() && !chassisController.p2pIsSettled() && !gamepad1.back) {
                                 sleep(10);
                             }

                         }
                         chassisController.stopP2PTask();
                     } else {
                         // auto stop at D1 or D2 tile, right side
                         chassisController.setP2PTurnPID(0.04,0.02,0.05);
                         chassisController.p2pDrive(new RobotPosition(-0.25, -0.34, 90), 1.0, 1.0, 0.05, 5, 0.1, false, 2000);
                         while (opModeIsActive() && !chassisController.p2pIsSettled() && !gamepad1.back) {
                             sleep(10);
                         }
                         chassisController.stopP2PTask();
                         // then turn and move back
                         if (!gamepad1.back) {
                             chassisController.p2pDrive(new RobotPosition(targetX, -targetY, targetA), 1.0, 1.0, 0.05, 5, 0.1, true, 2000);
                             while (opModeIsActive() && !chassisController.p2pIsSettled() && !gamepad1.back) {
                                 sleep(10);
                             }
                             chassisController.stopP2PTask();
                         }
                     }
                 }

            }

             /*
            if (gamepad1.right_trigger > 0.5){

                if (gamepad1.y){
                    pickupLevel --;
                    if (pickupLevel < 0) pickupLevel = 4;
                }
                sleep(100);

            }*/

            if (gamepad2.a || gamepad1.right_trigger > 0.5) {
                // suppose the cone on the unicorn
                resetArm_Task();

            }


            if (gamepad1.y && gamepad1YCnt == 0){
                gamepad1YCnt = 10;
                // add code for left 5 cone
                if (!left5Cone){
                    left5Cone = true;
                    pickLevel = 4;
                    left5ConeFirst = true;
                    //prepare data for pickup
                    robotPos = odometry.getRobotPosition();
                    if (robotPos.y > 0){
                        fakePosIndex = 6;
                    }
                    else{
                        fakePosIndex = 1;
                    }

                }
                else{
                    left5Cone = false;
                    left5ConeFirst = false;
                }

            }
            else{
                gamepad1YCnt--;
                gamepad1YCnt = gamepad1YCnt < 0 ? 0 : gamepad1YCnt;
            }

            if (gamepad1.right_trigger > 0.3){

                hSliderSystem.stopDetectCone();
                hSliderSystem.initConeDetectValue();
                hSliderSystem.setPanAngle(0);
            }




            if (gamepad2.x){

                vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 1.0);
                vSliderSystem.setTiltAngle(VHANDOVER_TILT_ANGLE);
                vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE);
                vSliderSystem.setRotateAngle(VHANDOVER_ROTATE_ANGLE);
                vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
                hSliderSystem.setTilt1Angle(HHANDOVER_TILT1_ANGLE);
                hSliderSystem.setTilt2Angle(HHANDOVER_TILT2_ANGLE);
                hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN, 1.0);
                hSliderSystem.setPanAngle(HHANDOVER_PAN_ANGLE);
                timeOut.reset();
                while(opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - HSLIDER_HANDOVER_LEN) > 0.02 && timeOut.milliseconds() < 3000){
                    sleep(10);
                }

                vSliderHoldConeFlag = false;
            }


            if (gamepad2.start) {
                autoCircle = true;

                firstPickupFlag = false;
                for (int i = 0; i < 5 && gamepad2.start; i++) {
                    if (opModeIsActive() && gamepad2.start) {
                        pickUp_HandOver_Cone_Thread_Task();
                        handOverCone = false;
                        sleep(200);  // wait handOverCone Flag is set by pickup_handOver_cone_thread
                        // wait for handOverCone Flag is set by pickup_handover_cone_thread
                        while (opModeIsActive() && !handOverCone && gamepad2.start) {
                            sleep(10);
                        }
                        sleep(10);
                        if (gamepad2.dpad_right && gamepad2.start) {
                            if (preSetPos[robotPosValue].teleVSlidePresetPos[2].available == true) {
                                Drop_Cone_Thread_Task(2);
                            } else {
                                Drop_Cone_Thread_Task(preSetPos[robotPosValue].defaultPolePos);
                            }
                        } else if (gamepad2.dpad_left && gamepad2.start) {
                            if (preSetPos[robotPosValue].teleVSlidePresetPos[0].available == true) {
                                Drop_Cone_Thread_Task(0);
                            } else {
                                Drop_Cone_Thread_Task(preSetPos[robotPosValue].defaultPolePos);
                            }
                        } else if (gamepad2.dpad_down && gamepad2.start) {
                            if (preSetPos[robotPosValue].teleVSlidePresetPos[3].available == true) {
                                Drop_Cone_Thread_Task(3);
                            } else {
                                Drop_Cone_Thread_Task(preSetPos[robotPosValue].defaultPolePos);
                            }
                        } else if (gamepad2.dpad_up && gamepad2.start) {
                            if (preSetPos[robotPosValue].teleVSlidePresetPos[1].available == true) {
                                Drop_Cone_Thread_Task(1);
                                //nigger
                            } else {
                                Drop_Cone_Thread_Task(preSetPos[robotPosValue].defaultPolePos);
                            }
                        } else if (gamepad2.start) {
                            Drop_Cone_Thread_Task(preSetPos[robotPosValue].defaultPolePos);
                        }
                        vSliderHoldConeFlag = true;
                        sleep(400); // make sure the v-slide hold cone-set vSliderHoldConeFlag, then wait for v-slide release cone
                        timeOut.reset();
                        while (opModeIsActive() && timeOut.milliseconds() < 3000 && vSliderHoldConeFlag) {
                            sleep(10);
                        }
                        sleep(200);

                    }


                }
                autoCircle = false;
                sleep(1000);  // wait for vertical slide down
                stopDropConeTask();
                stopPickUpTask();
            }




            // gamepad1 to control the chassis movement

            if (gamepad1.dpad_up && resetPosFlag){
                robotPos = odometry.getRobotPosition();
                double targetAngle = robotPos.angle;//(int)(robotPos.angle / 360) * 360;  // in case the robot rotate too much circles
                /*
                if (Math.abs(robotPos.angle % 360) > 180){
                    if (robotPos.angle > 0) targetAngle = targetAngle + 360;
                    if (robotPos.angle < 0) targetAngle = targetAngle - 360;
                }*/
                double targetX = robotPos.x;
                if (gamepad1.start){
                    // to move 2 tile forward distance  0.6 * 2
                   targetX = robotPos.x + 0.6 * 2 - 0.05;
                }
                else{
                    // to move 1 tile forward distance 0.6
                    targetX = robotPos.x + 0.6 -0.05;
                }

                chassisController.setMinPower(0.2,0.3,0.1);
                RobotPosition targetPos = new RobotPosition(targetX, robotPos.y, targetAngle);
                chassisController.p2pDrive(targetPos,1.0,0.4,0.02,2,0,true,4000);
                while(opModeIsActive() && !chassisController.p2pIsSettled() && !gamepad1.back){
                    sleep(10);
                }
                chassisController.stopP2PTask();
                chassisController.stopRobot();
                ///update the presetPos
                updateHSlideData();

            }else if(gamepad1.dpad_down && resetPosFlag){
                while(opModeIsActive() && (dropConeTaskRunning || pickUpConeTaskRunning)){
                    sleep(10);
                }

                resetArms();
                robotPos = odometry.getRobotPosition();
                double targetAngle =robotPos.angle;// (int)(robotPos.angle / 360) * 360;  // in case the robot rotate too much circles
                /*
                if (Math.abs(robotPos.angle % 360) > 180){
                    if (robotPos.angle > 0) targetAngle = targetAngle + 360;
                    if (robotPos.angle < 0) targetAngle = targetAngle - 360;
                }*/
                double targetX = robotPos.x;
                if (gamepad1.start){
                    // to move 2 tile forward distance  0.6 * 2
                    targetX = robotPos.x - 0.6 * 2 + 0.05;
                }
                else{
                    // to move 1 tile forward distance 0.6
                    targetX = robotPos.x - 0.6 + 0.05;
                }
                chassisController.setMinPower(0.2,0.3,0.1);
                RobotPosition targetPos = new RobotPosition(targetX, robotPos.y, targetAngle);
                chassisController.p2pDrive(targetPos,1.0,0.4,0.02,2,0,true,4000);
                while(opModeIsActive() && !chassisController.p2pIsSettled() && !gamepad1.back){
                    sleep(10);
                }
                chassisController.stopP2PTask();
                chassisController.stopRobot();
                updateHSlideData();
            }else if(gamepad1.dpad_left && resetPosFlag){
                while(opModeIsActive() && (dropConeTaskRunning || pickUpConeTaskRunning)){
                    sleep(10);
                }

                resetArms();

                chassisController.setMinPower(0.2,0.3,0.1);
                robotPos = odometry.getRobotPosition();
                double targetAngle = robotPos.angle ;//(int)(robotPos.angle / 360) * 360;  // in case the robot rotate too much circles
               /* if (Math.abs(robotPos.angle % 360) > 180){
                    if (robotPos.angle > 0) targetAngle = targetAngle + 360;
                    if (robotPos.angle < 0) targetAngle = targetAngle - 360;
                }*/
                double targetX = robotPos.x;
                double targetY = robotPos.y;
                if (gamepad1.start){
                    // to move 2 tile forward distance  0.6 * 2
                    targetY = robotPos.y + 0.6 * 2;
                }
                else{
                    // to move 1 tile forward distance 0.6
                    targetY = robotPos.y + 0.6;
                }
                double offsetX = 0.15;
                // move x direction forward first
                RobotPosition targetPos = new RobotPosition(targetX + offsetX, robotPos.y, targetAngle);
                chassisController.p2pDrive(targetPos,1.0,0.4,0.02,5,0,false,4000);
                while(opModeIsActive() && !chassisController.p2pIsSettled() && !gamepad1.back){
                    sleep(10);
                }
                if (!chassisController.p2pIsSettled()){
                    chassisController.stopP2PTask();
                    chassisController.stopRobot();
                }
                else{
                    // first step finished
                    targetPos = new RobotPosition(targetX + offsetX, targetY, targetAngle);
                    chassisController.p2pDrive(targetPos,1.0,0.4,0.02,5,0,false,4000);
                    while(opModeIsActive() && !chassisController.p2pIsSettled() && !gamepad1.back){
                        sleep(10);
                    }
                    if (!chassisController.p2pIsSettled()){
                        chassisController.stopP2PTask();
                        chassisController.stopRobot();
                    }
                    else{
                      // then move back 0.15
                        //hSliderSystem.setTilt1Angle(0);  // tilt horizontal slider arm down
                        //hSliderSystem.setTilt2Angle(0);  // tilt horizontal slider arm down
                        targetPos = new RobotPosition(targetX, targetY, targetAngle);
                        chassisController.p2pDrive(targetPos,0.6,0.4,0.02,2,0,false,4000);
                        while(opModeIsActive() && !chassisController.p2pIsSettled() && !gamepad1.back){
                            sleep(10);
                        }
                        if (!chassisController.p2pIsSettled()){
                            chassisController.stopP2PTask();
                            chassisController.stopRobot();
                        }
                    }
                }
                updateHSlideData();

            }else if(gamepad1.dpad_right  && resetPosFlag){
                while(opModeIsActive() && (dropConeTaskRunning || pickUpConeTaskRunning)){
                    sleep(10);
                }

                resetArms();


                chassisController.setMinPower(0.2,0.3,0.1);
                robotPos = odometry.getRobotPosition();
                double targetAngle = robotPos.angle;//(int)(robotPos.angle / 360) * 360;  // in case the robot rotate too much circles
                /*if (Math.abs(robotPos.angle % 360) > 180){
                    if (robotPos.angle > 0) targetAngle = targetAngle + 360;
                    if (robotPos.angle < 0) targetAngle = targetAngle - 360;
                }*/
                double targetX = robotPos.x;
                double targetY = robotPos.y;
                if (gamepad1.start){
                    // to move 2 tile forward distance  0.6 * 2
                    targetY = robotPos.y - 0.6 * 2;
                }
                else{
                    // to move 1 tile forward distance 0.6
                    targetY = robotPos.y - 0.6;
                }

                double offsetX = 0.15;
                // move x direction forward first
                RobotPosition targetPos = new RobotPosition(targetX + offsetX, robotPos.y, targetAngle);
                chassisController.p2pDrive(targetPos,1.0,0.4,0.02,5,0,false,4000);
                while(opModeIsActive() && !chassisController.p2pIsSettled() && !gamepad1.back){
                    sleep(10);
                }
                if (!chassisController.p2pIsSettled()){
                    chassisController.stopP2PTask();
                    chassisController.stopRobot();
                }
                else{
                    // first step finished
                    targetPos = new RobotPosition(targetX + offsetX, targetY, targetAngle);
                    chassisController.p2pDrive(targetPos,1.0,0.4,0.02,5,0,false,4000);
                    while(opModeIsActive() && !chassisController.p2pIsSettled() && !gamepad1.back){
                        sleep(10);
                    }
                    if (!chassisController.p2pIsSettled()){
                        chassisController.stopP2PTask();
                        chassisController.stopRobot();
                    }
                    else{
                        // then move back 0.15
                        //hSliderSystem.setTilt1Angle(0);  // tilt horizontal slider arm down
                        //hSliderSystem.setTilt2Angle(0);  // tilt horizontal slider arm down
                        targetPos = new RobotPosition(targetX, targetY, targetAngle);
                        chassisController.p2pDrive(targetPos,0.6,0.4,0.02,2,0,false,4000);
                        while(opModeIsActive() && !chassisController.p2pIsSettled() && !gamepad1.back){
                            sleep(10);
                        }
                        if (!chassisController.p2pIsSettled()){
                            chassisController.stopP2PTask();
                            chassisController.stopRobot();
                        }
                    }
                }
                updateHSlideData();
            }




            robotPos = odometry.getRobotPosition();
            if (true) {
                telemetry.addData("Position", "%.2f %.2f %.2f", robotPos.x, robotPos.y, robotPos.angle);
                telemetry.addData("encoder", "Left:%d Right:%d Middle:%d",
                        odometry.leftEnc.getCurrentPosition(),
                        odometry.rightEnc.getCurrentPosition(),
                        odometry.middleEnc.getCurrentPosition());
                telemetry.addData("imu", "%3.2f", odometry.getIMUReading());
                telemetry.addData("PosIndex", "%d", getPosIndex());
                if (left5Cone) {

                    telemetry.addData("5Cone", "Pos Index %d  PickLevel %d", fakePosIndex, pickLevel);
                } else {
                    telemetry.addLine("Normal Op");
                }
                telemetry.addData("PickupLevel", " %d", pickLevel);
                telemetry.addData("HSliderLen", "%2.3f", hSliderSystem.getSliderLen());
                telemetry.addData("HSliderTilt1", "%2.2f", hSliderSystem.getTilt1Angle());
                telemetry.addData("HSliderTilt2", "%2.2f", hSliderSystem.getTilt2Angle());
                telemetry.addData("HSliderPanA", "%2.2f", hSliderSystem.getPanAngle());
                telemetry.addData("HSliderClawA", "%2.2f", hSliderSystem.getClawAngle());


                telemetry.addData("VSliderLen", "%2.3f", vSliderSystem.getSliderLen());
                telemetry.addData("VSliderTiltA", "%2.2f", vSliderSystem.getTiltAngle());
                telemetry.addData("VSliderPanA", "%2.2f", vSliderSystem.getPanAngle());
                telemetry.addData("VSliderClawA", "%2.2f", vSliderSystem.getClawAngle());
                telemetry.addData("VSliderRotate", "%2.2f", vSliderSystem.getRotateAngle());

                telemetry.addData("LeftCone", "%2.2f", hSliderSystem.leftCone.getVoltage());
                telemetry.addData("RightCone", "%2.2f", hSliderSystem.rightCone.getVoltage());
                telemetry.addData("LeftIni", "%2.2f", hSliderSystem.leftIniValue);
                telemetry.addData("RightIni", "%2.2f", hSliderSystem.rightIniValue);
                telemetry.addData("VSliderRotate", "%2.2f", vSliderSystem.getRotateAngle());

                telemetry.update();
            }

            sleep(10);
            //idle();
        }

        //mecanumDrive.stopGoXYnTurn();
        chassisController.stopAllThread();
        chassis.stopRobot();

        odometry.stopThread();


    }

    private void updateHSlideData(){
        int posIndex = getPosIndex();
        if (posIndex >= 0 && posIndex < posNum) {
            hPickupLen = preSetPos[posIndex].hPickupLen;
            hPickUpPanAngle = preSetPos[posIndex].hPickupPan;
            hPickupTilt1Angle = preSetPos[posIndex].hPickupTilt1;
            hPickupTilt2Angle = preSetPos[posIndex].hPickupTilt2;
        }
    }

    private void getReadyPickup()   //Y
    {
        int posIndex = getPosIndex();
        if (posIndex >= 0 && posIndex < posNum){
            if (preSetPos[posIndex].hSlideMove){
                hSliderSystem.setSlideBlockServoRelease();
                hSliderSystem.sliderLenCtrl(0.15, 1.0);
                sleep(250);
                vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 1.0);
                vSliderSystem.setTiltAngle(VHANDOVER_TILT_ANGLE);
                vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE);
                vSliderSystem.setRotateAngle(VHANDOVER_ROTATE_ANGLE);
                vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
                vSliderHoldConeFlag = false;
                handOverCone = false;

                hPickupLen = preSetPos[posIndex].hPickupLen;
                hPickUpPanAngle = preSetPos[posIndex].hPickupPan;
                hPickupTilt1Angle = preSetPos[posIndex].hPickupTilt1;
                hPickupTilt2Angle = preSetPos[posIndex].hPickupTilt2;
                chassisController.stopAllThread();
                chassisController.stopRobot();

                hSliderSystem.sliderLenCtrl(hPickupLen, 1.0);


                hSliderSystem.setTilt1Angle(hPickupTilt1Angle);  //hPickupTiltAngle
                hSliderSystem.setTilt2Angle(hPickupTilt2Angle);  //hPickupTiltAngle
                hSliderSystem.setPanAngle(hPickUpPanAngle);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
               // if (firstPickupFlag){
                    timeOut.reset();
                    while(opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - hPickupLen) > 0.03 && timeOut.milliseconds() < 3000){
                        sleep(10);
                    }
                    //then move back
                    sleep(100);
                    hSliderSystem.sliderLenCtrl(hPickupLen - hPickUpBackLen, 1.0);

             //   }

            }
        }

    }


    private void gotoDropPos_DropCone(int polePosFlag){  // A
        int posIndex = getPosIndex();
        if (posIndex >= 0){
            ElapsedTime timeOut = new ElapsedTime();
            chassisController.stopAllThread();

            if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].available){
                hSliderSystem.setSlideBlockServoRelease();
                hSliderSystem.setSlideBlockServoRelease();
                vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
                sleep(150);

                vSliderHoldConeFlag = true;
                //open hSlideClaw
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
                sleep(100);
                hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN + 0.05, 1.0);
                sleep(100);

                if (posIndex < 8 && !(gamepad2.right_bumper && gamepad2.right_trigger > 0.5)){
                    hSlide_GotoPickUp_Thread_Task();
                }
                else if(gamepad2.right_bumper && gamepad2.right_trigger > 0.5){
                    hSliderSystem.setSlideBlockServoRelease();
                    hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN + 0.05, 1.0);
                    hSliderSystem.setTilt1Angle(45);
                    hSliderSystem.setTilt2Angle(45);
                }
                else {
                    hSliderSystem.setSlideBlockServoRelease();
                    hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN + 0.15, 1.0);
                    hSliderSystem.setTilt1Angle(30);
                    hSliderSystem.setTilt2Angle(30);
                  //  sleep(200);
                 //   hSliderSystem.sliderLenCtrl(0.01, 1.0);

                }

                double targetPanAngle = 200;
                double targetRoteAngle = 100;


                vDropTiltAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vtiltAngle;
                vSliderSystem.setTiltAngle(vDropTiltAngle);
                targetRoteAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vRotateAngle;
                vSliderSystem.setRotateAngle(targetRoteAngle);

                vDropLen = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen;
                vSliderSystem.sliderLenCtrl(vDropLen, 1.0);

                if (vDropLen > VSLIDELEN_LPOLE) {
                    // here is for High/ Medium Pole
                    //
                    if (polePosFlag == 1) {
                        // already up a little bit and tilt up
                        if(preSetPos[posIndex].teleVSlidePresetPos[polePosFlag - 1].available) {


                            if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag - 1].vslideLen
                                    < preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen) {
                                // left front pole higher than left back pole
                                // wait the slider higher than pre pole, in case hit the pole
                                // pre pan a little bit
                                /*
                                targetPanAngle = -20;  // pre pan a little bit, left is negative pan angle
                                vSliderSystem.setPanAngle(targetPanAngle);
                                timeOut.reset();
                                while(opModeIsActive() && vSliderSystem.getSliderLen() < preSetPos[posIndex].teleVSlidePresetPos[polePosFlag - 1].vslideLen + 0.05  && timeOut.milliseconds() < 2000) {
                                    sleep(10);  // wait for slide higher than pre index pole
                                }*/
                                // then pan
                                targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;
                                vSliderSystem.setPanAngle(targetPanAngle, 300);  //400
                            } else {
                                //left front pole lower than left back pole
                                vDropLen = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen;
                                vSliderSystem.sliderLenCtrl(vDropLen, 1.0);
                                sleep(300);
                                vDropTiltAngle = -70;
                                vSliderSystem.setTiltAngle(vDropTiltAngle);
                                sleep(200); // wait for tilt up
                                // then pan
                                targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;
                                vSliderSystem.setPanAngle(targetPanAngle, 300);
                                timeOut.reset();
                                while (opModeIsActive() && timeOut.milliseconds() < 3000) {
                                    if (Math.abs(vSliderSystem.getPanAngle()) > Math.abs(preSetPos[posIndex].teleVSlidePresetPos[polePosFlag - 1].vPanAngle) + 30) {
                                        // tilt it back
                                        vDropTiltAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vtiltAngle;
                                        vSliderSystem.setTiltAngle(vDropTiltAngle);
                                        vDropLen = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen;
                                        vSliderSystem.sliderLenCtrl(vDropLen, 1.0);
                                        break;
                                    }
                                    sleep(10);
                                }

                            }
                        }
                        else{
                            targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;
                            vSliderSystem.setPanAngle(targetPanAngle, 300);  //400
                        }
                    }
                    else if (polePosFlag == 2 )
                    {
                        if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag + 1].available) {
                            // right side pole
                            if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag + 1].vslideLen
                                    < preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen) {
                                // right front pole is higher than right back pole

                                // then pan
                                targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;
                                vSliderSystem.setPanAngle(targetPanAngle, 300);  //400
                            } else {
                                // right front pole is lower than right back pole
                                vDropLen = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag + 1].vslideLen;
                                vSliderSystem.sliderLenCtrl(vDropLen, 1.0);
                                sleep(300);
                                vDropTiltAngle = -70;
                                vSliderSystem.setTiltAngle(vDropTiltAngle);
                                sleep(300); // wait for tilt up
                                // then pan
                                targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;
                                vSliderSystem.setPanAngle(targetPanAngle, 300);
                                timeOut.reset();
                                while (opModeIsActive() && timeOut.milliseconds() < 3000) {
                                    if (Math.abs(vSliderSystem.getPanAngle()) > Math.abs(preSetPos[posIndex].teleVSlidePresetPos[polePosFlag + 1].vPanAngle) + 30) {
                                        // tilt it back
                                        vDropTiltAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vtiltAngle;
                                        vSliderSystem.setTiltAngle(vDropTiltAngle);
                                        vDropLen = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen;
                                        vSliderSystem.sliderLenCtrl(vDropLen, 1.0);
                                        sleep(300);
                                        break;
                                    }
                                    sleep(10);
                                }
                            }
                        }
                        else{
                            targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;
                            vSliderSystem.setPanAngle(targetPanAngle, 300); //400
                        }


                    }
                    else{
                        //posFlag = 0 or posFlag = 3, directly pan

                        targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;

                        if (polePosFlag == 0 || polePosFlag == 3){
                            if (vDropLen < VSLIDELEN_HPOLE){
                                vSliderSystem.setPanAngle(targetPanAngle);
                            }
                            else{
                                vSliderSystem.setPanAngle(targetPanAngle); //high pole
                            }

                        }
                        else{
                            vSliderSystem.setPanAngle(targetPanAngle);   //1000
                        }

                    }

                    // drop cone
                    if (true){  // dropConeTaskRunning
                        timeOut.reset();
                        while(opModeIsActive() &&  timeOut.milliseconds() < 2500){
                            if (Math.abs(vSliderSystem.getPanAngle() - targetPanAngle) < 2.0 ){//&& Math.abs(vSliderSystem.getSliderLen() - vDropLen) < 0.02){
                                break;
                            }
                            sleep(10);
                        }
                        targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle + Math.signum(targetPanAngle) * (VSLIDE_PAN_PUSH_ANGLE);
                        vSliderSystem.setPanAngle(targetPanAngle, 200);  // 200
                        timeOut.reset();
                        while(opModeIsActive() &&   timeOut.milliseconds() < 1000){
                            if (Math.abs(vSliderSystem.getPanAngle() - targetPanAngle) < 5.0 && Math.abs(vSliderSystem.getSliderLen() - vDropLen) < 0.02){
                                break;
                            }
                            sleep(10);
                        }
                        //drop cone
                        while(opModeIsActive() && gamepad2.back){
                            sleep(10);
                        }
                        vSliderSystem.setRotateAngle(Math.signum(targetRoteAngle) * 185);
                        sleep(150);


                        vSliderSystem.setTiltAngle(-80); // down
                        sleep(100);  // 200
                        vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
                        if (vDropLen > VSLIDELEN_MPOLE && (polePosFlag == 0 || polePosFlag == 3)){
                            vSliderHoldConeFlag = false;   //timing ok ?
                        }
                        if ((polePosFlag == 1 || polePosFlag == 2) && ((posIndex == 2) || posIndex == 5)){
                            vSliderHoldConeFlag = false;  // timing OK?
                        }



                        //pan back a little bit in case to shake pole
                        vSliderSystem.setPanAngle(targetPanAngle - Math.signum(targetPanAngle) * 20, 150);

                        // high pole sleep a little bit more
                        if ((polePosFlag == 0 && preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen > VSLIDELEN_MPOLE)
                                || (polePosFlag == 2 && preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen >VSLIDELEN_MPOLE)){
                            sleep(350);
                        }
                        else{
                            sleep(150);
                        }
                        sleep(150);
                        vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
                        /*
                        if (polePosFlag == 1){
                            if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag - 1].available){
                                if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen > preSetPos[posIndex].teleVSlidePresetPos[polePosFlag - 1].vslideLen){
                                    vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 0.2); // down
                                }
                            }
                        }
                        else if (polePosFlag == 2){
                            if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag + 1].available){
                                if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen > preSetPos[posIndex].teleVSlidePresetPos[polePosFlag + 1].vslideLen){
                                    vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 0.2); // down
                                }
                            }
                        }
                        else{
                            vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 0.2); // down
                        }
*/



                        vSliderHoldConeFlag = false; // timing ok for pole 1,2
                        /*
                        if (vSliderSystem.getSliderLen() < 0.35){
                            vSliderSystem.sliderLenCtrl(0.45, 1.0);
                        }
                        */
                        vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE); // VHAND
                        timeOut.reset();
                        while(opModeIsActive() && timeOut.milliseconds() < 2000 && Math.abs(vSliderSystem.getPanAngle()) > 20){
                            sleep(10);
                        }
                        /*
                        timeOut.reset();
                        while(opModeIsActive() && timeOut.milliseconds() < 2000 && vSliderSystem.getSliderLen() < 0.4){
                            sleep(10);
                        }*/


                        vSliderSystem.setTiltAngle(VHANDOVER_TILT_ANGLE);
                        vSliderSystem.setRotateAngle(VHANDOVER_ROTATE_ANGLE);
                        sleep(200);
                        vSliderHoldConeFlag = false;
                        vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
                        sleep(200);


                        // then down
                        vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
                        vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 1.0);
                        timeOut.reset();
                        while(opModeIsActive() && Math.abs(vSliderSystem.getSliderLen() - VSLIDER_HANDOVER_LEN) > 0.02 && timeOut.milliseconds() < 2000){
                            sleep(10);
                        }


                        vSliderHoldConeFlag = false;
                    }


                }
                else{
                    if (polePosFlag == 1 || polePosFlag == 2) {
                        vSliderSystem.setTiltAngle(65);
                        sleep(200);
                        targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;
                        vSliderSystem.setPanAngle(targetPanAngle, 300);  //400
                        timeOut.reset();
                        while(opModeIsActive() && timeOut.milliseconds() < 2000) {
                            if (Math.abs(vSliderSystem.getPanAngle()) > 50) {
                                break;
                            }
                        }
                        vSliderSystem.sliderLenCtrl(0.002, 1.0);
                        vSliderSystem.setTiltAngle(-19);
                        sleep(200);
                        timeOut.reset();
                        while(opModeIsActive() && timeOut.milliseconds() < 2000) {
                            if (Math.abs(vSliderSystem.getPanAngle() - targetPanAngle) < 5) {
                                break;
                            }
                        }
                        // pan again
                        targetPanAngle = targetPanAngle + Math.signum(targetPanAngle) * VSLIDE_PAN_PUSH_ANGLE;
                        vSliderSystem.setPanAngle(targetPanAngle);
                        timeOut.reset();
                        while( opModeIsActive() && timeOut.milliseconds() < 1500 && Math.abs(vSliderSystem.getPanAngle() - targetPanAngle) > 2){
                            sleep(10);
                        }
                        // rotate
                        vSliderSystem.setRotateAngle(Math.signum(vSliderSystem.getRotateAngle()) * 185);
                        sleep(400);
                        //tilt down
                        vSliderSystem.setTiltAngle(-40);

                        vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
                        sleep(300);

                        // up in case hit
                        double targetLen = 0.25;
                        vSliderSystem.sliderLenCtrl(targetLen,1.0);

                        timeOut.reset();
                        while(opModeIsActive() && timeOut.milliseconds() < 2000 && vSliderSystem.getSliderLen() < (targetLen - 0.02)){
                            sleep(10);
                        }
                        vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE);
                        timeOut.reset();
                        while(opModeIsActive() && timeOut.milliseconds() < 2000 && Math.abs(vSliderSystem.getPanAngle()) > 30){
                            sleep(10);
                        }

                        vSliderSystem.setRotateAngle(VHANDOVER_ROTATE_ANGLE);
                        sleep(200);
                        vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
                        sleep(200);
                        // then down
                        vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 1.0);
                        timeOut.reset();
                        while(opModeIsActive() && Math.abs(vSliderSystem.getSliderLen() - VSLIDER_HANDOVER_LEN) > 0.02 && timeOut.milliseconds() < 2000){
                            sleep(10);
                        }
                        vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
                        vSliderSystem.setTiltAngle(VHANDOVER_TILT_ANGLE);
                        vSliderHoldConeFlag = false;

                    }
                    else {

                        //only for low pole
                        //hard code here, regarding the setting in initialize pos data function, only at position 0,7,8,9,14,15
                        //tilt up and pan a little bit
                        sleep(300);
                        //vSliderSystem.setTiltAngle(0);
                        // vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN + 0.04, 1.0);
                        // sleep(200);

                        if (polePosFlag == 0 && (posIndex == 0 || posIndex == 9 || posIndex == 15)) {
                            // left back low pole
                            vSliderSystem.setPanAngle(-25);
                            vSliderSystem.setRotateAngle(-90);
                        } else if (polePosFlag == 3 && (posIndex == 7 || posIndex == 14 || posIndex == 8)) {
                            vSliderSystem.setPanAngle(25);
                            vSliderSystem.setRotateAngle(90);
                        }
                        sleep(400);
                        vSliderSystem.setTiltAngle(-19);  // tilt down


                        // then rotate and down to low pole len
                        if (polePosFlag == 0 && (posIndex == 0 || posIndex == 9 || posIndex == 15)){
                            vSliderSystem.setRotateAngle(-95);
                        }
                        else if(polePosFlag == 3 && (posIndex == 7 || posIndex == 14 || posIndex == 8)){
                            vSliderSystem.setRotateAngle(95);
                        }
                        // then down
                        vSliderSystem.sliderLenCtrl(0.002, 1.0);
                        sleep(300);
                        // then pan to position
                        double targetPan = 0;
                        if (polePosFlag == 0 && (posIndex == 0 || posIndex == 9 || posIndex == 15)){
                            targetPan = -55;
                            vSliderSystem.setPanAngle(targetPan);
                        }
                        else if(polePosFlag == 3 && (posIndex == 7 || posIndex == 14 || posIndex == 8)){
                            targetPan = 55;
                            vSliderSystem.setPanAngle(targetPan);
                        }

                        timeOut.reset();
                        while( opModeIsActive() && timeOut.milliseconds() < 1500 && Math.abs(vSliderSystem.getPanAngle() - targetPan) > 2){
                            sleep(10);
                        }
                        // rotate
                        vSliderSystem.setRotateAngle(Math.signum(vSliderSystem.getRotateAngle()) * 185);
                        sleep(400);
                        //tilt down
                        vSliderSystem.setTiltAngle(-40);

                        vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
                        sleep(300);

                        // up in casae hig
                        double targetLen = 0.25;
                        vSliderSystem.sliderLenCtrl(targetLen,1.0);

                        timeOut.reset();
                        while(opModeIsActive() && timeOut.milliseconds() < 2000 && vSliderSystem.getSliderLen() < (targetLen - 0.02)){
                            sleep(10);
                        }
                        vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE);
                        timeOut.reset();
                        while(opModeIsActive() && timeOut.milliseconds() < 2000 && Math.abs(vSliderSystem.getPanAngle()) > 30){
                            sleep(10);
                        }

                        vSliderSystem.setRotateAngle(VHANDOVER_ROTATE_ANGLE);
                        sleep(200);
                        vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
                        sleep(200);
                        // then down
                        vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 1.0);
                        timeOut.reset();
                        while(opModeIsActive() && Math.abs(vSliderSystem.getSliderLen() - VSLIDER_HANDOVER_LEN) > 0.02 && timeOut.milliseconds() < 2000){
                            sleep(10);
                        }
                        vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
                        vSliderSystem.setTiltAngle(VHANDOVER_TILT_ANGLE);
                        vSliderHoldConeFlag = false;
                    }



                }
            }

        }

    }

    private void gotoDropPos_DropCone5(int polePosFlag){  // A
        int posIndex = fakePosIndex;
        if (posIndex >= 0){
            ElapsedTime timeOut = new ElapsedTime();
            chassisController.stopAllThread();

            if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].available){
                hSliderSystem.setSlideBlockServoRelease();
                vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
                sleep(150);

                vSliderHoldConeFlag = true;
                //open hSlideClaw
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
                sleep(100);
                hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN + 0.05, 1.0);
                sleep(100);

                if (pickLevel >= 0){
                    hSlide_GotoPickUp5_Thread_Task();
                }
                else {
                    hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN + 0.15, 1.0);
                }

                double targetPanAngle = 200;
                double targetRoteAngle = 100;


                vDropTiltAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vtiltAngle;
                vSliderSystem.setTiltAngle(vDropTiltAngle);
                targetRoteAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vRotateAngle;
                vSliderSystem.setRotateAngle(targetRoteAngle);

                vDropLen = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen;
                vSliderSystem.sliderLenCtrl(vDropLen, 1.0);

                if (vDropLen > VSLIDELEN_LPOLE){
                    // here is for High/ Medium Pole
                    //
                    if (polePosFlag == 1 )
                    {
                        if(preSetPos[posIndex].teleVSlidePresetPos[polePosFlag - 1].available) {


                            if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag - 1].vslideLen
                                    < preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen) {

                                targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;
                                vSliderSystem.setPanAngle(targetPanAngle, 300);  //400
                            } else {
                                //left front pole lower than left back pole
                                vDropLen = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen;
                                vSliderSystem.sliderLenCtrl(vDropLen, 1.0);
                                sleep(300);
                                vDropTiltAngle = -70;
                                vSliderSystem.setTiltAngle(vDropTiltAngle);
                                sleep(200); // wait for tilt up
                                // then pan
                                targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;
                                vSliderSystem.setPanAngle(targetPanAngle, 300);
                                timeOut.reset();
                                while (opModeIsActive() && timeOut.milliseconds() < 3000) {
                                    if (Math.abs(vSliderSystem.getPanAngle()) > Math.abs(preSetPos[posIndex].teleVSlidePresetPos[polePosFlag - 1].vPanAngle) + 30) {
                                        // tilt it back
                                        vDropTiltAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vtiltAngle;
                                        vSliderSystem.setTiltAngle(vDropTiltAngle);
                                        vDropLen = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen;
                                        vSliderSystem.sliderLenCtrl(vDropLen, 1.0);
                                        break;
                                    }
                                    sleep(10);
                                }

                            }
                        }
                        else{
                            targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;
                            vSliderSystem.setPanAngle(targetPanAngle, 300);  //400
                        }



                    }
                    else if (polePosFlag == 2 )
                    {
                        if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag + 1].available) {
                            // right side pole
                            if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag + 1].vslideLen
                                    < preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen) {
                                // right front pole is higher than right back pole
                                /*
                                targetPanAngle = 20;  // pre pan a little bit, right is positive pan angle
                                vSliderSystem.setPanAngle(targetPanAngle);
                                timeOut.reset();
                                while(opModeIsActive() && vSliderSystem.getSliderLen() < preSetPos[posIndex].teleVSlidePresetPos[polePosFlag + 1].vslideLen + 0.05 && timeOut.milliseconds() < 2000) {
                                    sleep(10);  // wait for slide higher than pre index pole
                                }
                                */

                                // then pan
                                targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;
                                vSliderSystem.setPanAngle(targetPanAngle, 300);  //400
                            } else {
                                // right front pole is lower than right back pole
                                vDropLen = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag + 1].vslideLen;
                                vSliderSystem.sliderLenCtrl(vDropLen, 1.0);
                                sleep(300);
                                vDropTiltAngle = -70;
                                vSliderSystem.setTiltAngle(vDropTiltAngle);
                                sleep(300); // wait for tilt up
                                // then pan
                                targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;
                                vSliderSystem.setPanAngle(targetPanAngle, 300);
                                timeOut.reset();
                                while (opModeIsActive() && timeOut.milliseconds() < 3000) {
                                    if (Math.abs(vSliderSystem.getPanAngle()) > Math.abs(preSetPos[posIndex].teleVSlidePresetPos[polePosFlag + 1].vPanAngle) + 30) {
                                        // tilt it back
                                        vDropTiltAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vtiltAngle;
                                        vSliderSystem.setTiltAngle(vDropTiltAngle);
                                        vDropLen = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen;
                                        vSliderSystem.sliderLenCtrl(vDropLen, 1.0);
                                        sleep(300);
                                        break;
                                    }
                                    sleep(10);
                                }
                            }
                        }
                        else{
                            targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;
                            vSliderSystem.setPanAngle(targetPanAngle, 300); //400
                        }


                    }
                    else{
                        //posFlag = 0 or posFlag = 3, directly pan

                        targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle;

                        if (polePosFlag == 0 || polePosFlag == 3){
                            if (vDropLen < VSLIDELEN_HPOLE){
                                vSliderSystem.setPanAngle(targetPanAngle);
                            }
                            else{
                                vSliderSystem.setPanAngle(targetPanAngle); //high pole
                            }

                        }
                        else{
                            vSliderSystem.setPanAngle(targetPanAngle);   //1000
                        }

                    }

                    // drop cone
                    if (true){  // dropConeTaskRunning
                        timeOut.reset();
                        while(opModeIsActive() &&  timeOut.milliseconds() < 2500){
                            if (Math.abs(vSliderSystem.getPanAngle() - targetPanAngle) < 2.0 ){//&& Math.abs(vSliderSystem.getSliderLen() - vDropLen) < 0.02){
                                break;
                            }
                            sleep(10);
                        }
                        targetPanAngle = preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vPanAngle + Math.signum(targetPanAngle) * (VSLIDE_PAN_PUSH_ANGLE);
                        vSliderSystem.setPanAngle(targetPanAngle, 200);  // 200
                        timeOut.reset();
                        while(opModeIsActive() &&   timeOut.milliseconds() < 1000){
                            if (Math.abs(vSliderSystem.getPanAngle() - targetPanAngle) < 5.0 && Math.abs(vSliderSystem.getSliderLen() - vDropLen) < 0.02){
                                break;
                            }
                            sleep(10);
                        }
                        //drop cone
                        vSliderSystem.setRotateAngle(Math.signum(targetRoteAngle) * 185);
                        sleep(150);


                        vSliderSystem.setTiltAngle(-80); // down
                        sleep(100);  // 200
                        vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
                        if (vDropLen > VSLIDELEN_MPOLE && (polePosFlag == 0 || polePosFlag == 3)){
                            vSliderHoldConeFlag = false;   //timing ok ?
                        }
                        if ((polePosFlag == 1 || polePosFlag == 2) && ((posIndex == 2) || posIndex == 5)){
                            vSliderHoldConeFlag = false;  // timing OK?
                        }



                        //pan back a little bit in case to shake pole
                        vSliderSystem.setPanAngle(targetPanAngle - Math.signum(targetPanAngle) * 20, 150);

                        // high pole sleep a little bit more
                        if ((polePosFlag == 0 && preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen > VSLIDELEN_MPOLE)
                                || (polePosFlag == 2 && preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen >VSLIDELEN_MPOLE)){
                            sleep(350);
                        }
                        else{
                            sleep(150);
                        }

                        vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE + 45);
                        /*
                        if (polePosFlag == 1){
                            if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag - 1].available){
                                if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen > preSetPos[posIndex].teleVSlidePresetPos[polePosFlag - 1].vslideLen){
                                    vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 0.2); // down
                                }
                            }
                        }
                        else if (polePosFlag == 2){
                            if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag + 1].available){
                                if (preSetPos[posIndex].teleVSlidePresetPos[polePosFlag].vslideLen > preSetPos[posIndex].teleVSlidePresetPos[polePosFlag + 1].vslideLen){
                                    vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 0.2); // down
                                }
                            }
                        }
                        else{
                            vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 0.2); // down
                        }
*/



                        vSliderHoldConeFlag = false; // timing ok for pole 1,2
                        /*
                        if (vSliderSystem.getSliderLen() < 0.35){
                            vSliderSystem.sliderLenCtrl(0.45, 1.0);
                        }
                        */
                        vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE);
                        timeOut.reset();
                        while(opModeIsActive() && timeOut.milliseconds() < 2000 && Math.abs(vSliderSystem.getPanAngle()) > 30){
                            sleep(10);
                        }
                        /*
                        timeOut.reset();
                        while(opModeIsActive() && timeOut.milliseconds() < 2000 && vSliderSystem.getSliderLen() < 0.4){
                            sleep(10);
                        }*/


                        vSliderSystem.setTiltAngle(VHANDOVER_TILT_ANGLE);
                        vSliderSystem.setRotateAngle(VHANDOVER_ROTATE_ANGLE);
                        sleep(200);
                        vSliderHoldConeFlag = false;
                        vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
                        sleep(200);


                        // then down
                        vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
                        vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 1.0);
                        timeOut.reset();
                        while(opModeIsActive() && Math.abs(vSliderSystem.getSliderLen() - VSLIDER_HANDOVER_LEN) > 0.02 && timeOut.milliseconds() < 2000){
                            sleep(10);
                        }


                        vSliderHoldConeFlag = false;
                    }

                }
                else{
                    //only for low pole
                    //hard code here, regarding the setting in initialize pos data function, only at position 0,7,8,9,14,15
                    //tilt up and pan a little bit
                    sleep(300);
                    //vSliderSystem.setTiltAngle(0);
                    // vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN + 0.04, 1.0);
                    // sleep(200);

                    if (polePosFlag == 0 && (posIndex == 0 || posIndex == 9 || posIndex == 15)){
                        // left back low pole
                        vSliderSystem.setPanAngle(-25);
                        vSliderSystem.setRotateAngle(-90);
                    }
                    else if(polePosFlag == 3 && (posIndex == 7 || posIndex == 14 || posIndex == 8)){
                        vSliderSystem.setPanAngle(25);
                        vSliderSystem.setRotateAngle(90);
                    }
                    sleep(400);
                    vSliderSystem.setTiltAngle(-9);  // tilt down
                    // then rotate and down to low pole len
                    if (polePosFlag == 0 && (posIndex == 0 || posIndex == 9 || posIndex == 15)){
                        vSliderSystem.setRotateAngle(-95);
                    }
                    else if(polePosFlag == 3 && (posIndex == 7 || posIndex == 14 || posIndex == 8)){
                        vSliderSystem.setRotateAngle(95);
                    }
                    // then down
                    vSliderSystem.sliderLenCtrl(0.002, 1.0);
                    sleep(300);
                    // then pan to position
                    double targetPan = 0;
                    if (polePosFlag == 0 && (posIndex == 0 || posIndex == 9 || posIndex == 15)){
                        targetPan = -55;
                        vSliderSystem.setPanAngle(targetPan);
                    }
                    else if(polePosFlag == 3 && (posIndex == 7 || posIndex == 14 || posIndex == 8)){
                        targetPan = 55;
                        vSliderSystem.setPanAngle(targetPan);
                    }

                    timeOut.reset();
                    while( opModeIsActive() && timeOut.milliseconds() < 1500 && Math.abs(vSliderSystem.getPanAngle() - targetPan) > 2){
                        sleep(10);
                    }
                    // rotate
                    vSliderSystem.setRotateAngle(Math.signum(vSliderSystem.getRotateAngle()) * 185);
                    sleep(400);
                    //tilt down
                    vSliderSystem.setTiltAngle(-40);

                    vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
                    sleep(300);

                    // up in casae hig
                    double targetLen = 0.25;
                    vSliderSystem.sliderLenCtrl(targetLen,1.0);

                    timeOut.reset();
                    while(opModeIsActive() && timeOut.milliseconds() < 2000 && vSliderSystem.getSliderLen() < (targetLen - 0.02)){
                        sleep(10);
                    }
                    vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE);
                    timeOut.reset();
                    while(opModeIsActive() && timeOut.milliseconds() < 2000 && Math.abs(vSliderSystem.getPanAngle()) > 30){
                        sleep(10);
                    }

                    vSliderSystem.setRotateAngle(VHANDOVER_ROTATE_ANGLE);
                    sleep(200);
                    vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
                    sleep(200);
                    // then down
                    vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 1.0);
                    timeOut.reset();
                    while(opModeIsActive() && Math.abs(vSliderSystem.getSliderLen() - VSLIDER_HANDOVER_LEN) > 0.02 && timeOut.milliseconds() < 2000){
                        sleep(10);
                    }
                    vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
                    vSliderSystem.setTiltAngle(VHANDOVER_TILT_ANGLE);
                    vSliderHoldConeFlag = false;

                }
            }

        }

    }

    private static volatile int dropPosValue = 0;
    private static volatile boolean vSliderHoldConeFlag = false;
    private static volatile boolean dropConeTaskRunning = false;

    private void Drop_Cone_Thread_Task(int posFlag) {
        dropPosValue = posFlag;
        dropConeTaskRunning = true;
        new Thread(new Runnable() {
            @Override
            public void run() {
                dropConeTaskRunning = true;
                ElapsedTime timeOut = new ElapsedTime();
                if (opModeIsActive() && dropConeTaskRunning ) {
                    gotoDropPos_DropCone(dropPosValue);

                    timeOut.reset();
                    while(opModeIsActive() && timeOut.milliseconds() < 500){// && dropConeTaskRunning && Math.abs(vSliderSystem.getSliderLen() - VSLIDER_HANDOVER_LEN) > 0.02){
                        sleep(10);
                    }
                    dropConeTaskRunning = false;
                }
                dropConeTaskRunning = false;
            }
        }).start();
    }

    private void Drop_5Cone_Thread_Task(int posFlag) {
        dropPosValue = posFlag;
        dropConeTaskRunning = true;
        new Thread(new Runnable() {
            @Override
            public void run() {
                dropConeTaskRunning = true;
                ElapsedTime timeOut = new ElapsedTime();
                if (opModeIsActive() && dropConeTaskRunning ) {
                    gotoDropPos_DropCone5(dropPosValue);

                    timeOut.reset();
                    while(opModeIsActive() && timeOut.milliseconds() < 500){// && dropConeTaskRunning && Math.abs(vSliderSystem.getSliderLen() - VSLIDER_HANDOVER_LEN) > 0.02){
                        sleep(10);
                    }
                    dropConeTaskRunning = false;
                }
                dropConeTaskRunning = false;
            }
        }).start();
    }

    private void resetArm_Task() {

        new Thread(new Runnable() {
            @Override
            public void run() {
                hSliderSystem.setSlideBlockServoRelease();
                vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
                sleep(300);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
                sleep(100);
                vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE);
                vSliderSystem.setTiltAngle(VSLIDE_TILT_UP_ANGLE);
                hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN + 0.04, 1.0);
                sleep(200);

                hSliderSystem.setTilt1Angle(HSLIDE_TILT1_RESET_ANGLE);
                hSliderSystem.setTilt2Angle(HSLIDE_TILT2_RESET_ANGLE);
                hSliderSystem.sliderLenCtrl(0.01, 1.0);
            }
        }).start();
    }

    private void stopDropConeTask(){

        dropConeTaskRunning = false;
    }


    private void hSlide_GotoPickUp_Thread_Task(){

        new Thread(new Runnable() {
            @Override
            public void run() {
                sleep(100); // wait for v-slider up
                updateHSlideData();
                if (hPickupLen < HSLIDEPICKUPLEN_2TILES){

                    hSliderSystem.sliderLenCtrl(hPickupLen, 1.0);  // keep the claw out of substation, no need for auto
                    sleep(400);
                    hSliderSystem.sliderLenCtrl(0.01, 1.0);
                }
                else{
                    hSliderSystem.sliderLenCtrl(hPickupLen - hPickUpBackLen, 1.0);  // keep the claw out of substation, no need for auto
                    sleep(400);

                }

                hSliderSystem.setTilt1Angle(hPickupTilt1Angle);
                hSliderSystem.setTilt2Angle(hPickupTilt2Angle);
                hSliderSystem.setPanAngle(0);

            }
        }).start();
    }

    private void hSlide_GotoPickUp5_Thread_Task(){

        new Thread(new Runnable() {
            @Override
            public void run() {
                sleep(100); // wait for v-slider up
                telemetry.addData("In Dpad Function Level", " %d", pickLevel);
                telemetry.update();
                hPickupLen = hPickupLen;
                hPickupTilt1Angle = hPickup5Tilt1Angle[pickLevel];
                hPickupTilt2Angle = hPickup5Tilt2Angle[pickLevel];
                hPickUpPanAngle = hPickup5Cone_Pan_Angle;
                hSliderSystem.sliderLenCtrl(hPickupLen, 1.0);
                hSliderSystem.setTilt1Angle(hPickupTilt1Angle);
                hSliderSystem.setTilt2Angle(hPickupTilt2Angle);
                hSliderSystem.setPanAngle(hPickUpPanAngle);
                sleep(400);
            }
        }).start();
    }


    private void pickUp_HandOver_Cone_Thread_Task() {
        pickUpConeTaskRunning = true;
        handOverCone = false;
        new Thread(new Runnable() {
            @Override

            public void run() {
                handOverCone = false;
                pickUpConeTaskRunning = true;
                ElapsedTime timeOut = new ElapsedTime();
                robotPosValue = getPosIndex(); // to change the hPickUpTiltAngle, hPickUpPanAngle, hPickUpLen
                if (robotPosValue >= 0 && robotPosValue < 16) {
                    hPickupLen = preSetPos[robotPosValue].hPickupLen;
                    hPickUpPanAngle = preSetPos[robotPosValue].hPickupPan;
                    hPickupTilt1Angle = preSetPos[robotPosValue].hPickupTilt1;
                    hPickupTilt2Angle = preSetPos[robotPosValue].hPickupTilt2;

                    pickUpConeDone = false;
                    if (opModeIsActive() && !pickUpConeDone) {
                        handOverCone = false;  // hold cone
                        hSliderSystem.setTilt1Angle(hPickupTilt1Angle);  //hPickupTiltAngle
                        hSliderSystem.setTilt2Angle(hPickupTilt2Angle);  //hPickupTiltAngle
                        hSliderSystem.setPanAngle(hPickUpPanAngle);
                        hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
                        while (opModeIsActive() && vSliderHoldConeFlag) {
                            // wait for v-slider release the cone
                            sleep(10);
                        }

                        hSliderSystem.sliderLenCtrl(hPickupLen, 1.0);   //0.8


                        timeOut.reset();
                        while (opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - hPickupLen) > 0.02 && timeOut.milliseconds() < 1000) {
                            sleep(10);
                        }


                        // grab cone
                        //set block here
                        hSliderSystem.setSlideBlock();
                        hSliderSystem.setClawAngle(HSliderSystem.CLAW_CLOSE_ANGLE);
                        sleep(350);  // wait for claw hold cone

                        // lift a little bit
                        hSliderSystem.setTilt1Angle(HHANDOVER_TILT1_ANGLE);
                        hSliderSystem.setTilt2Angle(HHANDOVER_TILT2_ANGLE);
                        hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN, 1.0);  // may need wait for tilt servo, if yes, reduce the power
                        hSliderSystem.setPanAngle(HHANDOVER_PAN_ANGLE);
                        timeOut.reset();
                        while (opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - HSLIDER_HANDOVER_LEN) > 0.02 && timeOut.milliseconds() < 3000) {
                            sleep(10);
                        }

                        // to handover


                        if (hPickupLen < HSLIDEPICKUPLEN_2TILES) {
                            sleep(500);  // need wait for tilt servo move, since the slide only need move very short distance6
                        } else if (hPickupLen < HSLIDEPICKUPLEN_3TILES) {
                            sleep(100);
                        }


                        handOverCone = true;
                    }
                }

                pickUpConeTaskRunning = false;
            }
        }).start();

    }

    private void pickUpNow_HandOver_Cone_Thread_Task() {
        pickUpConeTaskRunning = true;
        handOverCone = false;
        new Thread(new Runnable() {
            @Override

            public void run() {
                handOverCone = false;
                pickUpConeTaskRunning = true;
                ElapsedTime timeOut = new ElapsedTime();

                // grab cone
                //set block here
                hSliderSystem.setSlideBlock();
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_CLOSE_ANGLE);
                sleep(350);  // wait for claw hold cone

                // lift a little bit
                hSliderSystem.setTilt1Angle(HHANDOVER_TILT1_ANGLE);
                hSliderSystem.setTilt2Angle(HHANDOVER_TILT2_ANGLE);
                hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN, 1.0);  // may need wait for tilt servo, if yes, reduce the power
                hSliderSystem.setPanAngle(HHANDOVER_PAN_ANGLE);
                timeOut.reset();
                while(opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - HSLIDER_HANDOVER_LEN) > 0.02 && timeOut.milliseconds() < 3000){
                    sleep(10);
                }

                // to handover


                if (hPickupLen < HSLIDEPICKUPLEN_2TILES){
                    sleep(500);  // need wait for tilt servo move, since the slide only need move very short distance6
                }else if(hPickupLen < HSLIDEPICKUPLEN_3TILES){
                    sleep(100);
                }


                handOverCone = true;


                pickUpConeTaskRunning = false;
            }
        }).start();

    }

    private void pickUp5_HandOver_Cone_Thread_Task() {
        pickUpConeTaskRunning = true;
        handOverCone = false;
        new Thread(new Runnable() {
            @Override
            public void run() {
                handOverCone = false;
                pickUpConeTaskRunning = true;
                ElapsedTime timeOut = new ElapsedTime();
                pickUpConeDone = false;
                if (opModeIsActive() && !pickUpConeDone ) {
                    handOverCone = false;  // hold cone
                    hSliderSystem.setTilt1Angle(hPickupTilt1Angle);  //hPickupTiltAngle
                    hSliderSystem.setTilt2Angle(hPickupTilt2Angle);  //hPickupTiltAngle
                    hSliderSystem.setPanAngle(hPickUpPanAngle);
                    hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
                    hSliderSystem.sliderLenCtrl(hPickupLen, 1.0);   //0.8

                    timeOut.reset();
                    while( opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - hPickupLen) > 0.02 && timeOut.milliseconds() < 3000){
                        sleep(10);
                    }

                    // grab cone
                    //set block here
                    hSliderSystem.setSlideBlock();
                    hSliderSystem.setClawAngle(HSliderSystem.CLAW_CLOSE_ANGLE);
                    sleep(350);  // wait for claw hold cone

                    // lift a little bit
                    hSliderSystem.setTilt1Angle(HHANDOVER_TILT1_ANGLE);
                    hSliderSystem.setTilt2Angle(HHANDOVER_TILT2_ANGLE);
                    sleep(200);
                    hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN, 1.0);  // may need wait for tilt servo, if yes, reduce the power
                    hSliderSystem.setPanAngle(HHANDOVER_PAN_ANGLE);
                    timeOut.reset();
                    while(opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - HSLIDER_HANDOVER_LEN) > 0.02 && timeOut.milliseconds() < 3000){
                        sleep(10);
                    }

                    // to handover


                    if (hPickupLen < HSLIDEPICKUPLEN_2TILES){
                        sleep(500);  // need wait for tilt servo move, since the slide only need move very short distance6
                    }else if(hPickupLen < HSLIDEPICKUPLEN_3TILES){
                        sleep(100);
                    }


                    handOverCone = true;
                }

                pickUpConeTaskRunning = false;
            }
        }).start();

    }

  private void stopPickUpTask(){
        pickUpConeTaskRunning = false;
   }





    private boolean readSDCardFile()
    {
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FIRST/ParkPos");
        File file = new File(dir,"parkingPos.txt");

        StringBuilder text = new StringBuilder();

        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;

            int lineCnt = 0;
            while ((line = br.readLine()) != null) {
                if (lineCnt == 0){
                    robotPos.x = Double.parseDouble(line);
                }else if(lineCnt == 1){
                    robotPos.y = Double.parseDouble(line);
                }else if(lineCnt == 2){
                    robotPos.angle = Double.parseDouble(line);
                }else if(lineCnt == 3){
                    posFlag = line;
                }

                lineCnt ++;
            }
            telemetry.update();
            br.close();
            return true;
        }
        catch (IOException e) {
            //You'll need to add proper error handling here
            return false;
        }
    }



    private void resetArms(){
        hSliderSystem.setSlideBlockServoRelease();
        vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
        sleep(150);
        hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
        vSliderSystem.setTiltAngle(VSLIDE_TILT_UP_ANGLE);
        vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE);
        sleep(200);
        hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN + 0.05, 1.0);
        sleep(200);
        hSliderSystem.setTilt1Angle(HSLIDE_TILT1_RESET_ANGLE);
        hSliderSystem.setTilt2Angle(HSLIDE_TILT2_RESET_ANGLE);
        hSliderSystem.setClawAngle(HSliderSystem.CLAW_CLOSE_ANGLE);
        hSliderSystem.sliderLenCtrl(0.01, 1.0);
        sleep(200);

    }

    private void teleFirstResetArms(){
        hSliderSystem.setSlideBlockServoRelease();
        vSliderSystem.setTiltAngle(VSLIDE_TILT_UP_ANGLE);
        vSliderSystem.setRotateAngle(0);
        vSliderSystem.sliderLenCtrl(0.1, 0.2);
        vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
        sleep(200);
        vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
        vSliderSystem.setPanAngle(0);
        sleep(200);
        hSliderSystem.sliderLenCtrl(0.01, 1.0);
        hSliderSystem.setClawAngle(HSliderSystem.CLAW_CLOSE_ANGLE);

        vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE);

        hSliderSystem.setTilt1Angle(HSLIDE_TILT1_RESET_ANGLE);
        hSliderSystem.setTilt2Angle(HSLIDE_TILT2_RESET_ANGLE);
        hSliderSystem.setPanAngle(0);
    }




    private void initializePosData(){

        for (int i = 0 ; i < posNum; i++){
            preSetPos[i] = new TeleOpPresetData();
            preSetPos[i].hPickupTilt1 = HPICKUP_TILT1;
            preSetPos[i].hPickupTilt2 = HPICKUP_TILT2;

            preSetPos[i].hPickupLen = HSLIDEPICKUPLEN_1TILE; // for default len
            preSetPos[i].hPickupPan = HSLIDEPICKUP_PANANGLE;
            preSetPos[i].hSlideMove = true;   // if false, do not move hSlider ofr position 8 ~ 15

            for (int j = 0; j < 4; j++){
                preSetPos[i].teleVSlidePresetPos[j] = new TeleVSlidePresetPos();

                preSetPos[i].teleVSlidePresetPos[j].vslideLen = VSLIDELEN_HPOLE;
                preSetPos[i].teleVSlidePresetPos[j].vtiltAngle = VSLIDE_TILTANGLE;
                preSetPos[i].teleVSlidePresetPos[j].available = true;  // default is true, later we modified it based position value


                ///////////// robot  -- pole ////////////////////////////////////////
                //      Pole(1)       (Front)   Pole(2)
                //              (Left)Robot(Right)
                //      Pole(0)       (Back)    Pole(3)

                if (j == 0){ // left back pole
                    preSetPos[i].teleVSlidePresetPos[j].vPanAngle = -55 + VSLIDE_PAN_PUSH_ANGLE;
                    preSetPos[i].teleVSlidePresetPos[j].vRotateAngle = -98;
                }else if (j == 1){ // left front pole
                    preSetPos[i].teleVSlidePresetPos[j].vPanAngle = -150 + VSLIDE_PAN_PUSH_ANGLE;
                    preSetPos[i].teleVSlidePresetPos[j].vRotateAngle = -98;
                }else if (j == 2){ // right front pole
                    preSetPos[i].teleVSlidePresetPos[j].vPanAngle = 150 - VSLIDE_PAN_PUSH_ANGLE;
                    preSetPos[i].teleVSlidePresetPos[j].vRotateAngle = 98;
                }else if (j == 3){ // right back pole
                    preSetPos[i].teleVSlidePresetPos[j].vPanAngle = 55 - VSLIDE_PAN_PUSH_ANGLE;
                    preSetPos[i].teleVSlidePresetPos[j].vRotateAngle = 98;
                }
            }

            //based on position i value, change the pole data
            // position
            // tile coordinate
            //  A Row Tile         blue substation
            //  i = 11(B2)      i = 3(B3)       i = 4(B4)       i = 12(B5)
            //  i = 10(C2)      i = 2(C3)       i = 5(C4)       i = 13(C5)
            //  i = 9(D2)       i = 1(D3)       i = 6(D4)       i = 14(D5)
            //  i = 8(E2)       i = 0(E3)       i = 7(E4)       i = 15(E5)
            //   F Row Tile         Red Substation

             // pos 1 hard code position
            double presetX = -0.45;
            double presetY = 0.3;
            double presetA = 0;
            double tileSize = 0.6;
            if (i == 0){
                preSetPos[i].posX = presetX - tileSize;  /// back 1 tile
                preSetPos[i].posY = presetY;
                preSetPos[i].posA = presetA;

                preSetPos[i].teleVSlidePresetPos[0].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[1].vslideLen = VSLIDELEN_MPOLE;
                preSetPos[i].teleVSlidePresetPos[2].vslideLen = VSLIDELEN_HPOLE;
                preSetPos[i].teleVSlidePresetPos[3].available = false;
                preSetPos[i].hPickupLen = HSLIDEPICKUPLEN_1TILE;
                preSetPos[i].hSlideMove = true;
                preSetPos[i].defaultPolePos = 2;
            }else if (i == 1){
                preSetPos[i].posX = presetX;  ///
                preSetPos[i].posY = presetY;
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[0].vslideLen = VSLIDELEN_MPOLE;
                preSetPos[i].teleVSlidePresetPos[1].vslideLen = VSLIDELEN_HPOLE;
                preSetPos[i].teleVSlidePresetPos[3].vslideLen = VSLIDELEN_HPOLE;
                preSetPos[i].teleVSlidePresetPos[2].available = false;
                preSetPos[i].hPickupLen = HSLIDEPICKUPLEN_2TILES;
                preSetPos[i].hSlideMove = true;
                preSetPos[i].defaultPolePos = 3;
            }else if (i == 2){
                preSetPos[i].posX = presetX + tileSize;  /// forward 1 tile
                preSetPos[i].posY = presetY;
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[0].vslideLen = VSLIDELEN_HPOLE;
                preSetPos[i].teleVSlidePresetPos[1].vslideLen = VSLIDELEN_MPOLE ;
                preSetPos[i].teleVSlidePresetPos[2].vslideLen = VSLIDELEN_HPOLE ;
                preSetPos[i].teleVSlidePresetPos[3].available = false;
                preSetPos[i].hPickupLen = HSLIDEPICKUPLEN_3TILES;
                preSetPos[i].hSlideMove = true;
                preSetPos[i].defaultPolePos = 0;
            }else if (i == 3){
                preSetPos[i].posX = presetX + 2 * tileSize;  /// forward 1 tile
                preSetPos[i].posY = presetY;
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[0].vslideLen = VSLIDELEN_MPOLE;
                preSetPos[i].teleVSlidePresetPos[1].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[3].vslideLen = VSLIDELEN_HPOLE;
                preSetPos[i].teleVSlidePresetPos[2].available = false;
                preSetPos[i].teleVSlidePresetPos[1].available = true;  // low pole

                preSetPos[i].hPickupLen = HSLIDEPICKUPLEN_1TILE;
                preSetPos[i].hSlideMove = false;
                preSetPos[i].defaultPolePos = 3;
            }else if (i == 4){
                preSetPos[i].posX = presetX + 2 * tileSize;  /// forward 1 tile
                preSetPos[i].posY = presetY - tileSize;  // left 1 tile
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[0].vslideLen = VSLIDELEN_HPOLE;
                preSetPos[i].teleVSlidePresetPos[2].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[3].vslideLen = VSLIDELEN_MPOLE;
                preSetPos[i].teleVSlidePresetPos[1].available = false;
                preSetPos[i].teleVSlidePresetPos[2].available = true; // add low pole
                preSetPos[i].hPickupLen = HSLIDEPICKUPLEN_1TILE;
                preSetPos[i].hSlideMove = false;
                preSetPos[i].defaultPolePos = 0;
            }else if (i == 5){
                preSetPos[i].posX = presetX + 1 * tileSize;  /// forward 1 tile
                preSetPos[i].posY = presetY - tileSize;  // left 1 tile
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[1].vslideLen = VSLIDELEN_HPOLE ;
                preSetPos[i].teleVSlidePresetPos[2].vslideLen = VSLIDELEN_MPOLE ;
                preSetPos[i].teleVSlidePresetPos[3].vslideLen = VSLIDELEN_HPOLE;
                preSetPos[i].teleVSlidePresetPos[0].available = false;
                preSetPos[i].hPickupLen = HSLIDEPICKUPLEN_3TILES;
                preSetPos[i].hSlideMove = true;
                preSetPos[i].defaultPolePos = 3;
            }else if (i == 6){
                preSetPos[i].posX = presetX ;  /// forward 1 tile
                preSetPos[i].posY = presetY - tileSize;  // left 1 tile
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[0].vslideLen = VSLIDELEN_HPOLE;
                preSetPos[i].teleVSlidePresetPos[2].vslideLen = VSLIDELEN_HPOLE ;
                preSetPos[i].teleVSlidePresetPos[3].vslideLen = VSLIDELEN_MPOLE;
                preSetPos[i].teleVSlidePresetPos[1].available = false;
                preSetPos[i].hPickupLen = HSLIDEPICKUPLEN_2TILES;
                preSetPos[i].hSlideMove = true;
                preSetPos[i].defaultPolePos = 0;
            }else if (i == 7){
                preSetPos[i].posX = presetX - tileSize;  /// forward 1 tile
                preSetPos[i].posY = presetY - tileSize;  // left 1 tile
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[1].vslideLen = VSLIDELEN_HPOLE ;
                preSetPos[i].teleVSlidePresetPos[2].vslideLen = VSLIDELEN_MPOLE;
                preSetPos[i].teleVSlidePresetPos[3].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[0].available = false;

                preSetPos[i].hPickupLen = HSLIDEPICKUPLEN_1TILE;
                preSetPos[i].hSlideMove = true;
                preSetPos[i].defaultPolePos = 1;
            }else if (i == 8){
                preSetPos[i].posX = presetX - tileSize;  /// forward 1 tile
                preSetPos[i].posY = presetY + tileSize;  // left 1 tile
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[1].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[2].vslideLen = VSLIDELEN_MPOLE ;
                preSetPos[i].teleVSlidePresetPos[3].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[0].available = false;
                preSetPos[i].defaultPolePos = 2;

            }else if (i == 9){
                preSetPos[i].posX = presetX;  /// forward 1 tile
                preSetPos[i].posY = presetY + tileSize;  // left 1 tile
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[0].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[2].vslideLen = VSLIDELEN_HPOLE ;
                preSetPos[i].teleVSlidePresetPos[3].vslideLen = VSLIDELEN_MPOLE;
                preSetPos[i].teleVSlidePresetPos[1].available = false;
                preSetPos[i].defaultPolePos = 2;

            }else if (i == 10){
                preSetPos[i].posX = presetX + tileSize;  /// forward 1 tile
                preSetPos[i].posY = presetY + tileSize;  // left 1 tile
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[1].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[2].vslideLen = VSLIDELEN_MPOLE;
                preSetPos[i].teleVSlidePresetPos[3].vslideLen = VSLIDELEN_HPOLE;
                preSetPos[i].teleVSlidePresetPos[0].available = false;
                preSetPos[i].teleVSlidePresetPos[1].available = true;
                preSetPos[i].defaultPolePos = 3;

            }else if (i == 11){
                preSetPos[i].posX = presetX + 2 * tileSize;  /// forward 1 tile
                preSetPos[i].posY = presetY + tileSize;  // left 1 tile
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[0].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[2].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[3].vslideLen = VSLIDELEN_MPOLE;
                preSetPos[i].teleVSlidePresetPos[1].available = false;
                preSetPos[i].teleVSlidePresetPos[2].available = true;
                preSetPos[i].teleVSlidePresetPos[0].available = true;
                preSetPos[i].defaultPolePos = 3;

            }else if (i == 12){
                preSetPos[i].posX = presetX + 2 * tileSize;  /// forward 1 tile
                preSetPos[i].posY = presetY - 2 * tileSize;  // left 1 tile
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[0].vslideLen = VSLIDELEN_MPOLE;
                preSetPos[i].teleVSlidePresetPos[1].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[3].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[2].available = false;
                preSetPos[i].teleVSlidePresetPos[1].available = true;
                preSetPos[i].teleVSlidePresetPos[3].available = true;
                preSetPos[i].defaultPolePos = 0;

            }else if (i == 13){
                preSetPos[i].posX = presetX + 1 * tileSize;  /// forward 1 tile
                preSetPos[i].posY = presetY - 2 * tileSize;  // left 1 tile
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[0].vslideLen = VSLIDELEN_HPOLE;
                preSetPos[i].teleVSlidePresetPos[1].vslideLen = VSLIDELEN_MPOLE;
                preSetPos[i].teleVSlidePresetPos[2].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[3].available = false;
                preSetPos[i].teleVSlidePresetPos[2].available = true;
                preSetPos[i].defaultPolePos = 0;
            }else if (i == 14){
                preSetPos[i].posX = presetX ;  /// forward 1 tile
                preSetPos[i].posY = presetY - 2 * tileSize;  // left 1 tile
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[0].vslideLen = VSLIDELEN_MPOLE;
                preSetPos[i].teleVSlidePresetPos[1].vslideLen = VSLIDELEN_HPOLE;
                preSetPos[i].teleVSlidePresetPos[3].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[2].available = false;
                preSetPos[i].defaultPolePos = 1;
            }else if (i == 15){
                preSetPos[i].posX = presetX - tileSize;  /// forward 1 tile
                preSetPos[i].posY = presetY - 2 * tileSize;  // left 1 tile
                preSetPos[i].posA = presetA;
                preSetPos[i].teleVSlidePresetPos[0].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[1].vslideLen = VSLIDELEN_MPOLE;
                preSetPos[i].teleVSlidePresetPos[2].vslideLen = VSLIDELEN_LPOLE;
                preSetPos[i].teleVSlidePresetPos[3].available = false;
                preSetPos[i].defaultPolePos = 1;
            }

            if (i >= 8 && i <= 15){
                preSetPos[i].hPickupLen = HSLIDEPICKUPLEN_1TILE;
                preSetPos[i].hSlideMove = false;
            }
        }
   }


   private int getPosIndex(){
        //based on robot position data, to get which tile the robot at

       // robot global position front (X+), left(Y+)
       // (0,0) is at field center(ground junction)

       int posIndex = -1;
       robotPos = odometry.getRobotPosition();
       if (robotPos.y <= 0.6 && robotPos.y > 0){
           if (robotPos.x >= -1.2 && robotPos.x <= -0.6){
               posIndex = 0;  //E3
           } else if(robotPos.x > -0.6 && robotPos.x <= 0){
               posIndex = 1;    //D3
           }else if(robotPos.x > 0 && robotPos.x <= 0.6){
               posIndex = 2;    //C3
           }else if(robotPos.x > 0.6 && robotPos.x <= 1.2){
               posIndex = 3;    //B3
           }
       } else if (robotPos.y < 1.2 && robotPos.y > 0.6){
           if (robotPos.x >= -1.2 && robotPos.x <= -0.6){
               posIndex = 8;  //E2
           } else if(robotPos.x > -0.6 && robotPos.x <= 0){
               posIndex = 9;    //D2
           }else if(robotPos.x > 0 && robotPos.x <= 0.6){
               posIndex = 10;    //C2
           }else if(robotPos.x > 0.6 && robotPos.x <= 1.2){
               posIndex = 11;    //B2
           }
       } else if (robotPos.y <= 0 && robotPos.y > -0.6){
           if (robotPos.x >= -1.2 && robotPos.x <= -0.6){
               posIndex = 7;  //E4
           } else if(robotPos.x > -0.6 && robotPos.x <= 0){
               posIndex = 6;    //D4
           }else if(robotPos.x > 0 && robotPos.x <= 0.6){
               posIndex = 5;    //C4
           }else if(robotPos.x > 0.6 && robotPos.x <= 1.2){
               posIndex = 4;    //B4
           }
       } else if (robotPos.y <= -0.6 && robotPos.y > -1.2){
           if (robotPos.x >= -1.2 && robotPos.x <= -0.6){
               posIndex = 15;  //E5
           } else if(robotPos.x > -0.6 && robotPos.x <= 0){
               posIndex = 14;    //D5
           }else if(robotPos.x > 0 && robotPos.x <= 0.6){
               posIndex = 13;    //C5
           }else if(robotPos.x > 0.6 && robotPos.x <= 1.2){
               posIndex = 12;    //B5
           }
       }


       return posIndex;
   }




}
