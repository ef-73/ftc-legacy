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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.List;


@Autonomous(name="Auto Only Parking With Preload")
@Disabled

public class AutoOnlyParking extends LinearOpMode {
    //for pick up
    private double HPICKUP_5_LEN = 0.73;
    private double HPICKUP_PAN_ANGLE = -2.8;
    private double[] hPickup5Tilt1Angle = {-51.31 + 2.0,-39.71 - 2.5,-25.38 - 3.0,-13.47 - 2.0,2.43 + 0.0};
    private double[] hPickup5Tilt2Angle = {-8.01 - 2.0,-15.58 + 2.5,-29.88 + 3.0,-41.59 + 2.0,-60.83 - 0.0};
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
    private static final double VSLIDELEN_HPOLE = 0.42;  // 0.44  for high pole

    private static final double VSLIDE_TILTANGLE = 5.5;  // drop tilt angle



    private static final double HSLIDEPICKUPLEN_1TILE = 0.24;
    private static final double HSLIDEPICKUPLEN_2TILES = 0.84;
    private static final double HSLIDEPICKUPLEN_3TILES = 1.402;

    private static final double HPICKUP_TILT1 = -81;
    private static final double HPICKUP_TILT2 = 4.05;
    private static final double HSLIDEPICKUP_PANANGLE = 0;
    private static final double HSLIDERPICKUP_PANANGLE = -6.5;

    // for pickup, need re-initialize in the code
    private static volatile double hPickupLen = 0;   // for level-5 cone
    private static volatile double hPickupTilt1Angle = 0;
    private static volatile double hPickupTilt2Angle = 0;
    private static volatile double hPickUpPanAngle = HSLIDERPICKUP_PANANGLE;
    // for v slider drop
    private static volatile double vDropLen = VSLIDELEN_MPOLE;
    private static volatile double vDropTiltAngle = VSLIDE_TILTANGLE;  //12
    private static volatile double vDropPanAngle = 55;
    private static volatile double vDropRotateAngle = 98;
    private final double VSLIDE_PAN_PUSH_ANGLE = 35;  // 30

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
            targetY = STARTY - 0.48 ;
            targetA = -90;
            vDropPanAngle = 25;
            vDropRotateAngle = 98;

        }else{
            telemetry.addLine("You Choose Right Side");
            STARTX = -1.54;
            STARTY = -0.9 ;  // middle of tile
            STARTA = 0;
            targetX = STARTX + 1.24;
            targetY = STARTY + 0.48 ;
            targetA = 90;
            vDropPanAngle = -25;
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

            timer.reset();
            vSliderSystem.setClawAngle(vSliderSystem.CLAW_CLOSE_ANGLE);
            vSliderSystem.setTiltAngle(-60);
            vSliderSystem.setRotateAngle(-28.79);
            pickLevel = 5;

            parkingPos = cameraObjectDetector.detectID(150);//  id start from 1,2,3

            if (parkingPos < 0 ) parkingPos = 3;

            if (sideChoice == 1){
                vDropRotateAngle = 98;
                vDropPanAngle = 115;
                vDropLen = VSLIDELEN_MPOLE;

                if (parkingPos == 1) {
                    parkPosition.x = STARTX + 0.65;
                    parkPosition.y = STARTY + 0.56;
                    parkPosition.angle = 0;
                } else if (parkingPos == 2) {
                    parkPosition.x = STARTX + 0.65;
                    parkPosition.y = STARTY - 0.05;
                    parkPosition.angle = 0;
                } else {
                    parkPosition.x = STARTX + 0.65;
                    parkPosition.y = STARTY - 0.65;
                    parkPosition.angle = 0;
                }
            }
            else{
                vDropRotateAngle = -98;
                vDropPanAngle = -115;
                vDropLen = VSLIDELEN_MPOLE;
                if (parkingPos == 1) {
                    parkPosition.x = STARTX + 0.65;
                    parkPosition.y = STARTY + 0.56;
                    parkPosition.angle = 0;

                } else if (parkingPos == 2) {
                    parkPosition.x = STARTX + 0.65;
                    parkPosition.y = STARTY - 0.05;
                    parkPosition.angle = 0;
                } else {
                    parkPosition.x = STARTX + 0.65;
                    parkPosition.y = STARTY - 0.65;
                    parkPosition.angle = 0;
                }
            }


            writeSDCardFile(parkPosition);

            robotPos = odometry.getRobotPosition();

            //go forward, then move forward a little bit
            int stepCnt = 0;
            targetVel = 0.7;
            chassisController.p2pDrive(new RobotPosition(STARTX + 0.5, STARTY, STARTA), targetVel, turnVel, 0.01, 1, 0.01, true, 2000);
            while (opModeIsActive() && !chassisController.p2pIsSettled()) {
                if (robotPos.x > STARTX + 0.15  && stepCnt == 0 ){
                    stepCnt = 1;
                    vSliderSystem.sliderLenCtrl(vDropLen, 1.0);
                    vSliderSystem.setTiltAngle(vDropTiltAngle);


                }

                sleep(10);
            }
            vSliderSystem.setRotateAngle(vDropRotateAngle);
            vSliderSystem.setPanAngle(vDropPanAngle);
            sleep(400);
            chassisController.stopP2PTask();


            vSlideDropPreLoadCone();

            timeOut.reset();
            while(opModeIsActive() && timeOut.milliseconds() < 4000){
                if (!vSliderHoldConeFlag) break;
                sleep(10);
            }
            sleep(300);


            resetHorizontalSlide();
            sleep(200);
            resetVerticalSlide();


/////////////////////////////////

                turnVel = 1.0;
                targetVel = 1.0;
            chassisController.p2pDrive(new RobotPosition(STARTX + 0.65, STARTY, STARTA), targetVel, turnVel, 0.01, 1, 0.01, true, 2000);
            while (opModeIsActive() && !chassisController.p2pIsSettled()) {
                sleep(10);
            }
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

                vSliderSystem.sliderLenCtrl(0.01, 0.5);
                sleep(200);

                updateTelemetry();

                //finish, stop all
                chassisController.stopAllThread();

                chassis.stopRobot();

                //write current position, may only use with tracking wheel
                robotPos = odometry.getRobotPosition();
                writeSDCardFile(robotPos);

                odometry.stopThread();
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
                vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
                vSliderSystem.setRotateAngle(-90);
                vSliderSystem.setPanAngle(-20);
                sleep(150);
                vSliderSystem.setTiltAngle(-50);

                sleep(200);
                vSliderSystem.sliderLenCtrl(0.01, 0.2);

                hSliderSystem.setSlideBlockServoRelease(); // must before the h-slide moving in

        }
        else{

                vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
                vSliderSystem.setRotateAngle(-90);
                vSliderSystem.setPanAngle(-20);
                sleep(150);

                vSliderSystem.setTiltAngle(-50);

                sleep(200);
                vSliderSystem.sliderLenCtrl(0.01, 0.2);


        }

    }

    private void resetHorizontalSlide(){
        if (opModeIsActive()){

            if (opModeIsActive()){

                hSliderSystem.sliderLenCtrl(0.01, 1.0);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_CLOSE_ANGLE);


                hSliderSystem.setTilt1Angle(81);
                hSliderSystem.setTilt2Angle(17);

                hSliderSystem.setPanAngle(-35);

                
            }

        }

    }
    private boolean detectConeTaskRunning = false;


    private void vSlideDropPreLoadCone(){
        if(opModeIsActive()) {
            double targetPanAngle = vDropPanAngle + Math.signum(vDropPanAngle) * (VSLIDE_PAN_PUSH_ANGLE);
            vSliderSystem.setPanAngle(targetPanAngle, 400);  // 160
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
            sleep(200);

            vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);

            if (vSliderSystem.getSliderLen() < 0.35) {
                vSliderSystem.sliderLenCtrl(0.35, 1.0);
            }


            vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE);
            timeOut.reset();
            while (opModeIsActive() && timeOut.milliseconds() < 2000 && Math.abs(vSliderSystem.getPanAngle()) > 15) {
                sleep(10);
            }
            timeOut.reset();
            while (opModeIsActive() && timeOut.milliseconds() < 2000 && vSliderSystem.getSliderLen() < 0.3) {
                sleep(10);
            }
            vSliderSystem.setTiltAngle(VHANDOVER_TILT_ANGLE);
            vSliderSystem.setRotateAngle(VHANDOVER_ROTATE_ANGLE);
            sleep(200);
            vSliderSystem.setClawAngle(VHANDOVER_CLAW_ANGLE);
            sleep(200);

            vSliderHoldConeFlag = false;
        }
    }




}