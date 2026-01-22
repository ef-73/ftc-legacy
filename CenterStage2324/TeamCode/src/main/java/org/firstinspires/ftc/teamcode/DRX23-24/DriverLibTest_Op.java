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

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="DriveLibTest-Demo")
@Disabled
@Config
public class DriverLibTest_Op extends LinearOpMode {
    ChassisSystem chassis = null;
    Odometry odometry = null;
    MecanumDriveLib chassisController = null;
    RobotPosition robotPos = new RobotPosition(0,0,0);

    public static double targetX = 0.6;
    public static double targetY = 0;
    public static double targetA = 0;
    public static double targetVel = 1.0;

    public static double turnAngle = 90;
    public static double turnVel = 1.0;
    FtcDashboard dashboard = null;
    Telemetry dashboardTelemetry = null;

    @Override
    public void runOpMode(){
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        // could save time for reading encoder
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }




        chassis = new ChassisSystem(this);

        sleep(1000);
        telemetry.addLine("odometry is setup");
        telemetry.update();

        odometry = new Odometry(this,chassis, robotPos, true);


        sleep(1000);
        chassisController = new MecanumDriveLib(this, odometry,chassis);

        telemetry.addData("Working Mode", "waiting for start");
        telemetry.update();

        waitForStart();
        // reset claw to right position first, only for this tele demo program

        while (opModeIsActive()){

            if (chassisController.p2pIsSettled() && chassisController.turnIsSettled()) {
                chassis.chassisTeleOp();
            }

            robotPos = odometry.getRobotPosition();
            if (gamepad1.y)
            {

                chassisController.p2pDrive(new RobotPosition(targetX,targetY,targetA),targetVel,turnVel,0.0,0.01,1,0,true,10000);
                while(opModeIsActive() && !chassisController.p2pIsSettled()){
                    dashBoardUpdate();
                    sleep(10);
                }

            }

            if (gamepad1.x)
            {
                int loopCnt = 0;
                while(loopCnt < 5){
                    loopCnt ++;
                    chassisController.p2pDrive(new RobotPosition(0.45,-0.35,0),targetVel,turnVel,0.0,0.1,5,1.0,false,10000);
                    while(opModeIsActive() && !chassisController.p2pIsSettled()){
                        dashBoardUpdate();
                        sleep(10);
                    }
                    chassisController.p2pDrive(new RobotPosition(1.6,-0.30,0),targetVel,turnVel,0.0,0.1,5,1.0,false,10000);
                    while(opModeIsActive() && !chassisController.p2pIsSettled()){
                        dashBoardUpdate();
                        sleep(10);
                    }
                    chassisController.p2pDrive(new RobotPosition(2.4,0.3,0),targetVel,turnVel,0.0,0.02,1,0.0,true,10000);
                    while(opModeIsActive() && !chassisController.p2pIsSettled()){
                        dashBoardUpdate();
                        sleep(10);
                    }
                    sleep(500);
                    odometry.updateRobotAngleByIMU();

                    chassisController.p2pDrive(new RobotPosition(1.6,-0.35,0),targetVel,turnVel,0.0,0.1,5,1.0,false,10000);
                    while(opModeIsActive() && !chassisController.p2pIsSettled()){
                        dashBoardUpdate();
                        sleep(10);
                    }
                    chassisController.p2pDrive(new RobotPosition(0.45,-0.30,0),targetVel,turnVel,0.0,0.1,5,1.0,false,10000);
                    while(opModeIsActive() && !chassisController.p2pIsSettled()){
                        dashBoardUpdate();
                        sleep(10);
                    }
                    chassisController.p2pDrive(new RobotPosition(0,0.0,0),targetVel,turnVel,0.0,0.02,1,0.0,true,10000);
                    while(opModeIsActive() && !chassisController.p2pIsSettled()){
                        dashBoardUpdate();
                        sleep(10);
                    }
                    sleep(500);
                    odometry.updateRobotAngleByIMU();
                }



            }

            if (gamepad1.a)
            {
                chassisController.turnToAngle(turnAngle,1, MecanumDriveLib.TURN_WHEEL.WHEELS_TURN,turnVel,true,4000);
                while(opModeIsActive() && !chassisController.turnIsSettled()){
                    sleep(10);
                }
            }

            if (gamepad1.b){
                odometry.updateRobotAngleByIMU();
            }




           telemetry.addData("Position", "%.2f %.2f %.2f", robotPos.x, robotPos.y, robotPos.angle);
           telemetry.addData("encoder", "LF:%d RF:%d LR:%d RR:%d",
                    chassis.leftFrontDrive.getCurrentPosition(),
                    chassis.rightFrontDrive.getCurrentPosition(),
                    chassis.leftRearDrive.getCurrentPosition(),
                    chassis.rightRearDrive.getCurrentPosition());
           telemetry.addData("IMU Heading:", "%3.2f", odometry.getIMUAngle());

            telemetry.update();


            dashBoardUpdate();
            sleep(10);
        }

        odometry.stopThread();

    }

    public void dashBoardUpdate(){
        dashboardTelemetry.addData("ctrlX", chassisController.powerX);
        dashboardTelemetry.addData("ctrlY", chassisController.powerY);
        dashboardTelemetry.addData("ctrlA", chassisController.powerR);

        dashboardTelemetry.addData("X", robotPos.x);
        dashboardTelemetry.addData("Y", robotPos.y);
        dashboardTelemetry.addData("A", robotPos.angle);
        dashboardTelemetry.addData("XVel", odometry.getRobotLocalVel().velX/ chassisController.MAX_VEL);
        dashboardTelemetry.addData("YVel", odometry.getRobotLocalVel().velY / chassisController.MAX_VEL);
        dashboardTelemetry.update();
    }




}
