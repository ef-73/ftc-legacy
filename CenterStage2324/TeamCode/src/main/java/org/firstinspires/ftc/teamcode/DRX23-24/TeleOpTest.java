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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name="Basic: TeleOp ", group="Linear OpMode")
//@Disabled
public class TeleOpTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    DropPurple_PushServo dropPurplePushServo = null;

    private SliderAndTraySystem_Servo sliderTrayController = null;


    private ChassisSystem chassisSystem = null;

    private AprilTagTeamObjectDetector aprilTagTeamObjectDetector = null;
    private Odometry odometry = null;
    private MecanumDriveLib chassisController = null;

    private AirPlaneLaunch airPlaneLaunch = null;
    private HangArmControl hangArmControl = null;

    private RobotPosition robotPosition = new RobotPosition(0,0,0);
    private RobotPosition camPosition = new RobotPosition(0,0,0);

    double targetAngle = 0;
    double targetX = 0;
    double targetY = 0;

    private boolean airPlaneLaunched = false;
    private boolean firstAdjust = true;

    private int launchButtonDelayCnt = 0;

    @Override
    public void runOpMode() {

        // could save time for reading encoder
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        dropPurplePushServo = new DropPurple_PushServo(this, false, DropPurple_PushServo.ArmLeftRight.LEFT);

        chassisSystem = new ChassisSystem(this);

        odometry = new Odometry(this,chassisSystem,new RobotPosition(0,0,180),false);


        chassisController = new MecanumDriveLib(this, odometry,chassisSystem);


        // aprilTagTeamObjectDetector = new AprilTagTeamObjectDetector(this, AllianceColor.BLUE,odometry, "WebcamFront", "WebcamBack");

        sliderTrayController = new SliderAndTraySystem_Servo(this, chassisSystem,false);

        airPlaneLaunch = new AirPlaneLaunch(this);
        hangArmControl = new HangArmControl(this);

        sleep(200); // give sometime for camera initialization
        // Wait for the game to start (driver presses PLAY)
        launchButtonDelayCnt = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        sliderTrayController.trayOpenBothLock();
        sleep(100);

        sliderTrayController.resetTrayRotate();
        sliderTrayController.startResetCheck();
        dropPurplePushServo.resetPushServo();
        dropPurplePushServo.resetDropServo();
        sliderTrayController.resetHSlide();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            dropPurplePushServo.teleOp_pushServo();

            sliderTrayController.joystickCtrl();



            robotPosition.setPosition(odometry.getRobotPosition());

            if (aprilTagTeamObjectDetector != null) {
                if (aprilTagTeamObjectDetector.getTagIDOrNot()) {
                    camPosition = aprilTagTeamObjectDetector.getRobotPosition();
                }
            }

/*
                if ( gamepad1.dpad_left || gamepad1.dpad_right){  // Math.abs(gamepad2.left_stick_x) > 0.2
                    if (firstAdjust){
                        odometry.setRobotPositionAngle(180);
                        firstAdjust = false;
                    }
                    // only strafe move robot as setting angle
                    double angleError = odometry.getRobotPosition().angle - 180;  // target will be 180

                    double steeringPower = 0.03 * angleError;
                    double drvPower = 0.4;
                    if (gamepad1.dpad_left){
                        drvPower *= -1;
                    }

                    chassisSystem.driveRobot(-drvPower + steeringPower, drvPower + steeringPower, drvPower - steeringPower, -drvPower - steeringPower);

                }
                else if(gamepad1.dpad_down){
                    double drvPower = 0.4;
                    chassisSystem.driveRobot(drvPower, drvPower , drvPower, drvPower );
                }
                else if(gamepad1.dpad_up){
                    double drvPower = -0.4;
                    chassisSystem.driveRobot(drvPower, drvPower , drvPower, drvPower );
                }
                else {
                    firstAdjust = true;
                    chassisSystem.chassisTeleOp();
                }
*/
            chassisSystem.chassisTeleOp();

            if (gamepad1.y){
                if (launchButtonDelayCnt == 0) {
                    if (!airPlaneLaunch.airplaneMotorStart) {
                        airPlaneLaunch.startAirplaneMotor();
                    } else {
                        airPlaneLaunched = true;
                        airPlaneLaunch.launchAirPlane();
                    }
                    launchButtonDelayCnt = 30;
                }
            }
            else{
                if (launchButtonDelayCnt > 0){
                    launchButtonDelayCnt --;
                }
            }

            if (gamepad1.b && airPlaneLaunched){
                hangArmControl.hangArmUp();
            }



            if(gamepad1.right_trigger > 0){
                airPlaneLaunch.swingCoverServo();
                dropPurplePushServo.toggleDropArm();
                if (airPlaneLaunch.airplaneMotorStart){
                    airPlaneLaunch.stopAirplaneMotor();
                }
            }

            if (gamepad1.start){
                //reset robot position angle
                Log.d("set robot angle", "180");
                odometry.setRobotPositionAngle(180);
                //force reset arm slide, in case the slide encoder position is not right
                sliderTrayController.forceResetSlide();
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);

            telemetry.addData("Slide Len ", "%3.3f", sliderTrayController.getSliderLen());
            telemetry.addData("ArmTilt Angle ", "%3.3f", sliderTrayController.getArmTiltAngle());
            telemetry.addData("TrayTilt Angle ", "%3.3f", sliderTrayController.getTrayTiltAngle());

            telemetry.addData("Tray Rotate", "%3.3f", sliderTrayController.getTrayRoateAngle());

            telemetry.addData("Slide Pos", "%d", sliderTrayController.getSliderPos());
            telemetry.addData("RobotPos", "%3.3f,%3.3f,%3.3f", robotPosition.x, robotPosition.y, robotPosition.angle);
            telemetry.addData("CamPos", "%3.3f, %3.3f,%3.3f", camPosition.x, camPosition.y, camPosition.angle );
            telemetry.addData("targetPos", "%3.3f, %3.3f, %3.3f", targetX, targetY, targetAngle );
            telemetry.addData("HangMotor", "%d", hangArmControl.hangMotor.getCurrentPosition());
            telemetry.addData("encoder", "LF:%d RF:%d LR:%d RR:%d",
                    chassisSystem.leftFrontDrive.getCurrentPosition(),
                    chassisSystem.rightFrontDrive.getCurrentPosition(),
                    chassisSystem.leftRearDrive.getCurrentPosition(),
                    chassisSystem.rightRearDrive.getCurrentPosition());
            telemetry.addData("Rotate Pos", "%d", sliderTrayController.trayRotatePos);
            telemetry.addLine("Cnt = " + launchButtonDelayCnt);
            telemetry.update();


            sleep(20);
        }

    }

    public void goToTagPosition(int posFlag){
        // posFlag = 1 left, 2 middle, 3 right
        double offset = 0.5;
        if (aprilTagTeamObjectDetector != null) {
            if (aprilTagTeamObjectDetector.getTagIDOrNot()) {
                camPosition.setPosition(aprilTagTeamObjectDetector.getRobotPosition());
                odometry.setRobotPositionXY(camPosition);


                targetAngle = calculateTargetAngle();
                targetX = 0;
                targetY = 0;

                if (camPosition.y > 0) {
                    // tag 1, 2, 3
                    targetX = aprilTagTeamObjectDetector.tagPosition[posFlag - 1][0] - offset;
                    targetY = aprilTagTeamObjectDetector.tagPosition[posFlag - 1][1];
                } else {
                    // tag 4,5 6
                    targetX = aprilTagTeamObjectDetector.tagPosition[posFlag - 1 + 3][0] - offset;
                    targetY = aprilTagTeamObjectDetector.tagPosition[posFlag - 1 + 3][1];

                }

                runtime.reset();
                chassisController.p2pDrive(new RobotPosition(targetX, targetY, targetAngle), 1.0, 1.0, 0.0, 0.02, 1, 0.0, true, 10000);

                while (opModeIsActive() && !chassisController.p2pIsSettled() && !gamepad1.back && runtime.milliseconds() < 2000) {
                    sleep(10);
                }
                chassisController.stopAllThread();
                chassisSystem.stopRobot();
            }
        }

    }

    public double calculateTargetAngle(){
        double robotAngle = odometry.getRobotPosition().angle;
        double targetAngle =  0;

        double deltaAngle = 0;

        // if facing tag is 0 degree
        /*
        int angleMultiply = (int)Math.floor(Math.abs(robotAngle) / 360);

        deltaAngle = Math.abs(robotAngle) - angleMultiply * 360;

        if (deltaAngle > 180){
            targetAngle = angleMultiply * 360 + 360;
        }else{
            targetAngle = angleMultiply * 360 ;
        }
        targetAngle = Math.signum(robotAngle) * targetAngle;
        */



        // if facing tag is 180 degree, back to tags

        int angleMultiply = (int)Math.floor(Math.abs(robotAngle) / 180);


        if (angleMultiply > 180){
            targetAngle = angleMultiply * 180 + 180;
        }else{
            if (Math.abs(angleMultiply) > 0) {
                targetAngle = angleMultiply * 180;
            }
            else{
                targetAngle = 180;
            }
        }

        targetAngle = Math.signum(robotAngle) * targetAngle;


        return targetAngle;
    }






}
