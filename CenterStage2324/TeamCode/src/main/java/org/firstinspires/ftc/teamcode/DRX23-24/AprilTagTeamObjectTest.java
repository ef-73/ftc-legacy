/* Copyright (c) 2023 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "AprilTagTeamObject", group = "Concept")
@Disabled
public class AprilTagTeamObjectTest extends LinearOpMode {

    AprilTagTeamObjectDetector aprilTagTeamObjectDetector = null;
    ElapsedTime runTime = new ElapsedTime();

    private AllianceColor teamColor = AllianceColor.BLUE;
    @Override
    public void runOpMode() {

        aprilTagTeamObjectDetector = new AprilTagTeamObjectDetector(this, teamColor,"WebcamBack", "WebcamFront");
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();
        runTime.reset();
        aprilTagTeamObjectDetector.resetTeamObjectDetectCnt(); // must reset the detect processor all the count


        if (opModeIsActive()) {

            while (aprilTagTeamObjectDetector.getTeamObjectDetectCnt() < 5){ // in detect processor after detect 5 frame set the position

               sleep(10);
            }

            TeamObjectPosition teamObjectPosition = aprilTagTeamObjectDetector.getTeamObjectPosition();

            if (teamObjectPosition == TeamObjectPosition.LEFT){
                // go to left position
            }else if (teamObjectPosition == TeamObjectPosition.MIDDLE){
                // go to middle
            }else{
                // go to right
            }

            aprilTagTeamObjectDetector.stopTeamObjectDetect();  // stop team object detection to save CPU resource

            double deltaTime = runTime.milliseconds();  // just for test how long to take for team object detection
            aprilTagTeamObjectDetector.aprilTagTeamObject_Detect_Thread_Task();

            aprilTagTeamObjectDetector.enablePosUpdate();
            while (opModeIsActive()) {
                teamObjectPosition = aprilTagTeamObjectDetector.getTeamObjectPosition();
                telemetry.addLine("runTime=" + String.valueOf(deltaTime));  // around 155ms
                telemetry.addLine("Position = " + teamObjectPosition.name());
                telemetry.addLine("ProcessedImage = " + aprilTagTeamObjectDetector.getTeamObjectDetectCnt());
                telemetry.update();
                // Save CPU resources; can stop or resume streaming when needed.
                if (gamepad1.dpad_down) {
                    aprilTagTeamObjectDetector.doCameraSwitching(AprilTagTeamObjectDetector.WebCamPos.BackCam);
                } else if (gamepad1.dpad_up) {
                    aprilTagTeamObjectDetector.doCameraSwitching(AprilTagTeamObjectDetector.WebCamPos.FrontCam);
                }

                // Share the CPU.
                sleep(20);
            }
        }

        sleep(20);
        // Save more CPU resources when camera is no longer needed
        aprilTagTeamObjectDetector.closeDetector();


    }   // end method runOpMode()



}   // end class
