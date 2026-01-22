package org.firstinspires.ftc.teamcode;

/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
import org.opencv.calib3d.Calib3d;
import org.openftc.apriltag.AprilTagDetectorJNI;

import java.util.ArrayList;

public abstract class DetectTeamObjectProcessor implements VisionProcessor
{
    public static AllianceColor teamColor = AllianceColor.BLUE;


    public static org.firstinspires.ftc.teamcode.DetectTeamObjectProcessor easyCreateWithDefaults()
    {
        return new org.firstinspires.ftc.teamcode.DetectTeamObjectProcessor.Builder().build();
    }

    public static class Builder
    {
        private AllianceColor teamColor = AllianceColor.BLUE;
        private boolean drawContour = false;
        private boolean drawCenter = false;
        /**
         * Set the team alliance color, so the openCV code could detect RED or BLUE
         *
         * @return the {@link org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.Builder} object, to allow for method chaining
         */
        public org.firstinspires.ftc.teamcode.DetectTeamObjectProcessor.Builder setAllianceColor(AllianceColor teamColor)
        {
            this.teamColor = teamColor;
            return this;
        }

        /**
         * Set the draw detect object contour or not
         *
         * @return the {@link org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.Builder} object, to allow for method chaining
         */

        public org.firstinspires.ftc.teamcode.DetectTeamObjectProcessor.Builder setDrawContour(boolean drawContour)
        {
            this.drawContour = drawContour;
            return this;
        }

        /**
         * Set the draw detect object center or not
         *
         * @return the {@link org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.Builder} object, to allow for method chaining
         */

        public org.firstinspires.ftc.teamcode.DetectTeamObjectProcessor.Builder setDrawCenter(boolean drawCenter)
        {
            this.drawCenter = drawCenter;
            return this;
        }



        /**
         * Create a {@link VisionProcessor} object which may be attached to
         * a {@link org.firstinspires.ftc.vision.VisionPortal} using
         * {@link org.firstinspires.ftc.vision.VisionPortal.Builder#addProcessor(VisionProcessor)}
         * @return a {@link VisionProcessor} object
         */
        public org.firstinspires.ftc.teamcode.DetectTeamObjectProcessor build()
        {
            return new DetectTeamObjectProcessorImpl(
                    teamColor, drawContour,drawCenter
            );
        }
    }



    /**
     * Get a list containing the latest detections, which may be stale
     * i.e. the same as the last time you called this
     * @return a list containing the latest detections.
     */
    public abstract TeamObjectPosition getPosition();

    public abstract void stopDetection();

    public abstract int getProcessedImageCnt();

    public abstract void resetProcessedImageCnt();

    public abstract void startDetection();


}

