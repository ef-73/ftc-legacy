/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static org.opencv.features2d.Features2d.drawKeypoints;

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.features2d.Features2d;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

/**
 * In this sample, we demonstrate how to use the {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int, OpenCvCameraFactory.ViewportSplitMethod)}
 * method in order to concurrently display the preview of two cameras, using
 * OpenCV on both.
 */
@TeleOp(name = "OpenCVDetectTeamObject")
//@Disabled
public class DetectTeamObjectTest extends LinearOpMode
{
    OpenCVDetectTeamObject detector = null;

    FtcDashboard dashboard = null;
    ElapsedTime runTIme = new ElapsedTime();

    ChassisSystem chassisSystem = null;
    AllianceColor teamColor = AllianceColor.RED;
    DistanceSensor leftDisSensor = null;
    DistanceSensor rightDisSensor = null;

    @Override
    public void runOpMode()
    {
        leftDisSensor = hardwareMap.get(DistanceSensor.class, "leftDis");
        rightDisSensor = hardwareMap.get(DistanceSensor.class, "rightDis");
        chassisSystem = new ChassisSystem(this);
        dashboard = FtcDashboard.getInstance();

        detector = new OpenCVDetectTeamObject(this, teamColor,"WebcamFront");

        if (readSDCardFile()){
            telemetry.addLine("read value from file");
        }else{
            telemetry.addLine("could not read value from file");
        }
        telemetry.addLine("Please choose what color to detect b-- red, x- blue in 5 seconds");
        telemetry.update();
        runTIme.reset();
        while(runTIme.milliseconds() < 5000){
            if (gamepad1.b){
                teamColor = AllianceColor.RED;
                detector.setDetectColor(teamColor);
            }
            else if(gamepad1.x){
                teamColor = AllianceColor.BLUE;
                detector.setDetectColor(teamColor);
            }
            sleep(10);
        }

     //   detector.enableDebug();
        telemetry.addLine(String.format("You choose color %s", teamColor));
        telemetry.update();




        dashboard.startCameraStream(detector.webcam, 0);
        waitForStart();

        while(detector.getPosFlag() == TeamObjectPosition.NULL){
            telemetry.addLine("wait for detection ");
            telemetry.update();
        }
        if (detector.getPosFlag()  == TeamObjectPosition.LEFT){
            telemetry.addLine("Left Pos");
        }else if(detector.getPosFlag() == TeamObjectPosition.MIDDLE){
            telemetry.addLine("Middle Pos");
        }else{
            telemetry.addLine("Right Pos Pos");
        }
        telemetry.update();


        while (opModeIsActive())
        {
            chassisSystem.chassisTeleOp();

            if (gamepad1.b){
                detector.setDetectColor(AllianceColor.RED); // red
               telemetry.addLine("detect red");

            }
            else if(gamepad1.x){
                detector.setDetectColor(AllianceColor.BLUE); //blue
                telemetry.addLine("detect blue");
            }
            if (gamepad1.y){
                writeSDCardFile();
                telemetry.addLine("Value saved to file");

            }

     //       telemetry.addData("Left Dis", "%3.3f", leftDisSensor.getDistance(DistanceUnit.MM));
      //      telemetry.addData("Right Dis", "%3.3f", rightDisSensor.getDistance(DistanceUnit.MM));

     //       telemetry.update();

            sleep(100);
        }
        if (detector != null){
            detector.stopDetect();
        }


    }


    private void writeSDCardFile()
    {
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FIRST/TeamObject");
        dir.mkdirs();
        File file = new File(dir, "detectionValue.txt");
        try {
            FileOutputStream f = new FileOutputStream(file, false);
            final PrintStream printStream = new PrintStream(f);

            //red lower threshold first
            printStream.println(Double.toString(OpenCVDetectTeamObject.RED_LOWER_LOW_H));
            printStream.println(Double.toString(OpenCVDetectTeamObject.RED_LOWER_LOW_S));
            printStream.println(Double.toString(OpenCVDetectTeamObject.RED_LOWER_LOW_V));
            printStream.println(Double.toString(OpenCVDetectTeamObject.RED_LOWER_HIGH_H));
            printStream.println(Double.toString(OpenCVDetectTeamObject.RED_LOWER_HIGH_S));
            printStream.println(Double.toString(OpenCVDetectTeamObject.RED_LOWER_HIGH_V));

            // red higher threshold
            printStream.println(Double.toString(OpenCVDetectTeamObject.RED_HIGHER_LOW_H));
            printStream.println(Double.toString(OpenCVDetectTeamObject.RED_HIGHER_LOW_S));
            printStream.println(Double.toString(OpenCVDetectTeamObject.RED_HIGHER_LOW_V));
            printStream.println(Double.toString(OpenCVDetectTeamObject.RED_HIGHER_HIGH_H));
            printStream.println(Double.toString(OpenCVDetectTeamObject.RED_HIGHER_HIGH_S));
            printStream.println(Double.toString(OpenCVDetectTeamObject.RED_HIGHER_HIGH_V));

            // blue, only higher threshold
            printStream.println(Double.toString(OpenCVDetectTeamObject.BLUE_HIGHER_LOW_H));
            printStream.println(Double.toString(OpenCVDetectTeamObject.BLUE_HIGHER_LOW_S));
            printStream.println(Double.toString(OpenCVDetectTeamObject.BLUE_HIGHER_LOW_V));
            printStream.println(Double.toString(OpenCVDetectTeamObject.BLUE_HIGHER_HIGH_H));
            printStream.println(Double.toString(OpenCVDetectTeamObject.BLUE_HIGHER_HIGH_S));
            printStream.println(Double.toString(OpenCVDetectTeamObject.BLUE_HIGHER_HIGH_V));

            printStream.close();
            f.close();
        }
        catch(Exception e){

        }

    }

    private boolean readSDCardFile()
    {
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FIRST/TeamObject");
        File file = new File(dir,"detectionValue.txt");

        StringBuilder text = new StringBuilder();

        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;

            int lineCnt = 0;
            while ((line = br.readLine()) != null) {
                if (lineCnt == 0){
                    OpenCVDetectTeamObject.RED_LOWER_LOW_H = Double.parseDouble(line);
                }else if(lineCnt == 1){
                    OpenCVDetectTeamObject.RED_LOWER_LOW_S = Double.parseDouble(line);
                }else if(lineCnt == 2){
                    OpenCVDetectTeamObject.RED_LOWER_LOW_V = Double.parseDouble(line);
                }else if(lineCnt == 3){
                    OpenCVDetectTeamObject.RED_LOWER_HIGH_H = Double.parseDouble(line);
                }else if(lineCnt == 4){
                    OpenCVDetectTeamObject.RED_LOWER_HIGH_S = Double.parseDouble(line);
                }else if(lineCnt == 5){
                    OpenCVDetectTeamObject.RED_LOWER_HIGH_V = Double.parseDouble(line);
                }else if(lineCnt == 6){
                    OpenCVDetectTeamObject.RED_HIGHER_LOW_H = Double.parseDouble(line);
                }else if(lineCnt == 7){
                    OpenCVDetectTeamObject.RED_HIGHER_LOW_S = Double.parseDouble(line);
                }else if(lineCnt == 8){
                    OpenCVDetectTeamObject.RED_HIGHER_LOW_V = Double.parseDouble(line);
                }else if(lineCnt == 9){
                    OpenCVDetectTeamObject.RED_HIGHER_HIGH_H = Double.parseDouble(line);
                }else if(lineCnt == 10){
                    OpenCVDetectTeamObject.RED_HIGHER_HIGH_S = Double.parseDouble(line);
                }else if(lineCnt == 11){
                    OpenCVDetectTeamObject.RED_HIGHER_HIGH_V = Double.parseDouble(line);
                }else if(lineCnt == 12){
                    OpenCVDetectTeamObject.BLUE_HIGHER_LOW_H = Double.parseDouble(line);
                }else if(lineCnt == 13){
                    OpenCVDetectTeamObject.BLUE_HIGHER_LOW_S = Double.parseDouble(line);
                }else if(lineCnt == 14){
                    OpenCVDetectTeamObject.BLUE_HIGHER_LOW_V = Double.parseDouble(line);
                }else if(lineCnt == 15){
                    OpenCVDetectTeamObject.BLUE_HIGHER_HIGH_H = Double.parseDouble(line);
                }else if(lineCnt == 16){
                    OpenCVDetectTeamObject.BLUE_HIGHER_HIGH_S = Double.parseDouble(line);
                }else if(lineCnt == 17){
                    OpenCVDetectTeamObject.BLUE_HIGHER_HIGH_V = Double.parseDouble(line);
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


}