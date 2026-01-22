package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.features2d.Features2d;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


@Config
public class OpenCVDetectTeamObject {
    private boolean debug = true;
    LinearOpMode mainTask;
    OpenCvCamera webcam;
    int cameraMonitorViewId = 0;
    int[] viewportContainerIds = null;

    public static  AllianceColor teamColor = AllianceColor.BLUE;  // 0 for red, 1 for blue
    int detectPosX = 0;
    int detectPosY = 0;

    int image_width = 640;
    int image_height = 480;
    int image_channels = 3;

    int rowStart = 100;  // depends on camera mounting angle, only process bottom part of image
    int colStart = 0;
    int areaThreshold = 200;

    Mat imgHSV = new Mat(image_width,image_height,image_channels);
    Mat imgThreshold = new Mat(image_width,image_height,1);
    Mat imgThresholdL = new Mat(image_width,image_height,1);
    Mat imgThresholdH = new Mat(image_width,image_height,1);

    public static double RED_LOWER_LOW_H = 0;
    public static double RED_LOWER_LOW_S = 100;
    public static double RED_LOWER_LOW_V = 20;
    public static double RED_LOWER_HIGH_H = 10;
    public static double RED_LOWER_HIGH_S = 255;
    public static double RED_LOWER_HIGH_V = 255;

    public static double RED_HIGHER_LOW_H = 160;
    public static double RED_HIGHER_LOW_S = 100;
    public static double RED_HIGHER_LOW_V = 20;
    public static double RED_HIGHER_HIGH_H = 179;
    public static double RED_HIGHER_HIGH_S = 255;
    public static double RED_HIGHER_HIGH_V = 255;

    public static double BLUE_HIGHER_LOW_H = 90;
    public static double BLUE_HIGHER_LOW_S = 100;
    public static double BLUE_HIGHER_LOW_V = 20;
    public static double BLUE_HIGHER_HIGH_H = 140;
    public static double BLUE_HIGHER_HIGH_S = 255;
    public static double BLUE_HIGHER_HIGH_V = 255;


    private TeamObjectPosition posFlag = TeamObjectPosition.NULL;  // left -- 0, middle - 1, right -- 2
    private int leftCnt = 0;
    private int middleCnt = 0;
    private int rightCnt = 0;
    private int processedImageCnt = 0;

    private boolean webcamOpenFlag = false;



    public OpenCVDetectTeamObject(LinearOpMode mainTask, AllianceColor teamColor, String camName){
        this.teamColor = teamColor;
        this.mainTask = mainTask;
        this.cameraMonitorViewId = mainTask.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", mainTask.hardwareMap.appContext.getPackageName());

        /**
         * This is the only thing you need to do differently when using multiple cameras.
         * Instead of obtaining the camera monitor view and directly passing that to the
         * camera constructor, we invoke {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int, OpenCvCameraFactory.ViewportSplitMethod)}
         * on that view in order to split that view into multiple equal-sized child views,
         * and then pass those child views to the constructor.
         */
        this.viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(mainTask.hardwareMap.get(WebcamName.class, camName), viewportContainerIds[0]);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.setPipeline(new detectObjectByColor());
                webcam.startStreaming(image_width, image_height, OpenCvCameraRotation.UPRIGHT);
                webcamOpenFlag = true;
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        this.posFlag = TeamObjectPosition.NULL;
        leftCnt = 0;
        rightCnt = 0;
        middleCnt = 0;
        processedImageCnt = 0;
    }

    class detectObjectByColor extends OpenCvPipeline
    {
        @Override
        public Mat processFrame(Mat input)
        {


            Mat image_roi = input.submat(rowStart,image_height,colStart,image_width);

            Imgproc.cvtColor(image_roi, imgHSV, Imgproc.COLOR_RGB2HSV);

            //for red
            Scalar lowerL, lowerU, higherL, higherU;
            if (teamColor == AllianceColor.RED){
                //red
               // mainTask.telemetry.addLine("Detect Red");
                //lower boundary
                lowerL = new Scalar(RED_LOWER_LOW_H,RED_LOWER_LOW_S,RED_LOWER_LOW_V);
                lowerU = new Scalar(RED_LOWER_HIGH_H,RED_LOWER_HIGH_S,RED_LOWER_HIGH_V);
                //upper boundary RED color range values; Hue (160 - 180)
                higherL = new Scalar(RED_HIGHER_LOW_H,RED_HIGHER_LOW_S,RED_HIGHER_LOW_V);
                higherU = new Scalar(RED_HIGHER_HIGH_H,RED_HIGHER_HIGH_S,RED_HIGHER_HIGH_V);

                //Core.inRange(imgHSV,lower,higher,imgThreshold);
                Core.inRange(imgHSV,lowerL,lowerU,imgThresholdL);
                Core.inRange(imgHSV,higherL,higherU,imgThresholdH);
                //Adding two images
                Core.addWeighted(imgThresholdH, 1.0,imgThresholdL, 1.0, 0, imgThreshold);
            }
            else{
                //for blue
             //   mainTask.telemetry.addLine("Detect Blue");
                higherL = new Scalar(BLUE_HIGHER_LOW_H,BLUE_HIGHER_LOW_S,BLUE_HIGHER_LOW_V);  // S: 100 ---50 more brighter
                higherU = new Scalar(BLUE_HIGHER_HIGH_H,BLUE_HIGHER_HIGH_S,BLUE_HIGHER_HIGH_V);

                Core.inRange(imgHSV,higherL,higherU,imgThreshold);
            }

            // find contours on threshold image-- imgThreshold
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(imgThreshold, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

            // if any contour exist...
            if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {

                // for each contour, display it in white
                for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
                    Imgproc.drawContours(image_roi, contours, idx, new Scalar(250, 255, 255));
                }


                for (int i = 0; i < contours.size(); i++)
                {
                    Moments c = Imgproc.moments(contours.get(i));
                    Log.d("Area:", String.valueOf(c.m00));
                    if (c.m00 > 3500) {

                        int cX = (int) (c.m10 / c.m00);
                        int cY = (int) (c.m01 / c.m00);

                        /*
                        Imgproc.putText(image_roi, String.valueOf(cX- image_width / 2) + "," + String.valueOf(cY - image_height /2), new Point(cX - 20, cY - 20),
                                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
                        */
                        Imgproc.putText(image_roi, String.valueOf(cX) + "," + String.valueOf(cY) + "," + String.valueOf(c.m00), new Point(cX + 20, cY + 20),
                                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
                        Imgproc.drawMarker(image_roi,new Point(cX,cY),new Scalar(255, 255, 255),Imgproc.MARKER_CROSS,20,10);

                        Log.d("Center", "CX" + cX + " CY" + cY);
                        if (debug) {
                            mainTask.telemetry.addData("(Cx,Cy) Area", "%d,%d, %.2f", cX, cY, c.m00);
                        }
                        if (cX > 220 && cX < 400){
                            middleCnt ++;
                        }else if (cX < 220){
                            leftCnt ++;
                        }

                    }

                }


            }
            processedImageCnt ++;
            if (processedImageCnt > 5){
                if (debug) {
                    mainTask.telemetry.addLine(String.format("Detect Color %s", teamColor));
                    mainTask.telemetry.addData("Image Cnt", " %d", processedImageCnt);
                    mainTask.telemetry.addData("middleCnt", "%d", middleCnt);
                    mainTask.telemetry.addData("leftCnt", "%d", leftCnt);

                }

                // only process 5 frame image
                if (middleCnt > 3){
                    posFlag = TeamObjectPosition.MIDDLE;

                }else if(leftCnt > 3){
                    posFlag = TeamObjectPosition.LEFT;

                }else{
                    posFlag = TeamObjectPosition.RIGHT;

                }
            }

            if (debug) {
                mainTask.telemetry.update();
            }
            return image_roi; //draw on input, imgThreshold, image_roi

        }
    }

    public void setDetectColor(AllianceColor teamColor){
        this.teamColor = teamColor;
    }


    public void stopDetect(){
        webcam.stopStreaming();
        webcam.stopRecordingPipeline();

        mainTask.sleep(50);

        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                webcam = null;
                webcamOpenFlag = false;
             }
        });

    }

    public boolean getWebCloseOrNot(){
        return webcamOpenFlag;
    }

    public TeamObjectPosition getPosFlag(){
        return posFlag;
    }

    public void resetDetectCnt(){
        processedImageCnt = 0;
        leftCnt = 0;
        rightCnt = 0;
        middleCnt = 0;
        posFlag = null;
    }

    public void enableDebug(){
        debug = true;
    }

}
