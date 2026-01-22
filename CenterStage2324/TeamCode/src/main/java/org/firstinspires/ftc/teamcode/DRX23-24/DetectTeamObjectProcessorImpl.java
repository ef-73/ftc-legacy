package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.ApriltagDetectionJNI;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

public class DetectTeamObjectProcessorImpl extends DetectTeamObjectProcessor {
    public static final String TAG = "TeamObjectProcessorImpl";


    private TeamObjectPosition detections = TeamObjectPosition.NULL;

    private final Object detectionsUpdateSync = new Object();

    private AllianceColor teamColor = AllianceColor.BLUE;
    private boolean drawContour;
    private boolean drawCenter;
    private boolean detectionFlag = false;

    int image_width = 640;
    int image_height = 480;
    int image_channels = 3;
    private int rowStart = 100;  // depends on camera mounting angle, only process bottom part of image
    private int colStart = 0;
    private int areaThreshold = 3500;
    private double area = 0;

    Mat image_roi = null;
    Mat imgHSV = null;
    Mat imgThreshold = null;
    Mat imgThresholdL = null;
    Mat imgThresholdH = null;


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

    private int posFlag = -1;  // left -- 0, middle - 1, right -- 2
    private int leftCnt = 0;
    private int middleCnt = 0;
    private int rightCnt = 0;
    private int processedImageCnt = 0;

    private List<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    private int objectCenterX = 0;
    private int objectCenterY = 0;

    private Paint textPaint = new Paint();
  //  private Paint rectPaint = new Paint();

  //  private Rect objectRect = null;





    public DetectTeamObjectProcessorImpl(AllianceColor teamColor, boolean drawContour, boolean drawCenter)
    {
        this.teamColor = teamColor;
        this.drawContour = drawContour;
        this.drawCenter  = drawCenter;
        detectionFlag = false;
        middleCnt = 0;
        leftCnt = 0;
        rightCnt = 0;
        processedImageCnt = 0;

        readSetValueFromSDCardFile();
    }



    @Override
    protected void finalize()
    {
        // Might be null if createApriltagDetector() threw an exception

    }

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {

    }

    @Override
    public void startDetection(){
        processedImageCnt = 0;
        leftCnt = 0;
        middleCnt = 0;
        rightCnt = 0;
        detectionFlag = true;
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos)
    {
        if (detectionFlag) {
            // Convert to greyscale
            image_width = (int) input.size().width;
            image_height = (int) input.size().height;


            image_roi = input.submat(rowStart, image_height, colStart, image_width);
            imgHSV = new Mat((int) (image_roi.size().width), (int) (image_roi.size().height), image_roi.channels());
            imgThreshold = new Mat(image_width, image_height, 1);
            imgThresholdL = new Mat(image_width, image_height, 1);
            imgThresholdH = new Mat(image_width, image_height, 1);
            Imgproc.cvtColor(image_roi, imgHSV, Imgproc.COLOR_RGB2HSV);

            //for red
            Scalar lowerL, lowerU, higherL, higherU;
            if (teamColor == AllianceColor.RED) {
                //red
                // mainTask.telemetry.addLine("Detect Red");
                //lower boundary
                lowerL = new Scalar(RED_LOWER_LOW_H, RED_LOWER_LOW_S, RED_LOWER_LOW_V);
                lowerU = new Scalar(RED_LOWER_HIGH_H, RED_LOWER_HIGH_S, RED_LOWER_HIGH_V);
                //upper boundary RED color range values; Hue (160 - 180)
                higherL = new Scalar(RED_HIGHER_LOW_H, RED_HIGHER_LOW_S, RED_HIGHER_LOW_V);
                higherU = new Scalar(RED_HIGHER_HIGH_H, RED_HIGHER_HIGH_S, RED_HIGHER_HIGH_V);

                //Core.inRange(imgHSV,lower,higher,imgThreshold);
                Core.inRange(imgHSV, lowerL, lowerU, imgThresholdL);
                Core.inRange(imgHSV, higherL, higherU, imgThresholdH);
                //Adding two images
                Core.addWeighted(imgThresholdH, 1.0, imgThresholdL, 1.0, 0, imgThreshold);
            } else {
                //for blue
                //   mainTask.telemetry.addLine("Detect Blue");
                higherL = new Scalar(BLUE_HIGHER_LOW_H, BLUE_HIGHER_LOW_S, BLUE_HIGHER_LOW_V);  // S: 100 ---50 more brighter
                higherU = new Scalar(BLUE_HIGHER_HIGH_H, BLUE_HIGHER_HIGH_S, BLUE_HIGHER_HIGH_V);

                Core.inRange(imgHSV, higherL, higherU, imgThreshold);
            }

            // find contours on threshold image-- imgThreshold
            contours = new ArrayList<>();
            hierarchy = new Mat();
            Imgproc.findContours(imgThreshold, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

            // if any contour exist...
            if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {

                for (int i = 0; i < contours.size(); i++) {
                    Moments c = Imgproc.moments(contours.get(i));

                    if (c.m00 > areaThreshold) {
                        area = c.m00;
                        int cX = (int) (c.m10 / c.m00);
                        int cY = (int) (c.m01 / c.m00);
                        //objectRect = Imgproc.boundingRect(contours.get(i));

                        Log.d("Center", "CX" + cX + " CY" + cY);

                        if(cX >= 400){
                            rightCnt ++;
                        }else if (cX >= 220 && cX < 400) {
                            middleCnt++;
                        } else if (cX < 220) {
                            leftCnt++;
                        }

                        objectCenterX = cX;
                        objectCenterY = cY;

                    }

                }


            }


            // Run

            synchronized (detectionsUpdateSync) {
                processedImageCnt++;
                if (processedImageCnt >= 5) {
                    // only process 5 frame image
                    if (middleCnt > 3) {

                        detections = TeamObjectPosition.MIDDLE;

                    } else if (leftCnt > 3) {

                        detections = TeamObjectPosition.LEFT;

                    } else {
                        detections = TeamObjectPosition.RIGHT;

                    }
                }
            }
        }

        // TODO do we need to deep copy this so the user can't mess with it before use in onDrawFrame()?
        return detections;
    }

    private MovingStatistics solveTime = new MovingStatistics(50);



    private final Object drawSync = new Object();

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        if (detectionFlag) {
            // Only one draw operation at a time thank you very much.
            // (we could be called from two different threads - viewport or camera stream)
            synchronized (drawSync) {
                if ((drawContour || drawCenter) && userContext != null) {
                    textPaint.setColor(Color.WHITE);
                    textPaint.setAntiAlias(true);
                    textPaint.setTypeface(Typeface.DEFAULT_BOLD);
                    textPaint.setTextSize(30);

                    /*
                    rectPaint = new Paint();
                    rectPaint.setAntiAlias(true);
                    if (teamColor == AllianceColor.BLUE) {
                        rectPaint.setColor(Color.rgb(0, 0, 255));
                    } else {
                        rectPaint.setColor(Color.rgb(255, 0, 0));
                    }

                    rectPaint.setStyle(Paint.Style.FILL);
                    */
                    canvas.save();


                    canvas.drawText("Position:" + detections.name(), 30, 30, textPaint);
                    canvas.drawText("Center:" + String.valueOf(objectCenterX) + "," + String.valueOf(objectCenterY) + "," +
                            String.valueOf(area), objectCenterX, objectCenterY + rowStart, textPaint);

                    canvas.restore();
                }
            }
        }
    }

    @Override
    public TeamObjectPosition getPosition(){
       synchronized (detectionsUpdateSync) {
            return detections;
        }
    }

    @Override
    public void stopDetection(){
        synchronized (detectionsUpdateSync) {
            detectionFlag = false;
        }
    }

    @Override
    public int getProcessedImageCnt(){
        synchronized (detectionsUpdateSync) {
            return processedImageCnt;
        }
    }

    @Override
    public void resetProcessedImageCnt(){
        synchronized (detectionsUpdateSync) {
            processedImageCnt = 0;
            detections = TeamObjectPosition.NULL;
            leftCnt = 0;
            rightCnt = 0;
            middleCnt = 0;
        }
    }

    private boolean readSetValueFromSDCardFile()
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
            br.close();
            return true;
        }
        catch (IOException e) {
            //You'll need to add proper error handling here
            return false;
        }
    }

}
