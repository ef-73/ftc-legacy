package org.firstinspires.ftc.teamcode;

import android.app.slice.Slice;
import android.graphics.Bitmap;
import android.renderscript.Matrix4f;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.matrices.ColumnMajorMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.DenseMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.SliceMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagDetector{
    private boolean debug = true;



    public DigitalChannel redLed = null;
    public DigitalChannel greenLed = null;

    private AprilTagProcessor aprilTag;

    private LinearOpMode mainTask;
    private String cameraName = "";


    private WebcamName webcam = null;

    private RobotPosition robotPosition = new RobotPosition(0,0,0);

    private RobotPosition robotPositionOdom = new RobotPosition(0,0,0);
    private RobotPosition lastRobotPositionOdom = new RobotPosition(0,0,0);


    private RobotPosition robotPositionCam = new RobotPosition(0,0,0);
    private RobotPosition lastRobotPositionCam = new RobotPosition(0,0,0);
    private boolean firstCatchFlag = false;
    private boolean updatePosFlag = false;

    List<RobotPosition> robotPositionList =new ArrayList<RobotPosition>();

    private boolean getAprilTag = false;
    private int cameraID = 1;

    private Odometry odometry = null;


    /*
    From ftc-doc
    Origin
    The 0,0,0 origin of the FIRST Tech Challenge coordinate system is the point in the center of the field, equidistant from all 4 perimeter walls (where the four center tiles meet). The origin point rests on the top surface of the floor mat.

    X Axis
    Looking at the origin from the RED WALL, the X axis extends through the origin point and runs to the right and left, parallel with the RED WALL. The X axis values increase to the right.

    Y Axis
    Looking at the origin from the RED WALL, the Y axis extends through the origin point and runs out and in, perpendicular to the RED WALL. Increasing Y values run out (away) from the RED WALL.

    Z Axis
    Looking at the origin from the RED WALL, the Z axis extends through the origin point and runs up and down in a vertical line. Increasing Z values extend upwards.
     */
    public float [][] tagPosition = { { 1.55f, 1.06f,0.0f}, { 1.55f, 0.90f,0.0f}, {1.55f, 0.74f,0.0f}, // id 1,2,3 blue drop plate side
                                        { 1.55f, -0.74f,0.0f}, { 1.55f, -0.90f,0.0f}, {1.55f, -1.06f,0.0f}, //id 4,5,6 red drop plate side
                                        { -1.8f, -1.03f,0.0f}, { -1.8f, -0.90f,0.0f},  // 7,8(small) red stack
                                        {-1.8f, 0.90f,0.0f},  { -1.8f, 1.03f,0.0f}}; // 9(small), 10 blue stack
    double rotateAngle = Math.toRadians(30);   // 30 degree for tag 1,2,3,4,5,6
    float tag1To6Height = -0.12f;  // tag 1-6 height
    float smallTagHeight = - 0.12f;
    float bigTagHeight = -0.15f;
    private final float FRONT_CAM_X = -0.10f; // negative off center to robot right side, positive off center to robot left
    private final float FRONT_CAM_Y = -0.23f; // robot front

    private final float BACK_CAM_X = 0.14f;// robot right-back side
    private final float BACK_CAM_Y = -0.16f;  // robot back
    private final float CAM_X = 0.115f;// robot right-back side
    private final float CAM_Y = -0.11f;  // robot back
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private List<MatrixF> globalTagPosList = new ArrayList<>();

    public AprilTagDetector(LinearOpMode mainTask, String cameraName){
        this.mainTask = mainTask;  // must be first
        this.cameraName = cameraName;

        cameraID = 1;

        this.odometry = null;
        initAprilTag();
        //  setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        initTagPosMatrix();
        initLed();
    }

    public AprilTagDetector(LinearOpMode mainTask, Odometry odometry, String cameraName){
        this.mainTask = mainTask;  // must be first
        this.cameraName = cameraName;

        cameraID = 1;

        this.odometry = odometry;
        initAprilTag();
        //  setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        initTagPosMatrix();
        initLed();
    }




    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
// Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        webcam = mainTask.hardwareMap.get(WebcamName.class, this.cameraName);


        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag)
                //.setCameraResolution(new Size(640,480))
                .build();


    }   // end method initAprilTag()



    public void closeDetector(){
        visionPortal.close();
    }

    public void stopStreaming(){
        visionPortal.stopStreaming();
    }

    public void resumeStreaming(){
        visionPortal.resumeStreaming();
    }

    public VisionPortal.CameraState getCameraState(){
        return visionPortal.getCameraState();
    }


    public void aprilTag_Detect_Thread_Task() {
        new Thread(new Runnable() {
            @Override

            public void run() {


                while (!mainTask.isStopRequested()  && mainTask.opModeIsActive()) {  //&& mainTask.opModeIsActive()
                    detectAprilTag();
                    mainTask.sleep(10);
                }

            }
        }).start();

    }



    public void detectAprilTag() {


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();


       // mainTask.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.

        robotPositionList.clear();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {



                if (debug){
                    mainTask.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    mainTask.telemetry.addLine(String.format("XYZ %6.3f %6.3f %6.3f  (meter)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    mainTask.telemetry.addLine(String.format("PRY %6.3f %6.3f %6.3f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    mainTask.telemetry.addLine(String.format("RBE %6.3f %6.3f %6.3f  (meter, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }


                    if (debug) {
                        mainTask.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));

                        mainTask.telemetry.addLine(String.format("XYZ %6.3f %6.3f %6.3f  (meter)", detection.rawPose.x, detection.rawPose.y, detection.rawPose.z));
                        int row = detection.rawPose.R.numRows();
                        int col = detection.rawPose.R.numCols();


                        mainTask.telemetry.addData("row, col", "%d %d", row, col);

                        for (int i = 0; i < row; i++) {
                            mainTask.telemetry.addData("row", " %3.3f, %3.3f, %3.3f",
                                    detection.rawPose.R.get(i, 0), detection.rawPose.R.get(i, 1), detection.rawPose.R.get(i, 2));
                        }
                    }

                    if (detection.id >=1 && detection.id <= 10){
                        //setLedTag();
                        MatrixF RT = detection.rawPose.R.transposed();
                        VectorF posVector = new VectorF((float)detection.rawPose.x, (float)detection.rawPose.y, (float)detection.rawPose.z);
                        VectorF rtPosVector = RT.multiplied(posVector).multiplied(-1.0f);
                        MatrixF at = MatrixF.diagonalMatrix(4,1.0f);

                        //copy RT to upper left
                        for (int i = 0; i < 3; i++){
                            for (int j = 0; j < 3; j++){
                                at.put(i,j, RT.get(i,j));

                            }
                        }
                        at.put(0,3,rtPosVector.get(0));
                        at.put(1,3,rtPosVector.get(1));
                        at.put(2, 3, rtPosVector.get(2));
                        VectorF pppVector = new VectorF(0.0f,0.0f,0.0f,1.0f);  // camera position

                        pppVector = new VectorF(CAM_X,0.0f,CAM_Y,1.0f);  // robot center  position

                        //

                        // tag global transform
                        VectorF globalRobot = globalTagPosList.get(detection.id - 1).multiplied(at).multiplied(pppVector);


                        //mainTask.telemetry.addData("GR Size", globalRobot.length());
                        if (debug) {
                            mainTask.telemetry.addData("GR ", "%3.3f, %3.3f, %3.3f, %3.3f", globalRobot.get(0), globalRobot.get(1), globalRobot.get(2), globalRobot.get(3));
                        }

                        // translate to robot global, at blue side, x positive pointing to tag, y is positive pointing to blue side
                        // robot coordinate x  <-- Z (camera Z), y  <--   -X(camera X)
                        RobotPosition newPos = new RobotPosition(globalRobot.get(2),-globalRobot.get(0), 0);
                        robotPositionList.add(newPos);


                    }
                    else{
                        setLedNoTag();
                    }

            } else {
                setLedNoTag();
                if (debug){
                    mainTask.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    mainTask.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));

                    mainTask.telemetry.update();

                }

            }
        }// end for() loop


        if (robotPositionList.size() > 0){

            RobotPosition robotCenterPos = new RobotPosition(0,0,0);

            for (RobotPosition pos : robotPositionList){

                robotCenterPos.x += pos.x;
                robotCenterPos.y += pos.y;
                robotCenterPos.angle  = 0;  // we don't use camera yaw angle, we update it by gyro sensor
            }

            //average position value by different tag
            robotPositionCam.x = robotCenterPos.x / robotPositionList.size();
            robotPositionCam.y = robotCenterPos.y / robotPositionList.size();
            robotPositionCam.angle = 0;

            if (debug){
                mainTask.telemetry.addData("Robot Pos:", "%3.3f, %3.3f, %3.3f", robotPositionCam.x, robotPositionCam.y, robotPositionCam.angle);
            }



            if (odometry != null){
                robotPositionOdom.setPosition(odometry.getRobotPosition());

                if (!firstCatchFlag && updatePosFlag){
                    firstCatchFlag = true;
                }
                else if (updatePosFlag){

                    // to make sure the data is right, we compare the odometry data
                    double delta = Math.abs(robotPositionCam.dis2Pos(lastRobotPositionCam) - (robotPositionOdom.dis2Pos(lastRobotPositionOdom)));
                    if (delta < 0.1){
                        robotPosition.setPosition(robotPositionCam);
                        getAprilTag = true;
                        setLedTag();
                        //odometry.setRobotPositionXY(robotPosition);
                    }


                }

            }
            lastRobotPositionCam.setPosition(robotPositionCam);
            lastRobotPositionOdom.setPosition(robotPositionOdom);

        }
        else{
            setLedNoTag();
        }
        if (debug){
            mainTask.telemetry.update();
        }



        // Add "key" information to telemetry
        //mainTask.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
       // mainTask.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
       // mainTask.telemetry.addLine("RBE = Range, Bearing & Elevation");


    }   //




    public boolean getTagIDOrNot(){
        return getAprilTag;
    }

    public void setGetTagFlagFalse(){
        getAprilTag = false;
    }

    public RobotPosition getRobotPosition(){
        getAprilTag = false;
        return robotPositionCam;


    }

    public void enablePosUpdate(){
        firstCatchFlag = false;
        updatePosFlag = true;
    }

    public void disablePosUpdate(){
        firstCatchFlag = false;
        updatePosFlag = false;
    }


    private void initTagPosMatrix(){
        /*
        // the camera frame coordinate
        //                      ^  Front Z
        //                      |
        //                      |
        //                    camera -- > X
        //          Y is pointing down
        // so simply we define the global coordinate like (0,0) is at field center
        // facing tag(1,2,3,4,5,6) is camera front, so from blue side to red side is positive X
        // from center to tag(pointing to tag) is Z positive, y is pointing down
        //  for example, for tag 1, the global coordinate should be (x:-1.04, y:-0.11(up), Z: 1.57(to center line))
        // the tage 1,2,3,4,5,6 is rotate around X 30 degree, so other components in matrix is cos(30) and sin(30)

         */
        /*
         public float [][] tagPosition = { { 1.55f, 1.04f,0.0f}, { 1.55f, 0.89f,0.0f}, {1.55f, 0.74f,0.0f}, // id 1,2,3 blue drop plate side
                                        { 1.55f, -0.74f,0.0f}, { 1.55f, -0.89f,0.0f}, {1.55f, -1.04f,0.0f}, //id 4,5,6 red drop plate side
                                        { -1.8f, -1.03f,0.0f}, { -1.8f, -0.90f,0.0f},  // 7,8(small) red stack
                                        {-1.8f, 0.90f,0.0f},  { -1.8f, 1.03f,0.0f}}; // 9(small), 10 blue stack

         */


        //tag 1
        MatrixF gtagM = MatrixF.diagonalMatrix(4, 1.0f);
        gtagM.put(0,3, -tagPosition[0][1]);   //X to center line
        gtagM.put(1,1, (float)Math.cos(rotateAngle));
        gtagM.put(1,2, (float)Math.sin(rotateAngle));
        gtagM.put(1,3, tag1To6Height);//height y
        gtagM.put(2,1,-(float)Math.sin(rotateAngle));
        gtagM.put(2,2, (float)Math.cos(rotateAngle));
        gtagM.put(2,3, tagPosition[0][0]);  // Z to center line
        globalTagPosList.add(gtagM);

        //tag 2
        gtagM = MatrixF.diagonalMatrix(4, 1.0f);
        gtagM.put(0,3, -tagPosition[1][1]);//X to center line
        gtagM.put(1,1, (float)Math.cos(rotateAngle));
        gtagM.put(1,2, (float)Math.sin(rotateAngle));
        gtagM.put(1,3, tag1To6Height);//height y
        gtagM.put(2,1,-(float)Math.sin(rotateAngle));
        gtagM.put(2,2, (float)Math.cos(rotateAngle));
        gtagM.put(2,3, tagPosition[1][0]); // Z to center line
        globalTagPosList.add(gtagM);
        //tag 3
        gtagM = MatrixF.diagonalMatrix(4, 1.0f);
        gtagM.put(0,3, -tagPosition[2][1]);//X to center line
        gtagM.put(1,1, (float)Math.cos(rotateAngle));
        gtagM.put(1,2, (float)Math.sin(rotateAngle));
        gtagM.put(1,3, tag1To6Height);//height y
        gtagM.put(2,1,-(float)Math.sin(rotateAngle));
        gtagM.put(2,2, (float)Math.cos(rotateAngle));
        gtagM.put(2,3, tagPosition[2][0]); // Z to center line
        globalTagPosList.add(gtagM);

        //tag 4
        gtagM = MatrixF.diagonalMatrix(4, 1.0f);
        gtagM.put(0,3, -tagPosition[3][1]);//X to center line
        gtagM.put(1,1, (float)Math.cos(rotateAngle));
        gtagM.put(1,2, (float)Math.sin(rotateAngle));
        gtagM.put(1,3, tag1To6Height);//height y
        gtagM.put(2,1,-(float)Math.sin(rotateAngle));
        gtagM.put(2,2, (float)Math.cos(rotateAngle));
        gtagM.put(2,3, tagPosition[3][0]); // Z to center line
        globalTagPosList.add(gtagM);

        //tag 5
        gtagM = MatrixF.diagonalMatrix(4, 1.0f);
        gtagM.put(0,3, -tagPosition[4][1]);//X to center line
        gtagM.put(1,1, (float)Math.cos(rotateAngle));
        gtagM.put(1,2, (float)Math.sin(rotateAngle));
        gtagM.put(1,3, tag1To6Height);//height y
        gtagM.put(2,1,-(float)Math.sin(rotateAngle));
        gtagM.put(2,2, (float)Math.cos(rotateAngle));
        gtagM.put(2,3, tagPosition[4][0]); // Z to center line
        globalTagPosList.add(gtagM);

        //tag 6
        gtagM = MatrixF.diagonalMatrix(4, 1.0f);
        gtagM.put(0,3, -tagPosition[5][1]);//X to center line
        gtagM.put(1,1, (float)Math.cos(rotateAngle));
        gtagM.put(1,2, (float)Math.sin(rotateAngle));
        gtagM.put(1,3, tag1To6Height);//height y
        gtagM.put(2,1,-(float)Math.sin(rotateAngle));
        gtagM.put(2,2, (float)Math.cos(rotateAngle));
        gtagM.put(2,3, tagPosition[5][0]); // Z to center line
        globalTagPosList.add(gtagM);

        // tag 7,8,9,10 is on other side, they are on vertical fence wall, so there is no rotation,
        // but global x is opposite with camera frame x ( if camera facing tags, the x is point to blue side

        //tag 7 big tag
        gtagM = MatrixF.diagonalMatrix(4, 1.0f);
        gtagM.put(0,0, -1.0f);  //rotation matrix x opposite
        gtagM.put(2,2, -1.0f);  //rotation matrix Z opposite

        gtagM.put(0,3, -tagPosition[6][1]);//X to center line
        gtagM.put(1,3, bigTagHeight);//height
        gtagM.put(2,3, tagPosition[6][0]); // Z to center line
        globalTagPosList.add(gtagM);

        //tag 8 small tag
        gtagM = MatrixF.diagonalMatrix(4, 1.0f);
        gtagM.put(0,0, -1.0f);  //rotation matrix x opposite
        gtagM.put(2,2, -1.0f);  //rotation matrix Z opposite

        gtagM.put(0,3, -tagPosition[7][1]);//X to center line
        gtagM.put(1,3, smallTagHeight);//height y
        gtagM.put(2,3, tagPosition[7][0]); // Z to center line
        globalTagPosList.add(gtagM);

        //tag 9 small tag
        gtagM = MatrixF.diagonalMatrix(4, 1.0f);
        gtagM.put(0,0, -1.0f);  //rotation matrix x opposite
        gtagM.put(2,2, -1.0f);  //rotation matrix Z opposite

        gtagM.put(0,3, -tagPosition[8][1]);//X to center line
        gtagM.put(1,3, smallTagHeight);  //height y
        gtagM.put(2,3, tagPosition[8][0]); // Z to center line
        globalTagPosList.add(gtagM);

        //tag 10 bitg tag
        gtagM = MatrixF.diagonalMatrix(4, 1.0f);
        gtagM.put(0,0, -1.0f);  //rotation matrix x opposite
        gtagM.put(2,2, -1.0f);  //rotation matrix Z opposite

        gtagM.put(0,3, -tagPosition[9][1]);//X to center line
        gtagM.put(1,3, bigTagHeight);  //height y
        gtagM.put(2,3, tagPosition[9][0]); // Z to center line
        globalTagPosList.add(gtagM);

    }

    public void initLed(){
        redLed = mainTask.hardwareMap.get(DigitalChannel.class, "red1");
        greenLed = mainTask.hardwareMap.get(DigitalChannel.class, "green1");


        redLed.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed.setMode(DigitalChannel.Mode.OUTPUT);

        redLed.setState(true);
        greenLed.setState(false);
    }

    public void setLedNoTag(){
        redLed.setState(true);
        greenLed.setState(false);
    }

    public void setLedTag(){

        redLed.setState(false);
        greenLed.setState(true);
    }

    /*
    Manually set the camera gain and exposure.
    This can only be called AFTER calling initAprilTag(), and only works for Webcams;
   */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            mainTask.telemetry.addData("Camera", "Waiting");
            mainTask.telemetry.update();
            while (!mainTask.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                mainTask.sleep(20);
            }
            mainTask.telemetry.addData("Camera", "Ready");
            mainTask.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!mainTask.isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                mainTask.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            mainTask.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            mainTask.sleep(20);
        }
    }


}
