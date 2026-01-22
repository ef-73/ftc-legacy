package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;

public class AprilTagTeamObjectDetector {
    private AprilTagProcessor aprilTag;
    private DetectTeamObjectProcessor detectTeamObjectProcessor;


    private boolean debug = false;
    public DigitalChannel redLed1 = null;
    public DigitalChannel greenLed1 = null;




    private LinearOpMode mainTask;
    private String cameraName1 = "";
    private String cameraName2 = "";

    private WebcamName webcamFront, webcamBack;



    private RobotPosition robotPosition = new RobotPosition(0,0,0);

    private RobotPosition robotPositionOdom = new RobotPosition(0,0,0);
    private RobotPosition lastRobotPositionOdom = new RobotPosition(0,0,0);


    private RobotPosition robotPositionCam = new RobotPosition(0,0,0);
    private RobotPosition lastRobotPositionCam = new RobotPosition(0,0,0);
    private boolean firstCatchFlag = false;
    private boolean updatePosFlag = false;

    List<RobotPosition> robotPositionList =new ArrayList<RobotPosition>();

    private static volatile boolean getAprilTag = false;
    private static ReentrantLock tagFlagLock = new ReentrantLock();


    private int cameraID = 1;

    private Odometry odometry = null;

    private int processedImgCnt = 0;
    private TeamObjectPosition teamObjectPosition = TeamObjectPosition.NULL;
    private AllianceColor teamColor = AllianceColor.RED;


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
    private final float FRONT_CAM_X = -0.0f; // negative off center to robot right side, positive off center to robot left
    private final float FRONT_CAM_Y = -0.15f; // robot front

    private final float BACK_CAM_X = 0.115f;// robot right-back side
    private final float BACK_CAM_Y = -0.11f;  // robot back
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private List<MatrixF> globalTagPosList = new ArrayList<>();

    public AprilTagTeamObjectDetector(LinearOpMode mainTask, String cameraName1, String cameraName2){
        this.mainTask = mainTask;  // must be first
        this.cameraName1 = cameraName1;
        this.cameraName2 = cameraName2;
        cameraID = 1;  //FRONT ONE
        getAprilTag = false;
        this.odometry = null;
        initAprilTagTeamObjectDetector();
      //  setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        initTagPosMatrix();
        initLed();
    }

    public AprilTagTeamObjectDetector(LinearOpMode mainTask, AllianceColor teamColor, String cameraName1, String cameraName2){
        this.mainTask = mainTask;  // must be first
        this.cameraName1 = cameraName1;
        this.cameraName2 = cameraName2;
        this.teamColor = teamColor;
        cameraID = 1;  //Back
        getAprilTag = false;
        this.odometry = odometry;
        initAprilTagTeamObjectDetector();
        // setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        initTagPosMatrix();
        initLed();
    }

    public AprilTagTeamObjectDetector(LinearOpMode mainTask, AllianceColor teamColor,Odometry odometry, String cameraName1, String cameraName2){
        this.mainTask = mainTask;  // must be first
        this.cameraName1 = cameraName1;
        this.cameraName2 = cameraName2;
        this.teamColor = teamColor;
        cameraID = 1;  //Back ONE
        this.odometry = odometry;
        getAprilTag = false;
        initAprilTagTeamObjectDetector();
       // setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        initTagPosMatrix();
        initLed();
    }


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTagTeamObjectDetector() {

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
                // default value for C920  in 640*480 resolution is pretty good enough
                // from camera calibration xml file for C920, 640*480
                //focalLength="622.001f, 622.001f"
                //principalPoint="319.803f, 241.251f"
                //distortionCoefficients="0.1208, -0.261599, 0, 0, 0.10308, 0, 0, 0"*/
                 .setLensIntrinsics(622.001f, 622.001f, 319.803f, 241.251f)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        detectTeamObjectProcessor = new DetectTeamObjectProcessor.Builder()
                .setAllianceColor(teamColor)
                .setDrawCenter(true)
                .setDrawContour(true)
                .build();

        webcamFront = mainTask.hardwareMap.get(WebcamName.class, this.cameraName1);
        webcamBack = mainTask.hardwareMap.get(WebcamName.class, this.cameraName2);
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcamFront, webcamBack);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)

                .addProcessors(aprilTag, detectTeamObjectProcessor)
                //.addProcessor(aprilTag)
                .setCameraResolution(new Size(640,480))
                .build();

        stopAprilTagDetect();  // only enable team object detect first

       // visionPortal.setActiveCamera(webcamFront);

    }   // end method initAprilTag()

    public int getTeamObjectDetectCnt(){

        return detectTeamObjectProcessor.getProcessedImageCnt();
    }

    public TeamObjectPosition getTeamObjectPosition(){
        return detectTeamObjectProcessor.getPosition();
    }

    public void resetTeamObjectDetectCnt(){
        detectTeamObjectProcessor.resetProcessedImageCnt();
    }

    public void stopTeamObjectDetect(){
        detectTeamObjectProcessor.stopDetection();
        visionPortal.setProcessorEnabled(detectTeamObjectProcessor, false);

    }

    public void startAprilTagDetect(){
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public void stopAprilTagDetect(){
        visionPortal.setProcessorEnabled(aprilTag, false);
    }

    public void closeDetector(){
        visionPortal.close();
    }

    public void stopStreaming(){
        visionPortal.stopStreaming();
    }

    public void resumeStreaming(){
        visionPortal.resumeStreaming();
    }


    public void aprilTagTeamObject_Detect_Thread_Task() {

        new Thread(new Runnable() {
            @Override

            public void run() {


                while (!mainTask.isStopRequested()  && mainTask.opModeIsActive()) {  //&& mainTask.opModeIsActive()
                    detectAprilTag();
                    mainTask.sleep(50);
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


                /*
                if (debug){
                    mainTask.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    mainTask.telemetry.addLine(String.format("XYZ %6.3f %6.3f %6.3f  (meter)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    mainTask.telemetry.addLine(String.format("PRY %6.3f %6.3f %6.3f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    mainTask.telemetry.addLine(String.format("RBE %6.3f %6.3f %6.3f  (meter, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }

                */
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


                        if (cameraID == 2){
                            //back camera
                            pppVector = new VectorF(BACK_CAM_X, 0.0f, BACK_CAM_Y, 1.0f);  // robot center  position
                        }
                        else{
                            //front camera,
                            pppVector = new VectorF(FRONT_CAM_X,0.0f,FRONT_CAM_Y,1.0f);  // robot center  position
                        }
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
                        tagFlagLock.lock();
                        try{
                            getAprilTag = true;
                            setLedTag();
                        }finally {
                            tagFlagLock.unlock();
                        }


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


    /**
     * Set the active camera according to input from the gamepad.
     */
    public void doCameraSwitching(WebCamPos camName) {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {

            if (camName == WebCamPos.FrontCam) {
                 visionPortal.setActiveCamera(webcamFront);
                 cameraID = 1;
            } else if (camName == WebCamPos.BackCam) {
                visionPortal.setActiveCamera(webcamBack);
                cameraID = 2;
            }

        }

    }   // end method doCameraSwitching()


    // 1 -- front
    //-1 -- back
    public int getCameraID(){
        if (visionPortal.getActiveCamera().toString().contains("389EAF60")){
            return 1; //back
        }
        else{//(visionPortal.getActiveCamera().toString() == "222E1FAF"){
            return 2;

        }

    }



    public boolean getTagIDOrNot(){
        tagFlagLock.lock();
        try{
            return getAprilTag;
        }finally {
            tagFlagLock.unlock();
        }

    }

    public void setGetTagFlagFalse(){
        getAprilTag = false;
    }

    public RobotPosition getRobotPosition(){

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
        redLed1 = mainTask.hardwareMap.get(DigitalChannel.class, "red1");
        greenLed1 = mainTask.hardwareMap.get(DigitalChannel.class, "green1");


        redLed1.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed1.setMode(DigitalChannel.Mode.OUTPUT);

        redLed1.setState(true);
        greenLed1.setState(false);
    }

    public void setLedNoTag(){
        redLed1.setState(true);
        greenLed1.setState(false);
    }

    public void setLedTag(){

        redLed1.setState(false);
        greenLed1.setState(true);
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

    public enum WebCamPos{
        FrontCam,
        BackCam,
    }

    public void startTeamObjectDetection(){
        detectTeamObjectProcessor.startDetection();
    }


}
