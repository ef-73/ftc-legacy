package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/*
4 rings = C (Farthest)
1 ring = B (Middle)
0 rings = A (Closest)
 */
@Autonomous(name="test")
public class AutonomousRings extends LinearOpMode {
    MecanumChassis robot   = new MecanumChassis();
    DcMotor lf,rf,lr,rr;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    int ringHeight;
    int box;
    private static int FIRST_BOX = 1;
    private static int SECOND_BOX = 2;
    private static int THIRD_BOX = 3;
    private static final String VUFORIA_KEY =
            "ASOYVAj/////AAABmeLWppvv2E3aulir9L58q2c9jnovtgBUKGOLf6fQRRl2Gmimfb6klxUTuUN4LrMvt1f67Z30a8JuLdxFRlq0VUETIh1E4MUANlTcBWjIT5fg8XYN5C/zIenRIy70ABp5uZ1XlbaWQ9jz38leD/fPbed0WjSN+D6Nmkv9FkcInu8tbv16uB8uWXMUEBcYAnejcYvys1ohlAdc6s1+sWI0QXSawYUOHQoV1hsmY6WpysBbGYv3lQFprY9AyBT69A9ju78WZAm4KAXGugGnD9n1wWMtIJfxo4BYfFtTFJNFI7nnv0EyRB6eZPkq0ScbcYP/z3MoKcFixKvrT9Q47TEI9VZeyLK9GujKZxTIM5PtELbt";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private ElapsedTime runtime = new ElapsedTime();
    // State used for updating telemetry
    volatile Orientation angles;        //just for telemetry
    Acceleration gravity;
    //Mecanum auto drive class
    MecanumAutoDrive mecanumAutoDrive = null;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d:%7d:%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition());

        telemetry.addData("IMU Mode", "IMU calibrating....");
        telemetry.update();

        //make sure the IMU gyro is calibrated before continue
        while(!isStopRequested() && ! robot.imu.isGyroCalibrated() &&
                ! robot.imu.isAccelerometerCalibrated() &&
                ! robot.imu.isMagnetometerCalibrated() &&
                ! robot.imu.isSystemCalibrated())
        {
            idle();
        }
        telemetry.update();


        telemetry.addData("Working Mode", "waiting for start");
        telemetry.update();
        mecanumAutoDrive = new MecanumAutoDrive(robot, telemetry);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();
        robot.arm.setTargetPosition(-100);
        robot.arm.setPower(0.5);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //distances are in meters
        mecanumAutoDrive.goStraightTask(0.1,0,0.5, 0, 5);
        sleep(100);
        for (int count =0;count<200;count++) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        ringHeight = Math.abs((int) (recognition.getTop() - recognition.getBottom()));




                        telemetry.addData("ring height", ringHeight);
                        telemetry.addData("box", box);
                        telemetry.update();

                    }
                    telemetry.update();
                }
            }
        }
        mecanumAutoDrive.goStraightTask(0.45, 0, 0.5,1, 10);
        sleep(100);
        robot.shooter.setPower(0.85);
        sleep(1200);
        robot.roller.setPower(1);
        robot.shooter.setPower(0.85);
        sleep(1000);
        robot.roller.setPower(0);
        sleep(500);
        robot.roller.setPower(1);
        sleep(1000);
        robot.shooter.setPower(0.9);
        robot.roller.setPower(0);
        sleep(500);
        robot.roller.setPower(1);
        sleep(1000);
        robot.shooter.setPower(0);
        robot.roller.setPower(0);

        if (tfod != null) {
            tfod.shutdown();
        }
        if (ringHeight > 120){
            //4 rings
            mecanumAutoDrive.goStraightTask(0.6, 0, 0.5,10, 10);

            mecanumAutoDrive.turnRobotTask(180, 0.5, 5, MecanumAutoDrive.TURN_METHOD.TWO_WHEEL, 10);
            mecanumAutoDrive.straferTask(0.1, 180, -0.6, 1, 10);
            moveArm(-600);
            robot.claw.setPosition(0.4);
            sleep(100);
            box = FIRST_BOX;
            //The farthest box
            robot.arm.setTargetPosition(400);
            robot.arm.setPower(0.5);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            mecanumAutoDrive.goStraightTask(0.4, 180, 0.5, 10, 10);
            //mecanumAutoDrive.straferTask(0.08, 180, 0.6, 1, 10);
            //mecanumAutoDrive.goStraightTask(1.03, 180, 0.5, 10, 10);
        } else if (ringHeight > 1){
            //1 ring
            mecanumAutoDrive.goStraightTask(0.35, 0, 0.5, 10., 10);

            mecanumAutoDrive.turnRobotTask(180, 0.5, 5, MecanumAutoDrive.TURN_METHOD.TWO_WHEEL, 10);
            moveArm(-600);
            robot.claw.setPosition(0.4);
            sleep(100);
            box = SECOND_BOX;
            //middle box
            robot.arm.setTargetPosition(400);
            robot.arm.setPower(0.5);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            //mecanumAutoDrive.straferTask(0.02, 180, -0.6, 1, 10);
            //mecanumAutoDrive.goStraightTask(0.78, 180, 0.5, 10, 10);
        } else {
            //no rings
            mecanumAutoDrive.goStraightTask(0.1, 0, 0.5, 10, 10);

            mecanumAutoDrive.turnRobotTask(180, 0.5, 5, MecanumAutoDrive.TURN_METHOD.TWO_WHEEL, 10);
            mecanumAutoDrive.straferTask(0.1, 180, -0.6, 1, 10);
            moveArm(-600);
            robot.claw.setPosition(0.4);
            sleep(100);
            box = THIRD_BOX;
            //closest box
            robot.arm.setTargetPosition(400);
            robot.arm.setPower(0.5);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            //mecanumAutoDrive.straferTask(0.08, 180, 0.6, 1, 10);
            //mecanumAutoDrive.goStraightTask(0.52, 180, 0.5, 10, 10);
        }
        /*
        robot.arm.setTargetPosition(-600);
        robot.arm.setPower(0.3);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(robot.arm.isBusy()){

        }
        sleep(1000);
        robot.claw.setPosition(0);
        sleep(300);
        robot.arm.setTargetPosition(400);
        if (box==1){
            mecanumAutoDrive.goStraightTask(1.03, 180, -0.5, 10, 10);
            mecanumAutoDrive.straferTask(0.08, 180, -0.6, 1, 10);
        } else if (box == 2){
            mecanumAutoDrive.goStraightTask(0.78, 180, -0.5, 10, 10);
            mecanumAutoDrive.straferTask(0.02, 180, 0.6, 1, 10);
        } else {
            mecanumAutoDrive.goStraightTask(0.53, 180, -0.5, 10, 10);
            mecanumAutoDrive.straferTask(0.08, 180, -0.6, 1, 10);
        }
        robot.claw.setPosition(0.4);
        sleep(500);
        */
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    private void moveArm(int pos) {
        robot.arm.setTargetPosition(3*pos/5);
        robot.arm.setPower(0.5);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.arm.isBusy()) {}
        robot.arm.setTargetPosition(pos);
        robot.arm.setPower(0.1);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.arm.isBusy()) {}
    }
}
