package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Ethan_config {

    public DcMotor  leftFrontDrive = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftRearDrive   = null;
    public DcMotor  rightRearDrive  = null;
    public DcMotor intake_left = null;
    public DcMotor intake_right = null;
    public DcMotor arm_tilt = null;
    public DcMotor arm_stretch = null;
    public Servo claw_rotate = null;
    public Servo claw_move = null;
    //public Servo hook_left = null;
    //public Servo hook_right = null;
    public BNO055IMU imu;
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public Ethan_config(){
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front");
        leftRearDrive  = hwMap.get(DcMotor.class, "left_rear");
        rightRearDrive = hwMap.get(DcMotor.class, "right_rear");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        intake_left  = hwMap.get(DcMotor.class, "left_intake");
        intake_right = hwMap.get(DcMotor.class, "right_intake");
        arm_tilt  = hwMap.get(DcMotor.class, "arm_tilt");
        arm_stretch = hwMap.get(DcMotor.class, "arm_stretch");

        //intake
        intake_left.setDirection(DcMotor.Direction.FORWARD);
        intake_right.setDirection(DcMotor.Direction.REVERSE);
        arm_tilt.setDirection(DcMotor.Direction.FORWARD);
        arm_stretch.setDirection(DcMotor.Direction.FORWARD);




        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
        intake_left.setPower(0);
        intake_right.setPower(0);
        arm_stretch.setPower(0);
        arm_tilt.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_stretch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_stretch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // set left motor to run to target encoder position and stop with brakes on.
        //arm_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //arm_stretch.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        //servo
        // Define and initialize ALL installed servos.
        claw_rotate  = hwMap.get(Servo.class, "claw_tilt");
        claw_move = hwMap.get(Servo.class, "claw_move");
        claw_rotate.setPosition(0.5);     //middle position
        claw_move.setPosition(0.5);     //middle position
        /*
        hook_left  = hwMap.get(Servo.class, "hook_left");
        hook_right = hwMap.get(Servo.class, "hook_right");
        hook_left.setPosition(0.25);     //middle position
        hook_right.setPosition(0.78);     //middle position
        */
        /*
        //color sensor
        // Get a reference to our sensor object.
        // get a reference to the color sensor.
        sensorColor = hwMap.get(ColorSensor.class, "color_sensor");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hwMap.get(DistanceSensor.class, "color_sensor");
    */

    }


}

