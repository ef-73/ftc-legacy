package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotChassis {
    /* Public OpMode members. */
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightRearDrive = null;
    public DcMotor intake_left = null;
    public DcMotor intake_right = null;
    public DcMotor arm_tilt = null;
    public DcMotor arm_stretch = null;

    public BNO055IMU imu;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RobotChassis() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Define and Initialize Motors
        leftFrontDrive = hwMap.get(DcMotor.class, "lf_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rf_drive");
        leftRearDrive = hwMap.get(DcMotor.class, "lr_drive");
        rightRearDrive = hwMap.get(DcMotor.class, "rr_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        intake_left = hwMap.get(DcMotor.class, "intake_left");
        intake_right = hwMap.get(DcMotor.class, "intake_right");

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
        arm_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_stretch.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }
}

