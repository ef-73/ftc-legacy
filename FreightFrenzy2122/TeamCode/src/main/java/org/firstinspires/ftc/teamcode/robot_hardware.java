package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class robot_hardware {
    HardwareMap hw = null;
    public DcMotor lf_drive,lr_drive,rr_drive,rf_drive,arm,left_intake,right_intake;
    public Servo claw1,claw2,grip;
    public BNO055IMU imu;
    public robot_hardware(){

    }
    public void init(HardwareMap hwMap, Telemetry telemetry){
        hw = hwMap;
        try {
            lf_drive = hw.get(DcMotor.class, "lf_drive");
            lr_drive = hw.get(DcMotor.class, "lr_drive");
            rf_drive = hw.get(DcMotor.class, "rf_drive");
            rr_drive = hw.get(DcMotor.class, "rr_drive");

            left_intake = hw.get(DcMotor.class,"left_intake");
            right_intake = hw.get(DcMotor.class,"right_intake");
            claw1 = hw.get(Servo.class, "claw1");
            claw2 = hw.get(Servo.class, "claw2");
            arm = hw.get(DcMotor.class, "arm");
            grip = hw.get(Servo.class, "grip");
            lf_drive.setDirection(DcMotorSimple.Direction.REVERSE);
            lr_drive.setDirection(DcMotorSimple.Direction.REVERSE);
            imu = hw.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;
            imu.initialize(parameters);

            telemetry.addData("Mode", "calibrating...");
            telemetry.update();

            // make sure the imu gyro is calibrated before continuing.
            while (!imu.isGyroCalibrated()) {
                //idle
            }
        }
        catch (Exception e){
            telemetry.addLine("failed initialization, check your ids");
            telemetry.update();
        }
    }

}
