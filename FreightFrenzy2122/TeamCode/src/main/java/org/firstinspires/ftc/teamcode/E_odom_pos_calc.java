package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is mad`e from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Forward:    Driving forward and backwards               Left-joystick Forward/Backwards
 * 2) Strafe:  Strafing right and left                     Left-joystick Right and Left
 * 3) Turn:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backwards when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="E_odom_pos", group="Linear Opmode")
//@Disabled
public class E_odom_pos_calc extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;
    private DcMotor lr = null;
    private DcMotor rf = null;
    private DcMotor rr = null;
    private DcMotor arm = null;
    private DcMotor extend = null;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;

    boolean running = false;

    private NormalizedColorSensor colorSensor = null;



    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        lf  = hardwareMap.get(DcMotor.class, "lf");
        lr  = hardwareMap.get(DcMotor.class, "lr");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "base_color");
        //arm = hardwareMap.get(DcMotor.class, "arm");
        //extend = hardwareMap.get(DcMotor.class, "lift");

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        lf.setDirection(DcMotor.Direction.FORWARD);
        lr.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);
        //arm.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Orientation             lastAngles = new Orientation();
        double                  globalAngle;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        resetAngle();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //boolean armExtend = gamepad1.dpad_up;
            //boolean armRetract = gamepad1.dpad_down;

            double leftFrontPower  = 0;
            double rightFrontPower = 0;
            double leftBackPower   = 0;
            double rightBackPower  = 0;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double Vgy   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double Vgx =  -gamepad1.left_stick_x;
            double Vrotate     =  gamepad1.right_stick_x;

            double Vry = Math.sin(DegtoRad(getAngle()))* Vgy + Math.cos(DegtoRad(getAngle())) * Vgx;
            double Vrx = Math.cos(DegtoRad(getAngle())) * Vgy - Math.sin(DegtoRad(getAngle())) * Vgx;

            double lfPower = Vry - Vrx - Vrotate;
            double rfPower = Vry + Vrx + Vrotate;
            double lrPower = Vry + Vrx - Vrotate;
            double rrPower = Vry - Vrx + Vrotate;

            double maxNumber = Math.max(Math.max(Math.abs(lfPower), Math.abs(lrPower)), Math.max(Math.abs(rfPower), Math.abs(rrPower)));
            if(maxNumber > 1){
                lfPower /= maxNumber;
                lrPower /= maxNumber;
                rfPower /= maxNumber;
                rrPower /= maxNumber;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            if(!running){
                lf.setPower(lfPower);
                rf.setPower(rfPower);
                lr.setPower(lrPower);
                rr.setPower(rrPower);
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.a
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels

//            if(gamepad1.a){
//                DriveForDisInit(1, 290.2);
//                GoToPosition(1, 0.5, 290.2);
//            }
            if (gamepad1.y){
                goStraightDistanceIMU(2,1,290.2,0.01, 0.02);
            }
//            else if (gamepad1.dpad_left){
//                turnToAngleIMU(90, 0.01, 0.02);
//            }
//            else if (gamepad1.dpad_right){
//                turnToAngleIMU(-90, 0.01, 0.02);
//            }

            if (gamepad1.a){
                lineDetection(2, 0.5, 1, 290.2, 0.01, 0.02);
            }

            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + 52runtime.toString());
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.update();
        }
    }

    private double DegtoRad(double degrees){
        double output = degrees*3.14/180;
        return output;
    }

    private void GoToPosition(double DisM, double rateOfAccel, double ppr){
        //calculate number of ticks required for distance
        //int dis_ticks = Math.round(Math.round(DisM)/(0.096f*3.1415f)*Math.round(ppr));
        double currentX = MotorAveragePos()/ppr*(0.096f*3.1415f);
        //create doubles for max speed and min speed
        double maxSped = 0.7, minSped = 0.1;

        while (Math.abs(currentX) < Math.abs(DisM)-0.05){
            currentX = MotorAveragePos()/ppr*(0.096f*3.1415f);

            //get the current x value by finding the circumference of the wheel then multiplying it to the ppr of the motor than dividing it by motor position

            double accel = minSped + rateOfAccel*currentX;
            double decel = minSped - rateOfAccel/2*(currentX-DisM);
            double sped = Math.min(accel, Math.min(decel, maxSped));
            lf.setPower(sped);
            lr.setPower(sped);
            rf.setPower(sped);
            rr.setPower(sped);

            telemetry.addData("sped", sped);
            telemetry.addData("x", currentX);
            telemetry.addData("accel", accel);
            telemetry.addData("decel", decel);
            telemetry.addData("avrg", MotorAveragePos());
            telemetry.update();

            if (gamepad1.x) {
                break;
            }
        }
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
    }

    private double MotorAveragePos(){
        double Avrg = (lr.getCurrentPosition()+lr.getCurrentPosition()+rf.getCurrentPosition()+rr.getCurrentPosition())/4;
        return Avrg;
    }

    private void DriveForDisInit(double DisM, double ppr){
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*int dis_ticks = Math.round(Math.round(DisM)/(0.096f*3.1415f)*Math.round(ppr));

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (DisM>0){
            lf.setTargetPosition(dis_ticks);
            lr.setTargetPosition(dis_ticks);
            rf.setTargetPosition(dis_ticks);
            rr.setTargetPosition(dis_ticks);
        }
        else{
            lf.setTargetPosition(dis_ticks);
            lr.setTargetPosition(dis_ticks);
            rf.setTargetPosition(dis_ticks);
            rr.setTargetPosition(dis_ticks);
        }*/
    }

    private void goStraightDistanceIMU(double DisM, double rateOfAccel, double ppr, double KP, double KD){
        //calculate number of ticks required for distance
        //int dis_ticks = Math.round(Math.round(DisM)/(0.096f*3.1415f)*Math.round(ppr));
        double startingMotorPos = (lr.getCurrentPosition()+lr.getCurrentPosition()+rf.getCurrentPosition()+rr.getCurrentPosition())/4;
        double currentX = (MotorAveragePos()-startingMotorPos)/ppr*(0.096f*3.1415f);
        //create doubles for max speed and min speed
        double maxSped = 0.7, minSped = 0.1;



        double forwardAngle = getAngle();//The angle we want to move towards
        double currentAngle = getAngle();//Our current angle
        double error = forwardAngle - currentAngle;
        double previousError = error;

        while (Math.abs(currentX) < Math.abs(DisM)-0.05){
            currentX = (MotorAveragePos()-startingMotorPos)/ppr*(0.096f*3.1415f);
            currentAngle = getAngle();

            previousError = error;
            error = forwardAngle - currentAngle;

            double derivative = error - previousError;
            double powerModifier = error * KP + KD * derivative;

            //get the current x value by finding the circumference of the wheel then multiplying it to the ppr of the motor than dividing it by motor position

            double accel = minSped + rateOfAccel*currentX;
            double decel = minSped - rateOfAccel/2*(currentX-DisM);
            double sped = Math.min(accel, Math.min(decel, maxSped));
            lf.setPower(sped-powerModifier);
            lr.setPower(sped-powerModifier);
            rf.setPower(sped+powerModifier);
            rr.setPower(sped+powerModifier);

            telemetry.addData("sped", sped);
            telemetry.addData("x", currentX);
            telemetry.addData("accel", accel);
            telemetry.addData("decel", decel);
            telemetry.addData("avrg", MotorAveragePos());
            telemetry.addData("power modifier", powerModifier);
            telemetry.update();

            if (gamepad1.x) {
                break;
            }
        }
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private void turnToAngleIMU(double targetAngle, double kp, double kd){
        double startingAngle = getAngle();
        double currentAngle = getAngle();
        double error = targetAngle - currentAngle;
        double previousError = error;

        double newError = targetAngle - currentAngle;
        double oldError = targetAngle - currentAngle;

        if(targetAngle > startingAngle){
            while (currentAngle <= targetAngle){
                //while current angle is less than ot equal to current angle (turning counter-clockwise)
                currentAngle = getAngle();

                previousError = error;
                error = targetAngle - currentAngle;

                double derivative = error - previousError;
                double power = error * kp + kd * derivative;

                lf.setPower(-0.2+power);
                rf.setPower(0.2-power);
                lr.setPower(-0.2+power);
                rr.setPower(0.2-power);

                if (gamepad1.x){
                    break;
                }
            }
        }
        else if (targetAngle < startingAngle){
            while (currentAngle >= targetAngle){
                //while current angle is greater than or equal to current angle (turning clockwise)
                currentAngle = getAngle();

                previousError = error;
                error = targetAngle - currentAngle;

                double derivative = error - previousError;
                double power = error * kp + kd * derivative;

                lf.setPower(0.2+power);
                rf.setPower(-0.2-power);
                lr.setPower(0.2+power);
                rr.setPower(-0.2-power);

                if(gamepad1.x){
                    break;
                }
            }
        }

        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);
    }

    private void lineDetection (double safeDis, double disAftLine, double rateOfAccel, double ppr, double KP, double KD){
        double redInput;
        double blueInput;
        double greenInput;
        double avgRGB;
        double targetDis;
        double startingMotorPos = (lr.getCurrentPosition()+lr.getCurrentPosition()+rf.getCurrentPosition()+rr.getCurrentPosition())/4;
        double currentX = (MotorAveragePos()-startingMotorPos)/ppr*(0.096f*3.1415f);
        //create doubles for max speed and min speed
        double maxSped = 1, minSped = 0.1;

        double forwardAngle = getAngle();//The angle we want to move towards
        double currentAngle = getAngle();//Our current angle
        double error = forwardAngle - currentAngle;
        double previousError = error;

        targetDis = safeDis;

        if (colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight)colorSensor;
            light.enableLight(!light.isLightOn());
        }

        while (Math.abs(currentX) < Math.abs(targetDis)-0.05) {
            currentX = (MotorAveragePos()-startingMotorPos)/ppr*(0.096f*3.1415f);
            currentAngle = getAngle();
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            colorSensor.setGain(5);

            previousError = error;
            error = forwardAngle - currentAngle;

            double derivative = error - previousError;
            double powerModifier = error * KP + KD * derivative;

            redInput = colors.red;
            blueInput = colors.red;
            greenInput = colors.red;

            double maxNumber = Math.max(redInput, Math.max(blueInput, greenInput));
            if(maxNumber > 1){
                redInput /= maxNumber;
                blueInput /= maxNumber;
                greenInput /= maxNumber;
            }
            avgRGB = (redInput+blueInput+greenInput)/3;

            double accel = minSped + rateOfAccel*currentX;
            double decel = minSped - rateOfAccel/2*(currentX-targetDis);
            double sped = Math.min(accel, Math.min(decel, maxSped));

            lf.setPower(sped-powerModifier);
            lr.setPower(sped-powerModifier);
            rf.setPower(sped+powerModifier);
            rr.setPower(sped+powerModifier);

            if (avgRGB>0.4){
                targetDis = disAftLine;
            }
            if (gamepad1.x) {
                break;
            }

            telemetry.addData("sped", sped);
            telemetry.addData("x", currentX);
            telemetry.addData("accel", accel);
            telemetry.addData("decel", decel);
            telemetry.addData("avrg", MotorAveragePos());
            telemetry.addData("power modifier", powerModifier);
            telemetry.update();

            telemetry.addData("red Value", redInput);
            telemetry.addData("red Value", blueInput);
            telemetry.addData("red Value", greenInput);
            telemetry.addData("Normalized Value", avgRGB);
            telemetry.update();
        }
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
    }
}