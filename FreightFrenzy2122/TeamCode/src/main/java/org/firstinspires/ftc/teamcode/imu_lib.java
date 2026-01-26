package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//
public class imu_lib {
    //robot hardware definitions
    robot_hardware robot;
    action_lib     action;

    //IMU overhead
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;


    public imu_lib(robot_hardware robot,action_lib action){
        this.robot = robot;
        this.action = action;
    }
    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {

        Orientation angles = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double getProportionalTerm(double target,double gain)
    {
        double correction, angle;
        angle = getAngle();
        double error  = target-angle;
        correction = error * gain;
        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power)
    {
        int dir =1;


        if (degrees < globalAngle)
        {   // turn right.
            dir=1;
        }
        else if (degrees > globalAngle)
        {   // turn left.
            dir=-1;
        }
        else return;

        // set power to rotate.
        this.action.run_drive(power*dir,power*dir,power*dir,power*dir);

        // rotate until turn is completed.
        if (degrees < globalAngle)
        {
            while (getAngle() > degrees) {}
        }
        else    // left turn.
            while (getAngle() < degrees) {}

        // turn the motors off.
        this.action.stop_drive();
    }

}
