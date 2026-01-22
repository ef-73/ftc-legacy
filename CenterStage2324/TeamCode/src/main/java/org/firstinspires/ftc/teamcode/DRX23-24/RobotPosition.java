package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.robot.Robot;

public class RobotPosition {
    double x, y, angle;
    public RobotPosition(double x, double y, double angle)
    {
        this.x = x;this.y = y; this.angle = angle;
    }
    public void setPosition(double x, double y, double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public void setPosition(RobotPosition pos){
        this.x = pos.x;
        this.y = pos.y;
        this.angle = pos.angle;
    }

    public double dis2Pos(RobotPosition pos){
        return Math.hypot(this.x - pos.x, this.y - pos.y);
    }


}
