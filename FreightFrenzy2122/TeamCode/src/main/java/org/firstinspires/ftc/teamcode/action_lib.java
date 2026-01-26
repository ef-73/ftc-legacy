package org.firstinspires.ftc.teamcode;

public class action_lib {

    robot_hardware robot;
    action_lib(robot_hardware robot){
        this.robot = robot;
    }

    //your functions go here
    void run_drive(double lf,double rf,double rr,double lr){
        this.robot.lf_drive.setPower(lf);
        this.robot.lr_drive.setPower(lr);
        this.robot.rr_drive.setPower(rr);
        this.robot.rf_drive.setPower(rf);
    }
    void stop_drive(){
        this.robot.lf_drive.setPower(0);
        this.robot.lr_drive.setPower(0);
        this.robot.rr_drive.setPower(0);
        this.robot.rf_drive.setPower(0);
    }
}
