package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class PosPIDController {
    private double kp = 0;
    private double ki = 0;
    private double kd = 0;

    private double integral = 0;  // integral sum
    private double maxSumIntegral = 0.5;  // max integral sum, half full output(1.0)
    private double integralRange  = 0;  // in the error range start the integral
    private double derivative  = 0;
    private double lastDerivative = 0;
    /**
     * Exponential Moving average filter equation
     * results = k * now + (k -1) * last
     * update : last = now
     */
    private double filterAlpha = 1.0;  //filter factor, filterFac * current + (1- filterFac) * last

    private boolean shouldResetOnCross = true;  // when error change sign , reset integral

    // we normalize output to -1 ~ 1, for easy use setpower() function
    private double maxOutput = 1.0;
    private double minOutput = -1.0;

    private double target = 0;
    private double newReading = 0;
    private double lastReading = 0;
    private double error = 0;
    private double lastError = 0;

    public PosPIDController(double kp, double ki, double kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        error = 0;
        lastError = 0;
        derivative = 0;
        lastDerivative = 0;
        integral = 0;
    }
    public void setPID(double kp, double ki, double kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setTarget(double target)
    {
        this.target = target;
    }

    public void setReading(double reading)
    {
        this.newReading = reading;
    }

    public void setOutputRange(double min, double max)
    {
        this.maxOutput = max;
        this.minOutput = min;
    }

    public void setShouldResetOnCross(boolean flag)
    {
        this.shouldResetOnCross = flag;
    }
//
    public void setIntegralRange(double range)
    {
        this.integralRange = Math.abs(range);
    }
    public void setMaxSumIntegral(double maxSum)
    {
        this.maxSumIntegral = Math.abs(maxSum);
    }

    public void setFilerAlpha(double fac)
    {
        fac = Math.abs(fac) > 1 ? 1 : Math.abs(fac);
        this.filterAlpha = fac;
    }
    public double calculate(double error)
    {
        double output = 0;
        this.error = error;
        derivative = this.error - this.lastError;
        //filter derivative
        derivative = filterAlpha * derivative + (1 - filterAlpha) * lastDerivative;


        // error enter integral range, start the integration
        if (Math.abs(this.error) < Math.abs(integralRange)){
            integral += ki * error;
        }
        if (shouldResetOnCross && Math.signum(this.error) != Math.signum(lastError))
        {
            //reset, when error change sign
            integral = 0;
        }
        // clip integral into (min, max)
        integral = Range.clip(integral, -maxSumIntegral, maxSumIntegral);

        output = kp * error + integral + kd * derivative;

        output = Range.clip(output, minOutput, maxOutput);

        this.lastDerivative = this.derivative;
        this.lastError = this.error; //update
        return output;
    }

}
