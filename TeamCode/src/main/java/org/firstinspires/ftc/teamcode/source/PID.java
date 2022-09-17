package org.firstinspires.ftc.teamcode.source;
import static java.lang.Double.NaN;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is a simple PID controller class that can be used for nearly any system.  There is no feedforward control feature, so you will have to add that separately to the controller output if needed.
 */
public class PID {
    private double P;
    private double I;
    private double D;

    private double It = 0;
    private double dt = 0.05;
    public double output;
    public double Err;
    private double pErr = NaN;

    private double min;
    private double max;

    private double kP;
    private double kI;
    private double kD;
    private double integralErrCap;
    private ElapsedTime timer = new ElapsedTime();

    public PID() {}

    public void setCoefficients(double kP, double kI, double kD, double minOutput, double maxOutput, double integralErrCap) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralErrCap = integralErrCap;
        this.min = minOutput;
        this.max = maxOutput;
    }
    public void update(double setpoint, double processValue) {
        dt = timer.milliseconds()/1000;
        timer.reset();
        Err = setpoint - processValue;
        if(pErr == NaN) {
            pErr = Err;
        }
        P = kP * Err;
        if(Math.abs(Err) < integralErrCap) {
            I = kI * Err * dt;
            It += I;
        } else {
            It = 0;
        }
        D = -1 * kD * (pErr - Err)/dt;
        output = Range.clip(P + It + D, min, max);
        pErr = Err;
    }
}
