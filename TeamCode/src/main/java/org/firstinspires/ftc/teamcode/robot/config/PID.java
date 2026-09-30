package org.firstinspires.ftc.teamcode.robot.config;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private double Kp;
    private double Ki;
    private double Kd;
    private double integral = 0;
    private double previousError = 0;
    ElapsedTime timer = new ElapsedTime();

    public PID(double Kp, double Ki, double Kd, double flywheelKf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double out(double referencePos, double currentPos) {
        double error = referencePos - currentPos;
        double derivative = (error - previousError) / timer.seconds();
        integral += error * timer.seconds();

        double _o = (Kp * error) + (Ki * integral) + (Kd * derivative);

        previousError = error;
        timer.reset();
        return _o;
    }
}
