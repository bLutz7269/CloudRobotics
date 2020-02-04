package org.firstinspires.ftc.teamcode.RobotMotion;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.*;

/**
 * This is a class that tells us where the robot is on the field
 */
public class RobotLocation {
    private double xLocation;
    private double yLoctation;

    public double currentInches;

    public RobotLocation(double xLocation, double yLoctation){
        this.xLocation = xLocation;
        this.yLoctation = yLoctation;
    }

    /**
     *
     * @param currentAngle
     * @param inches
     * uses Trig to get the encoder distance of the x and y
     */
    public void setLocation(double currentAngle, double inches){
        xLocation += (cos(toRadians(currentAngle)) * inches);
        yLoctation += (sin(toRadians(currentAngle)) * inches);
    }
    

    public double getxLocation() {return xLocation;}
    public double getyLoctation() { return yLoctation; }
}