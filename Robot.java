package org.firstinspires.ftc.teamcode.RobotMotion;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

abstract public class Robot extends OpMode {
    DcMotor ldb;
    DcMotor ldf;
    DcMotor rdb;
    DcMotor rdf;

    DcMotor lIntake;
    DcMotor rIntake;

    Servo lFoundation;
    Servo rFoundation;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    double globalAngle;

    private double xLocation;
    private double yLoctation;

    double left_encoder=0;
    double left_encoder_prev;
    double right_encoder=0;
    double right_encoder_prev;

    public void robotInit(){
        telemetry.addData("Status", "Wait");
        telemetry.update();

        ldb = hardwareMap.dcMotor.get("ldb");
        ldb.setDirection(DcMotor.Direction.REVERSE);
        ldb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ldb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ldf = hardwareMap.dcMotor.get("ldf");
        ldf.setDirection(DcMotor.Direction.REVERSE);
        ldf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ldf.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rdb = hardwareMap.dcMotor.get("rdb");
        rdb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rdb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rdf = hardwareMap.dcMotor.get("rdf");
        rdf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rdf.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ldb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ldf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rdb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rdf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lIntake = hardwareMap.dcMotor.get("lIntake");

        rIntake = hardwareMap.dcMotor.get("rIntake");

        lFoundation = hardwareMap.servo.get("lFoundation");
        rFoundation = hardwareMap.servo.get("rFoundation");


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initIMU();

        resetAngle();

        telemetry.addData("Say", "Are you ready kids?");
        telemetry.addData("Angle" , getAngle());
        telemetry.update();
    }

    public void initIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }

    private double getAngle() {
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

    public void Turn_degrees(Double targetRotation, double power){
        encodersOff();
        targetRotation = targetRotation-1;
        double startRotation = getAngle();
        double currentRotation = getAngle();
        double motorPower;
        double direction;

        if(startRotation > targetRotation) direction = -1;
        else direction = 1;

        while (currentRotation < targetRotation-1 || currentRotation > targetRotation+1){

            currentRotation = getAngle();
            motorPower = AnglePower(startRotation, targetRotation, currentRotation, power);
            rotMecanumPowerDrive(motorPower * direction);

            if(currentRotation > targetRotation) direction = -1;
            else direction = 1;

            telemetry.addLine("Motor Power:" + (motorPower * direction));
            telemetry.addLine("Robot Angle:" + getAngle());
            telemetry.addLine("Targer Angle:" + targetRotation);
            telemetry.update();
        }
        encodersOn();
    }

    public double AnglePower(double startRotation, double targetRotation, double currentRotation, double power){
        double anglePower;
        double angularDistance = startRotation - targetRotation;
        //double angularRatio = currentRotation/targetRotation;
        anglePower = (((currentRotation - targetRotation)/(angularDistance)*(currentRotation - targetRotation)/(angularDistance))*power + 0.15);
        return anglePower;
    }

    private void mecanumPowerDrive(double forward, double rotation){
        ldb.setPower(forward - rotation);
        ldf.setPower(forward - rotation);
        rdb.setPower(forward + rotation);
        rdf.setPower(forward + rotation);
    }

    private void rotMecanumPowerDrive(double power){
        mecanumPowerDrive(0, power);
    }

    private void encodersOff() {
        ldb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ldf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rdb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rdf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void encodersOn() {
        ldb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ldf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rdb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rdf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void driveBack(double inches, double power, Boolean opmode){
        ldf.setTargetPosition((int)(-inches * 90) + ldf.getCurrentPosition());
        ldb.setTargetPosition((int)(-inches * 90) + ldb.getCurrentPosition());
        rdb.setTargetPosition((int)(-inches * 90) + rdb.getCurrentPosition());
        rdf.setTargetPosition((int)(-inches * 90) + rdf.getCurrentPosition());

        ldf.setPower(power);
        ldb.setPower(power);
        rdb.setPower(power);
        rdf.setPower(power);
        while ((opmode && ((ldf.isBusy()) || (ldb.isBusy()) || (rdf.isBusy()) || ( rdb.isBusy())))) {
            set_LR_encoders_prev();
            setLREncoders();
            ldf.setPower(power - getError());
            ldb.setPower(power - getError());
            rdb.setPower(power + getError());
            rdf.setPower(power+ getError());
            setLocation((getAngle()),encodersToInches());
            telemetry.addData("ldf encoder-fwd", ldf.getCurrentPosition() + "  busy=" + ldf.isBusy());
            telemetry.addData("ldb encoder-fwd", ldb.getCurrentPosition() + "  busy=" + ldb.isBusy());
            telemetry.addData("rdf encoder-fwd", rdf.getCurrentPosition() + "  busy=" + rdf.isBusy());
            telemetry.addData("rdb encoder-fwd", rdb.getCurrentPosition() + "  busy=" + rdb.isBusy());
            telemetry.addData("Robot Angle:", + getAngle());
            telemetry.addData("RobotX:", xLocation);
            telemetry.addData("RobotY:", yLoctation);
            telemetry.update();
        }
    }

    public void driveForward(double inches, double power, boolean intakeAtEnd, Boolean opmode){
        ldf.setTargetPosition((int)(inches * 90) + ldf.getCurrentPosition());
        ldb.setTargetPosition((int)(inches * 90) + ldb.getCurrentPosition());
        rdb.setTargetPosition((int)(inches * 90) + rdb.getCurrentPosition());
        rdf.setTargetPosition((int)(inches * 90) + rdf.getCurrentPosition());

        ldf.setPower(power);
        ldb.setPower(power);
        rdb.setPower(power);
        rdf.setPower(power);

        intake(true);

        while ((opmode && ((ldf.isBusy()) || (ldb.isBusy()) || (rdf.isBusy()) || ( rdb.isBusy())))) {
            set_LR_encoders_prev();
            setLREncoders();
            ldf.setPower(power - getError());
            ldb.setPower(power - getError());
            rdb.setPower(power + getError());
            rdf.setPower(power+ getError());
            setLocation((getAngle()),encodersToInches());
            telemetry.addData("left Encoder", left_encoder + "  busy=" + ldf.isBusy());
            telemetry.addData("right Encoder", right_encoder + "  busy=" + rdf.isBusy());
            telemetry.addData("Robot Angle:", + getAngle());
            telemetry.addData("RobotX:", xLocation);
            telemetry.addData("RobotY:", yLoctation);
            telemetry.update();
        }

        intake(false);
    }
    public double encodersToInches(){
        double inches = (((left_encoder-left_encoder_prev)/90) + ((right_encoder-right_encoder_prev)/90))/2;
        return inches;
    }

    public double getError(){
        double error = left_encoder - right_encoder;
        error = error/40;
        return error;
    }

    public void setLREncoders(){
        left_encoder = (ldb.getCurrentPosition() + ldf.getCurrentPosition())/2;
        right_encoder = (rdb.getCurrentPosition() + rdf.getCurrentPosition())/2;
    }

    public void set_LR_encoders_prev(){
        left_encoder_prev = left_encoder;
        right_encoder_prev = right_encoder;
    }

    public void foundationGrab(Boolean grb) {
        if (grb) {
            lFoundation.setPosition(0);
            rFoundation.setPosition(1);

        } else {
            lFoundation.setPosition(1);
            rFoundation.setPosition(0);
        }
    }

    public void intake(Boolean i){
        if (i == true) {
            lIntake.setPower(-1);
            rIntake.setPower(1);
        } else {
            lIntake.setPower(0);
            rIntake.setPower(0);
        }

    }

    public void setLocation(double currentAngle, double inches){
        xLocation += (cos(toRadians(currentAngle)) * inches);
        yLoctation += (sin(toRadians(currentAngle)) * inches);
    }


    public double getxLocation() {return xLocation;}
    public double getyLoctation() { return yLoctation; }

}
