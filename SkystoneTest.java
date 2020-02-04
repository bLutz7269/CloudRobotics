package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRGyro;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotMotion.RobotLocation;

import static java.lang.Math.*;



@Autonomous(name="SkystoneTest", group="Linear Opmode")

public class SkystoneTest extends LinearOpMode {

    // Declare OpMode members.
    double fl = 0;
    double fr = 0;
    double bl = 0;
    double br = 0;

    double left_encoder = 0.0;
    double right_encoder=0.0;
    double left_encoder_prev;
    double right_encoder_prev;

    double counterAngle;

    int ldfP = 0;
    int ldbP = 0;
    int rdfP = 0;
    int rdbP = 0;

    DcMotor ldf;
    DcMotor ldb;
    DcMotor rdf;
    DcMotor rdb;

    DcMotor rIntake;
    DcMotor lIntake;

    DcMotor lift;

    Servo lGrabber;
    Servo rGrabber;
    Servo lFoundation;
    Servo rFoundation;

    ColorSensor cs;


    RobotLocation robotLocation = new RobotLocation(0,0);

    BNO055IMU imu;

    Orientation lastAngles = new Orientation();
    double globalAngle;

    private ElapsedTime t = new ElapsedTime();



    //ColorSensor csr;

    //creating a global variable that will get the color values
    int green = 0;
    int red = 0;
    int blue = 0;

    public void mecanumDrive(double lefty, double leftx, double rightx) {

        fl = lefty-leftx+rightx;
        fr = lefty+leftx-rightx;
        bl = lefty+leftx+rightx;
        br = lefty-leftx-rightx;

        if(fl>1)fl=1;
        if(fr>1)fr=1;
        if(bl>1)bl=1;
        if(br>1)br=1;
    }
    /*
    public void driveright(double inches, double power){
        ldf.setTargetPosition((int)(inches * 1120) + ldf.getCurrentPosition());
        ldf.setPower(power);
        ldb.setTargetPosition((int)(-inches * 1120) + ldb.getCurrentPosition());
        ldb.setPower(power);
        rdb.setTargetPosition((int)(inches * 1120) + rdb.getCurrentPosition());
        rdb.setPower(power);
        rdf.setTargetPosition((int)(-inches * 1120) + rdf.getCurrentPosition());
        rdf.setPower(power);
        while ((opModeIsActive() && ((ldf.isBusy()) || (ldb.isBusy()) || (rdf.isBusy()) || ( rdb.isBusy())))) {
            telemetry.addData("ldf encoder-fwd", ldf.getCurrentPosition() + "  busy=" + ldf.isBusy());
            telemetry.addData("ldb encoder-fwd", ldb.getCurrentPosition() + "  busy=" + ldb.isBusy());
            telemetry.addData("rdf encoder-fwd", rdf.getCurrentPosition() + "  busy=" + rdf.isBusy());
            telemetry.addData("rdb encoder-fwd", rdb.getCurrentPosition() + "  busy=" + rdb.isBusy());
            telemetry.update();
        }
    }

    public void driveleft(double inches, double power){
        ldf.setTargetPosition((int)(-inches * 1120) + ldf.getCurrentPosition());
        ldf.setPower(power);
        ldb.setTargetPosition((int)(inches * 1120) + ldb.getCurrentPosition());
        ldb.setPower(power);
        rdb.setTargetPosition((int)(-inches * 1120) + rdb.getCurrentPosition());
        rdb.setPower(power);
        rdf.setTargetPosition((int)(inches * 1120) + rdf.getCurrentPosition());
        rdf.setPower(power);
        while ((opModeIsActive() && ((ldf.isBusy()) || (ldb.isBusy()) || (rdf.isBusy()) || ( rdb.isBusy())))) {
            telemetry.addData("ldf encoder-fwd", ldf.getCurrentPosition() + "  busy=" + ldf.isBusy());
            telemetry.addData("ldb encoder-fwd", ldb.getCurrentPosition() + "  busy=" + ldb.isBusy());
            telemetry.addData("rdf encoder-fwd", rdf.getCurrentPosition() + "  busy=" + rdf.isBusy());
            telemetry.addData("rdb encoder-fwd", rdb.getCurrentPosition() + "  busy=" + rdb.isBusy());
            telemetry.update();
        }
    }
     */
    public void goToPosition(double targetx, double targety){

        //Lines up with the x and then moves either back or forward depending on the target location
        Turn_degrees(0.0);
        if(targetx < robotLocation.getxLocation())driveBack(robotLocation.getxLocation()-targetx, 75);
        else driveForward(targetx-robotLocation.getxLocation(), 0.75);

        //checks to see which angle to point to depending on the direction of y target location
        if(targety<robotLocation.getyLoctation())Turn_degrees(90.0);
        else Turn_degrees(-90.0);
        driveBack(robotLocation.getyLoctation()-targety, 0.75);

    }

    public void driveForward(double inches, double power){
        ldf.setTargetPosition((int)(inches * 90) + ldf.getCurrentPosition());
        ldb.setTargetPosition((int)(inches * 90) + ldb.getCurrentPosition());
        rdb.setTargetPosition((int)(inches * 90) + rdb.getCurrentPosition());
        rdf.setTargetPosition((int)(inches * 90) + rdf.getCurrentPosition());

        ldf.setPower(power);
        ldb.setPower(power);
        rdb.setPower(power);
        rdf.setPower(power);
        while ((opModeIsActive() && ((ldf.isBusy()) || (ldb.isBusy()) || (rdf.isBusy()) || ( rdb.isBusy())))) {
            ldf.setPower(power - getError());
            ldb.setPower(power - getError());
            rdb.setPower(power + getError());
            rdf.setPower(power+ getError());
            telemetry.addData("left Encoder", left_encoder + "  busy=" + ldf.isBusy());
            telemetry.addData("right Encoder", right_encoder + "  busy=" + rdf.isBusy());
            telemetry.addData("Robot Angle:", + getAngle()+counterAngle);
            telemetry.addData("RobotX:", robotLocation.getxLocation());
            telemetry.addData("RobotY:", robotLocation.getyLoctation());
            telemetry.update();
        }
        robotLocation.setLocation((getAngle()+ counterAngle), inches);
    }

    public void driveForward(double inches, double power, boolean intakeAtEnd){
        ldf.setTargetPosition((int)(inches * 90) + ldf.getCurrentPosition());
        ldb.setTargetPosition((int)(inches * 90) + ldb.getCurrentPosition());
        rdb.setTargetPosition((int)(inches * 90) + rdb.getCurrentPosition());
        rdf.setTargetPosition((int)(inches * 90) + rdf.getCurrentPosition());

        ldf.setPower(power);
        ldb.setPower(power);
        rdb.setPower(power);
        rdf.setPower(power);

        intake(true);

        while ((opModeIsActive() && ((ldf.isBusy()) || (ldb.isBusy()) || (rdf.isBusy()) || ( rdb.isBusy())))) {
            set_LR_encoders_prev();
            setLREncoders();
            ldf.setPower(power - getError());
            ldb.setPower(power - getError());
            rdb.setPower(power + getError());
            rdf.setPower(power+ getError());
            robotLocation.setLocationDuring((getAngle()+counterAngle),encodersToInches());
            telemetry.addData("left Encoder", left_encoder + "  busy=" + ldf.isBusy());
            telemetry.addData("right Encoder", right_encoder + "  busy=" + rdf.isBusy());
            telemetry.addData("Robot Angle:", + getAngle()+counterAngle);
            telemetry.addData("RobotX:", robotLocation.getxLocation());
            telemetry.addData("RobotY:", robotLocation.getyLoctation());
            telemetry.update();
        }
        robotLocation.setLocation((getAngle()+ counterAngle), inches);

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


    public void driveBack(double inches, double power){
        ldf.setTargetPosition((int)(-inches * 90) + ldf.getCurrentPosition());
        ldb.setTargetPosition((int)(-inches * 90) + ldb.getCurrentPosition());
        rdb.setTargetPosition((int)(-inches * 90) + rdb.getCurrentPosition());
        rdf.setTargetPosition((int)(-inches * 90) + rdf.getCurrentPosition());

        ldf.setPower(power);
        ldb.setPower(power);
        rdb.setPower(power);
        rdf.setPower(power);
        while ((opModeIsActive() && ((ldf.isBusy()) || (ldb.isBusy()) || (rdf.isBusy()) || ( rdb.isBusy())))) {
            telemetry.addData("ldf encoder-fwd", ldf.getCurrentPosition() + "  busy=" + ldf.isBusy());
            telemetry.addData("ldb encoder-fwd", ldb.getCurrentPosition() + "  busy=" + ldb.isBusy());
            telemetry.addData("rdf encoder-fwd", rdf.getCurrentPosition() + "  busy=" + rdf.isBusy());
            telemetry.addData("rdb encoder-fwd", rdb.getCurrentPosition() + "  busy=" + rdb.isBusy());
            telemetry.addData("Robot Angle:", + getAngle()+ counterAngle);
            telemetry.addData("RobotX:", robotLocation.getxLocation());
            telemetry.addData("RobotY:", robotLocation.getyLoctation());
            telemetry.update();
        }
        robotLocation.setLocation((getAngle()+counterAngle)+180, inches);
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

    public void colorTest(){
        red = cs.red();
        green = cs.green();
        blue = cs.blue();
        telemetry.addData("Blue", blue);
        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.update();
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

    /**
     *
     * @param targetRotation
     * This code will allow the robot to turn to __ angle
     * get the curent angle and add or subtract to turn __ degrees from current angle
     */
    public void Turn_degrees(Double targetRotation){
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
            motorPower = AnglePower(startRotation, targetRotation, currentRotation);
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

    public double AnglePower(double startRotation, double targetRotation, double currentRotation){
        double anglePower;
        double angularDistance = startRotation - targetRotation;
        double angularRatio = currentRotation/targetRotation;
        anglePower = (((currentRotation - targetRotation)/(angularDistance)*(currentRotation - targetRotation)/(angularDistance)) + 0.15);
        return anglePower;
    }

    private void mecanumPowerDrive(double forward, double rotation){
        ldb.setPower(forward - rotation);
        ldf.setPower(forward - rotation);
        rdb.setPower(forward + rotation);
        rdf.setPower(forward + rotation);
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private void rotMecanumPowerDrive(double power){
        mecanumPowerDrive(0, power);
    }

    private void strMecanumPowerDrive(double power){
        mecanumPowerDrive(power, 0);
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


    public void initialization(){
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

        if(getAngle() > 0)counterAngle = -getAngle();
        else counterAngle = getAngle();

        telemetry.addData("Say", "Are you ready kids?");
        telemetry.addData("Angle" , getAngle());
        telemetry.addData("counterAngle", counterAngle);
        telemetry.update();

    }


    public void runOpMode(){
        initialization();
        waitForStart();

        // run until finishes all of the tasks or the driver presses stop
        while (opModeIsActive()) {
            resetAngle();
            if(getAngle() > 0)counterAngle = -getAngle();
            else counterAngle = getAngle();

            driveForward(40, 0.4, true);

            t.reset();
            while(t.time()<0.5){
                telemetry.addData("RobotX:", robotLocation.getxLocation());
                telemetry.addData("RobotY:", robotLocation.getyLoctation());
                telemetry.update();
            }
            //Turn_degrees(getAngle() + 90.0);

            //driveForward(15,0.75);
            /*
            goToPosition(18.0, -35.0);

            t.reset();
            while(t.time()<5){
                telemetry.addData("RobotX:", robotLocation.getxLocation());
                telemetry.addData("RobotY:", robotLocation.getyLoctation());
                telemetry.update();
            }
            */
            stop();
        }
    }
}