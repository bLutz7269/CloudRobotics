package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.BM.BalisticProfile;


@Autonomous(name="DriveTurns", group="Linear Opmode")

public class DriveTurns extends LinearOpMode {

    // Declare OpMode members.
    double fl = 0;
    double fr = 0;
    double bl = 0;
    double br = 0;

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

    Servo lFoundation;
    Servo rFoundation;

    double acceleration_distance= 15;
    double deceleration_distance=60;
    double start_power=0.175;
    double full_power=.6;
    double end_power=0.12;
    BalisticProfile.CURVE_TYPE acceleration_curve = BalisticProfile.CURVE_TYPE.SINUSOIDAL_NORMAL;
    BalisticProfile.CURVE_TYPE deceleration_curve = BalisticProfile.CURVE_TYPE.SINUSOIDAL_NORMAL;

    private BalisticProfile rotationBalisticProfile = new BalisticProfile(acceleration_distance, deceleration_distance, start_power, full_power,end_power, acceleration_curve, deceleration_curve);
    private BalisticProfile buildplateProfile = new BalisticProfile(20, 60, 0.2, 0.7,0.15, acceleration_curve, deceleration_curve);
    private BalisticProfile regularProfile = rotationBalisticProfile;
    BNO055IMU imu;

    Orientation lastAngles = new Orientation();
    double globalAngle;

    private ElapsedTime t = new ElapsedTime();

    //ColorSensor csr;

    //creating a global variable that will get the value of green
    int green = 0;

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
            telemetry.addData("Robot Angle:", + getAngle());
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
            telemetry.addData("Robot Angle:", + getAngle());
            telemetry.update();
        }
    }

    public void driveForward(double inches, double power){
        ldf.setTargetPosition((int)(inches * 94) + ldf.getCurrentPosition());
        ldf.setPower(power);
        ldb.setTargetPosition((int)(inches * 94) + ldb.getCurrentPosition());
        ldb.setPower(power);
        rdb.setTargetPosition((int)(inches * 94) + rdb.getCurrentPosition());
        rdb.setPower(power);
        rdf.setTargetPosition((int)(inches * 94) + rdf.getCurrentPosition());
        rdf.setPower(power);
        while ((opModeIsActive() && ((ldf.isBusy()) || (ldb.isBusy()) || (rdf.isBusy()) || ( rdb.isBusy())))) {
            telemetry.addData("ldf encoder-fwd", ldf.getCurrentPosition() + "  busy=" + ldf.isBusy());
            telemetry.addData("ldb encoder-fwd", ldb.getCurrentPosition() + "  busy=" + ldb.isBusy());
            telemetry.addData("rdf encoder-fwd", rdf.getCurrentPosition() + "  busy=" + rdf.isBusy());
            telemetry.addData("rdb encoder-fwd", rdb.getCurrentPosition() + "  busy=" + rdb.isBusy());
            telemetry.addData("Robot Angle:", + getAngle());
            telemetry.update();
        }
    }

    public void driveBack(double inches, double power){
        ldf.setTargetPosition((int)(-inches * 94) + ldf.getCurrentPosition());
        ldf.setPower(power);
        ldb.setTargetPosition((int)(-inches * 94) + ldb.getCurrentPosition());
        ldb.setPower(power);
        rdb.setTargetPosition((int)(-inches * 94) + rdb.getCurrentPosition());
        rdb.setPower(power);
        rdf.setTargetPosition((int)(-inches * 94) + rdf.getCurrentPosition());
        rdf.setPower(power);
        while ((opModeIsActive() && ((ldf.isBusy()) || (ldb.isBusy()) || (rdf.isBusy()) || ( rdb.isBusy())))) {
            telemetry.addData("ldf encoder-fwd", ldf.getCurrentPosition() + "  busy=" + ldf.isBusy());
            telemetry.addData("ldb encoder-fwd", ldb.getCurrentPosition() + "  busy=" + ldb.isBusy());
            telemetry.addData("rdf encoder-fwd", rdf.getCurrentPosition() + "  busy=" + rdf.isBusy());
            telemetry.addData("rdb encoder-fwd", rdb.getCurrentPosition() + "  busy=" + rdb.isBusy());
            telemetry.addData("Robot Angle:", + getAngle());
            telemetry.update();
        }
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

    public void foundationGrab(Boolean grb) {
        if (grb) {
            lFoundation.setPosition(0);
            rFoundation.setPosition(1);

        } else {
            lFoundation.setPosition(1);
            rFoundation.setPosition(0);
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



    /**
     *
     * @param targetRotation
     * This code will allow the robot to turn to __ angle
     * get the curent angle and add or subtract to turn __ degrees from current angle
     */
    public void Turn_degrees(Double targetRotation){
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
    }

    public double AnglePower(double startRotation, double targetRotation, double currentRotation){
        double anglePower;
        double angularDistance = startRotation - targetRotation;
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


    private void initilization(){
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
        lIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rIntake = hardwareMap.dcMotor.get("rIntake");
        rIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lFoundation = hardwareMap.servo.get("lFoundation");
        rFoundation = hardwareMap.servo.get("rFoundation");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initIMU();

        resetAngle();

        telemetry.addData("Say", "Are you ready kids?");
        telemetry.update();

        //csr = hardwareMap.colorSensor.get("csr");
        //csr.enableLed(true);

    }


    public void runOpMode(){
        initilization();
        resetAngle();


        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            rotationBalisticProfile = regularProfile;
            // driveleft is wackish
            ldfP = 0;
            ldbP = 0;
            rdfP = 0;
            rdbP = 0;
            foundationGrab(true);
            encodersOff();
            ldb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ldf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rdb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rdf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Turn_degrees(getAngle() - 90.0);
            foundationGrab(false);
            encodersOn();
            t.reset();
            while(time<5);
            stop();
        }
    }
}