package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="DriveForward4inches", group="Linear Opmode")

public class DriveForward4Inches extends LinearOpMode {

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

    DcMotor lift;

    Servo lGrabber;
    Servo rGrabber;
    Servo lFoundation;
    Servo rFoundation;

    GyroSensor gyro;

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
            telemetry.update();
        }
    }

    public void turn90r( double power, double times){
        int distance = 2100;
        distance = (int)(distance * times);
        ldf.setTargetPosition((distance) + ldf.getCurrentPosition());
        ldf.setPower(power);
        ldb.setTargetPosition((distance) + ldb.getCurrentPosition());
        ldb.setPower(power);
        rdb.setTargetPosition((-distance) + rdb.getCurrentPosition());
        rdb.setPower(power);
        rdf.setTargetPosition((-distance) + rdf.getCurrentPosition());
        rdf.setPower(power);
        while ((opModeIsActive() && ((ldf.isBusy()) || (ldb.isBusy()) || (rdf.isBusy()) || ( rdb.isBusy())))) {
            telemetry.addData("ldf encoder-fwd", ldf.getCurrentPosition() + "  busy=" + ldf.isBusy());
            telemetry.addData("ldb encoder-fwd", ldb.getCurrentPosition() + "  busy=" + ldb.isBusy());
            telemetry.addData("rdf encoder-fwd", rdf.getCurrentPosition() + "  busy=" + rdf.isBusy());
            telemetry.addData("rdb encoder-fwd", rdb.getCurrentPosition() + "  busy=" + rdb.isBusy());
            telemetry.update();
        }
    }
    public void turn90l( double power, double times){
        int distance = 2100;
        distance = (int)(distance * times);
        ldf.setTargetPosition((-distance) + ldf.getCurrentPosition());
        ldf.setPower(power);
        ldb.setTargetPosition((-distance) + ldb.getCurrentPosition());
        ldb.setPower(power);
        rdb.setTargetPosition((distance) + rdb.getCurrentPosition());
        rdb.setPower(power);
        rdf.setTargetPosition((distance) + rdf.getCurrentPosition());
        rdf.setPower(power);
        while ((opModeIsActive() && ((ldf.isBusy()) || (ldb.isBusy()) || (rdf.isBusy()) || ( rdb.isBusy())))) {
            telemetry.addData("ldf encoder-fwd", ldf.getCurrentPosition() + "  busy=" + ldf.isBusy());
            telemetry.addData("ldb encoder-fwd", ldb.getCurrentPosition() + "  busy=" + ldb.isBusy());
            telemetry.addData("rdf encoder-fwd", rdf.getCurrentPosition() + "  busy=" + rdf.isBusy());
            telemetry.addData("rdb encoder-fwd", rdb.getCurrentPosition() + "  busy=" + rdb.isBusy());
            telemetry.update();
        }
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

    public void gyroStart(){
        gyro.calibrate();
    }

    public void setAngle(int angle ) {
        while(gyro.rawX() < angle-0.5 || gyro.rawX()> angle+0.5 )
        if (gyro.rawX() > angle) {
            turn90l(.5, 0.1);
        } else if (gyro.rawX()< angle){
            turn90r(.5,0.1);
        }
    }

    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
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



        lIntake = hardwareMap.dcMotor.get("lIntake");
        lIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rIntake = hardwareMap.dcMotor.get("rIntake");
        rIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lGrabber = hardwareMap.servo.get("lGrabber");
        rGrabber = hardwareMap.servo.get("rGrabber");

        lFoundation = hardwareMap.servo.get("lFoundation");
        rFoundation = hardwareMap.servo.get("rFoundation");

        telemetry.addData("Say", "Are you ready kids?");
        telemetry.update();

        //csr = hardwareMap.colorSensor.get("csr");
        //csr.enableLed(true);

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // driveleft is wackish
            ldfP = 0;
            ldbP = 0;
            rdfP = 0;
            rdbP = 0;

            driveForward(12, .75);
            stop();
        }
    }
}