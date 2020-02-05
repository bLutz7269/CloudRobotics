package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotMotion.Robot;
import org.firstinspires.ftc.teamcode.RobotMotion.RobotLocation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "RedopencvSkystoneDetector", group="Sky autonomous")
//@Disabled//comment out this line before using
public class RedOpenCvSkystone extends LinearOpMode {

    double fl = 0;
    double fr = 0;
    double bl = 0;
    double br = 0;

    double left_encoder = 0.0;
    double right_encoder=0.0;
    double left_encoder_prev;
    double right_encoder_prev;

    double counterAngle;

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

    Robot robot;

    RobotLocation robotLocation = new RobotLocation(0,0);

    BNO055IMU imu;

    Orientation lastAngles = new Orientation();
    double globalAngle;

    private ElapsedTime t = new ElapsedTime();


    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 1.2f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    //@Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId;
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_RIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
        String loc;

        initialization();

        waitForStart();
        while (opModeIsActive()) {
             loc = runCamera(5);
             runtime.reset();
             while(runtime.time() < 5) {
                 telemetry.addLine(loc);
                 telemetry.update();
             }
             if(loc == "Left"){
                 robot.Turn_degrees(15.0, 0.5);
                 robot.driveForward(40, 0.7, true, opModeIsActive());

             }else if(loc == "Mid"){

                 robot.driveForward(40, 0.7, true,  opModeIsActive());

             }else if(loc == "Right"){
                 robot.Turn_degrees(-15.0, 0.5);
                 robot.driveForward(40, 0.7, true, opModeIsActive());

             }

        }
    }

    public void initializeOpenCv(){
        OpenCvCamera phoneCam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_RIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
    }

    public String runCamera(double time){
        String location;
        runtime.reset();
        while(runtime.time() < time) {
            telemetry.addData("Left", valLeft);
            telemetry.addData("Middle", valMid);
            telemetry.addData("Right", valRight);
            telemetry.update();
            sleep(100);
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        }
        if(valLeft == 0)location = "Left";
        else if(valMid==0)location = "Mid";
        else location = "Right";

        return location;
    }


    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
    public void mecanumDrive(double lefty, double leftx, double rightx) {

        fl = lefty - leftx + rightx;
        fr = lefty + leftx - rightx;
        bl = lefty + leftx + rightx;
        br = lefty - leftx - rightx;

        if (fl > 1) fl = 1;
        if (fr > 1) fr = 1;
        if (bl > 1) bl = 1;
        if (br > 1) br = 1;
    }

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
            set_LR_encoders_prev();
            setLREncoders();
            ldf.setPower(power - getError());
            ldb.setPower(power - getError());
            rdb.setPower(power + getError());
            rdf.setPower(power+ getError());
            robotLocation.setLocation((getAngle()+counterAngle),encodersToInches());
            telemetry.addData("left Encoder", left_encoder + "  busy=" + ldf.isBusy());
            telemetry.addData("right Encoder", right_encoder + "  busy=" + rdf.isBusy());
            telemetry.addData("Robot Angle:", + getAngle()+counterAngle);
            telemetry.addData("RobotX:", robotLocation.getxLocation());
            telemetry.addData("RobotY:", robotLocation.getyLoctation());
            telemetry.update();
        }
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
            robotLocation.setLocation((getAngle()+counterAngle),encodersToInches());
            telemetry.addData("left Encoder", left_encoder + "  busy=" + ldf.isBusy());
            telemetry.addData("right Encoder", right_encoder + "  busy=" + rdf.isBusy());
            telemetry.addData("Robot Angle:", + getAngle()+counterAngle);
            telemetry.addData("RobotX:", robotLocation.getxLocation());
            telemetry.addData("RobotY:", robotLocation.getyLoctation());
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
            set_LR_encoders_prev();
            setLREncoders();
            ldf.setPower(power - getError());
            ldb.setPower(power - getError());
            rdb.setPower(power + getError());
            rdf.setPower(power+ getError());
            robotLocation.setLocation((getAngle()+counterAngle),encodersToInches());
            telemetry.addData("ldf encoder-fwd", ldf.getCurrentPosition() + "  busy=" + ldf.isBusy());
            telemetry.addData("ldb encoder-fwd", ldb.getCurrentPosition() + "  busy=" + ldb.isBusy());
            telemetry.addData("rdf encoder-fwd", rdf.getCurrentPosition() + "  busy=" + rdf.isBusy());
            telemetry.addData("rdb encoder-fwd", rdb.getCurrentPosition() + "  busy=" + rdb.isBusy());
            telemetry.addData("Robot Angle:", + getAngle()+ counterAngle);
            telemetry.addData("RobotX:", robotLocation.getxLocation());
            telemetry.addData("RobotY:", robotLocation.getyLoctation());
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
}
