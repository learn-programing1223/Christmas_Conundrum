package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Robot{

    HardwareMap hardwareMap;
    public DcMotor leftBack, leftFront, rightFront, rightBack, fourBar;
    public Servo arm, clawLeft, clawRight;
    public DistanceSensor dist;
    public BNO055IMU imu;
    public OpenCvWebcam webcam;
    public double TickToInches; //Add value later
    public boolean clawClosed = false;
    public boolean armDown = true;

    final static double TICKS_TO_INCH_STRAIGHT = 19.5;
    final static double TICKS_TO_INCH_STRAFE = 30;

    public double kp = 1;
    public double ki = 0;
    public double kd = 0;

    ElapsedTime PIDtimer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");

        fourBar = hardwareMap.get(DcMotorEx.class, "fourBar");
        arm = hardwareMap.get(Servo.class, "arm");
        clawLeft = hardwareMap.get(Servo.class, "leftClaw");
        clawRight = hardwareMap.get(Servo.class, "rightClaw");

        dist = hardwareMap.get(DistanceSensor.class, "dist");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBar.setTargetPosition(0);
        fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fourBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        this.hardwareMap = hardwareMap;

    }

    public void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new cameraPipeline());
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        OpenCvWebcam finalWebcam = webcam;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                finalWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }



    public void straight(int distance,int rev, double power) {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double targetAngle = imu.getAngularOrientation().firstAngle;
        double currentAngle;
        double correction;
        double intSum = 0;
        double derivative;
        double error;
        double prevErr = 0;
        double totalCorrection;
        PIDtimer.reset();
        double cycleTime = PIDtimer.seconds();

        while (Math.abs(leftFront.getCurrentPosition())<=distance * TICKS_TO_INCH_STRAIGHT) {
            currentAngle = imu.getAngularOrientation().firstAngle;
            error = targetAngle - currentAngle;
            correction = kp * error;
            intSum += error * ki * cycleTime;
            derivative = kd * (prevErr - error)/cycleTime;
            prevErr = error;
            totalCorrection = correction+intSum+derivative;

            leftFront.setPower(power * rev - totalCorrection);
            leftBack.setPower(power * rev - totalCorrection);
            rightFront.setPower(power * rev + totalCorrection);
            rightBack.setPower(power * rev + totalCorrection);
            cycleTime = PIDtimer.seconds();
            PIDtimer.reset();
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public double angleWrap360(double angle){
            while(angle<0 || angle>360){
                if(angle<0){angle+=360;}
                if(angle>360){angle-=360;}
            }
        return angle;
    }

    public void turnTo(int angle, double power) {
        double cur_angle = angleWrap360(Math.toDegrees(imu.getAngularOrientation().firstAngle));
        double tar_angle = angleWrap360(angle);
        int dir;
        double e = 360 - cur_angle;
        tar_angle = angleWrap360(tar_angle + e);

        if (tar_angle <= 180) {
            dir = 1;
        } else {
            dir = -1;
        }

        while (Math.abs(Math.toRadians(angle) - imu.getAngularOrientation().firstAngle) > 0.05) {

            leftFront.setPower(power * -1 * dir);
            leftBack.setPower(power * -1 * dir);
            rightFront.setPower(power * dir);
            rightBack.setPower(power * dir);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        //check if it turns right or left for positive vs. negative
    }

    //arm goes from 0.3 to 0.75
    //clawLeft goes from 0.4 to 0.7
    //clawRight goes from 0.1 to 0.4

    public void claw(){
        if(clawClosed) {
            clawRight.setPosition(0.4);
            clawLeft.setPosition(0.4);
        } else {
            clawRight.setPosition(0.06);
            clawLeft.setPosition(0.74);
        }
        clawClosed = !clawClosed;
    }

    public void moveArm(){
        if(armDown) {
            arm.setPosition(0.3);
        } else {
            arm.setPosition(0.75);
        }
        armDown = !armDown;
    }


    public void Strafe(double distance, double power,int dir) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double targetAngle = imu.getAngularOrientation().firstAngle;
        double currentAngle;
        double correction;
        double intSum = 0;
        double derivative;
        double error;
        double prevErr = 0;
        double totalCorrection;
        PIDtimer.reset();
        double cycleTime = PIDtimer.seconds();

        while(Math.abs(leftFront.getCurrentPosition())<=distance * TICKS_TO_INCH_STRAFE){
            currentAngle = imu.getAngularOrientation().firstAngle;
            error = targetAngle - currentAngle;
            correction = kp * error;
            intSum += error * ki * cycleTime;
            derivative = kd * (prevErr - error)/cycleTime;
            prevErr = error;
            totalCorrection = correction+intSum+derivative;


            leftFront.setPower(power*dir*-1 - totalCorrection);
            leftBack.setPower(power*dir - totalCorrection);
            rightFront.setPower(power*dir + totalCorrection);
            rightBack.setPower(power*dir*-1 + totalCorrection);

            cycleTime = PIDtimer.seconds();
            PIDtimer.reset();
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

}

