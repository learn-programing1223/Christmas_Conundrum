package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot{

    public DcMotor leftBack, leftFront, rightFront, rightBack, fourBar;
    public Servo arm, clawLeft, clawRight;
    public BNO055IMU imu;
    public double TickToInches; //Add value later
    public boolean clawClosed = false;
    public boolean armDown = true;

    public Robot(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");

        fourBar = hardwareMap.get(DcMotorEx.class, "fourBar");
        arm = hardwareMap.get(Servo.class, "arm");
        clawLeft = hardwareMap.get(Servo.class, "leftClaw");
        clawRight = hardwareMap.get(Servo.class, "rightClaw");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


//        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fourBar.setDirection(DcMotorSimple.Direction.REVERSE);
//        fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fourBar.setTargetPosition(0);
//        fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }
    public void straight(int distance,int rev, double power) {

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (Math.abs(leftBack.getCurrentPosition()- distance)>0) {

            leftFront.setPower(power * rev);
            leftBack.setPower(power * rev);
            rightFront.setPower(power * rev);
            rightBack.setPower(power * rev);
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

    public void TurnTo(int angle, double power) {
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
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        while(Math.abs(leftBack.getCurrentPosition()-distance)<0){
            leftFront.setPower(power*dir*-1);
            leftBack.setPower(power*dir);
            rightFront.setPower(power*dir);
            rightBack.setPower(power*dir*-1);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}

