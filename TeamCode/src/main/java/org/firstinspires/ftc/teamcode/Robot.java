package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot{

    public DcMotor leftBack, leftFront, rightFront, rightBack;
    public BNO055IMU imu;
    public double TickToInches; //Add value later

    public Robot(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

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

