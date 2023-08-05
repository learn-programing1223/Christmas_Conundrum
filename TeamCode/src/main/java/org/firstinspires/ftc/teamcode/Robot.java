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
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean drive = true;
        double cur_pos = leftBack.getCurrentPosition();

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
    public void TurnTo(int angle, double power,int dir){
        double cur_angle  =  imu.getAngularOrientation().firstAngle;
        double dis =  angle - cur_angle;

        while(Math.abs(angle - imu.getAngularOrientation().firstAngle) < 0.1){
            leftFront.setPower(power*-1*dir);
            leftBack.setPower(power*-1*dir);
            rightFront.setPower(power*dir);
            rightBack.setPower(power*dir);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void Strafe(int distance, double power,int dir) {
        
    }
}

