package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Drive extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot bot = new Robot(hardwareMap, this);

        while(opModeIsActive());

        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double ly = gamepad1.left_stick_y * -1; //multiplied by 1 because left y is opposite(up is -1, down is 1)

        double lf = lx+ly+rx;
        double lb = lx+ly-rx;
        double rf = lx-ly+rx;
        double rb = lx-ly-rx;

        double vector = Math.sqrt(lx+ly * lx+rx * rx*ly);
        double max = Math.max(Math.max(lf,lb),Math.max(rf,rb));
        double ratio = 0;



        if(max == 0){
            ratio = 0;
        }
        else{
            ratio = vector/max*.8;
        }

        //you never actually set the motor powers
    }

}