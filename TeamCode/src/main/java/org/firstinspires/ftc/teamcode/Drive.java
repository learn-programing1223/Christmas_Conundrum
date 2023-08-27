package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ASKdrive", group="hi")
public class Drive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        Robot bot = new Robot(hardwareMap, this);

        waitForStart();
        boolean lateA = false;
        boolean lateB = false;



        while(opModeIsActive()) {

            double lx = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double ly = gamepad1.left_stick_y * -1; //multiplied by 1 because left y is opposite(up is -1, down is 1)

            double lf = ly + lx + rx;
            double lb = ly - lx + rx;
            double rf = ly - lx - rx;
            double rb = ly + lx - rx;

            double vector = Math.sqrt((lx*lx) + (ly * ly) + (rx * rx));
            double max = Math.max(Math.max(Math.abs(lf),Math.abs(lb)),Math.max(Math.abs(rf),Math.abs(rb)));
            double ratio = 0;


            if (max == 0) {
                ratio = 0;
            } else {
                ratio = vector / max * .8;
            }
            //arm goes from 0.3 to 0.75
            //clawLeft goes from 0.4 to 0.7
            //clawRight goes from 0.1 to 0.4


            if (gamepad1.a && !lateA){
                bot.claw();
            }
            if (gamepad1.b && !lateB){
                bot.moveArm();
            }

            lateA = gamepad1.a;
            lateB = gamepad1.b;

            if(gamepad1.left_trigger > 0.1){
                bot.fourBar.setPower(0.6 * gamepad1.left_trigger);
            }
            else if(gamepad1.right_trigger > 0.1){
                bot.fourBar.setPower(-0.6 * gamepad1.right_trigger);
            }
            else{
                bot.fourBar.setPower(0);
            }



            bot.leftBack.setPower(lb * ratio);
            bot.leftFront.setPower(lf * ratio);
            bot.rightFront.setPower(rf * ratio);
            bot.rightBack.setPower(rb * ratio);


            telemetry.addData("fourBar: ", bot.fourBar.getCurrentPosition());
            telemetry.update();

        }

    }

}