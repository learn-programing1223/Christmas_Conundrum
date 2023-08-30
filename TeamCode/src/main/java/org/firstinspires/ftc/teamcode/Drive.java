package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="1C - Drive", group="hi")
public class Drive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        Robot bot = new Robot(hardwareMap, this);

        waitForStart();
        boolean lateA = false;
        boolean lateB = false;
        boolean lateDown = false;
        boolean lateUp = false;
        boolean lateLeft = false;
        boolean lateRight = false;
        boolean armLowered =false;
        boolean lateX = false;
        boolean lateY = false;
        boolean reset = false;
        boolean autoPickup = true;
        double slowmode = 0.6;
        float targetPos = 0;

        //increment is how fast the four bar should raise
        double increment = 5;


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

            //a is grab
            if (gamepad1.a && !lateA){
                reset = false;
                bot.claw();
            }
            lateA = gamepad1.a;


            //b is raise
            if (gamepad1.b && !lateB){
                reset = false;
                bot.moveArm();
                armLowered=false;
            }
            lateB = gamepad1.b;

            //x is preset raise the topping to the yellow junction height
            if (gamepad1.x && !lateX){
                //arm lowered is set to 0.3.
                if(!armLowered) {
                    bot.arm.setPosition(0.36);
                } else {
                    bot.arm.setPosition(0.3);
                }
                armLowered = !armLowered;
            }
            lateX = gamepad1.x;


            //y is preset raise the topping to the top of the tree
            if (gamepad1.y && !lateY){
                bot.moveArm();
                armLowered=false;
                bot.fourBar.setPower(0.8);
                targetPos=200;
                bot.fourBar.setTargetPosition(Math.round(targetPos));
                armLowered=false;
            }
            lateY = gamepad1.y;


            //custom fourbar adjustment
            if(gamepad1.left_trigger > 0.1){
                reset = false;
                bot.fourBar.setPower(0.8);

                if(targetPos + (gamepad1.left_trigger*increment) >= 200){
                    targetPos=200;
                }
                else{
                    targetPos += (gamepad1.left_trigger*increment);
                }
                bot.fourBar.setTargetPosition(Math.round(targetPos));
            }
            else if(gamepad1.right_trigger > 0.1){
                reset = false;
                bot.fourBar.setPower(0.2);
                if(targetPos - (gamepad1.right_trigger*increment) <= 0){
                    targetPos=0;
                }
                else{
                    targetPos -= (gamepad1.right_trigger*increment);
                }
                bot.fourBar.setTargetPosition(Math.round(targetPos));
            }

            //dpad:
            //Down is reset
            //Up is star/top
            //Left is mid
            //Right is bottom


            //reset position, claw open
            else if (gamepad1.dpad_down && !lateDown){
                bot.fourBar.setPower(0.2);
                targetPos=0;
                bot.fourBar.setTargetPosition(Math.round(targetPos));

                if(!bot.armDown){
                    bot.moveArm();
                    armLowered=false;
                }
                if(bot.clawClosed){
                    bot.claw();
                }
                reset = true;
            }

            //top position, claw closed
            else if (gamepad1.dpad_up && !lateUp){
                reset = false;
                if(!bot.clawClosed){
                    bot.claw();
                }
                if(bot.armDown){
                    bot.moveArm();
                    armLowered=false;
                }
                bot.fourBar.setPower(0.8);
                targetPos=150;
                bot.fourBar.setTargetPosition(Math.round(targetPos));
            }

            //middle of tree
            else if (gamepad1.dpad_left && !lateLeft){
                reset = false;
                if(!bot.clawClosed){
                    bot.claw();
                }
                if(bot.armDown){
                    bot.moveArm();
                    armLowered=false;
                }
                bot.fourBar.setPower(0.2);
                targetPos=0;
                bot.fourBar.setTargetPosition(Math.round(targetPos));
            }

            //bottom level of tree
            else if (gamepad1.dpad_right && !lateRight){
                reset = false;
                if(!bot.clawClosed){
                    bot.claw();
                }
                if(!bot.armDown){
                    bot.moveArm();
                    armLowered=false;
                }
                bot.fourBar.setPower(0.8);
                targetPos=200;
                bot.fourBar.setTargetPosition(Math.round(targetPos));
            }

            lateDown=gamepad1.dpad_down;
            lateUp=gamepad1.dpad_up;
            lateLeft=gamepad1.dpad_left;
            lateRight=gamepad1.dpad_right;

            //auto pickup feature
            if(reset && autoPickup && !bot.clawClosed){
                if(bot.dist.getDistance(DistanceUnit.INCH) < 3){
                    bot.claw();
                }
            }



            bot.leftBack.setPower(0.8 * lb * ratio * slowmode);
            bot.leftFront.setPower(0.8 * lf * ratio * slowmode);
            bot.rightFront.setPower(0.8 * rf * ratio * slowmode);
            bot.rightBack.setPower(0.8 * rb * ratio * slowmode);


            telemetry.addData("fourBar: ", bot.fourBar.getCurrentPosition());
            telemetry.addData("distance: ", bot.dist.getDistance(DistanceUnit.INCH));
            telemetry.addData("reset? ", reset);
            telemetry.update();

        }

    }

}