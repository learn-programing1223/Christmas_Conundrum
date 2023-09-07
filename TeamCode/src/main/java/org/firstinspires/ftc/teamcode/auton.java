package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auton", group="auto")
public class auton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap, this);

        bot.initOpenCV();
        FtcDashboard.getInstance().startCameraStream(bot.webcam, 60);

        waitForStart();

//          strafing in negative directions is going to the right


        //close claw and raise
//        bot.claw();
//        bot.fourBar.setPower(0.8);
//        bot.fourBar.setTargetPosition(Math.round(170));
//
//
//        sleep(100);
//        telemetry.addData("white pixels: ", cameraPipeline.whitePixels);
//        telemetry.update();
//
//        //check white pixels
//        boolean orange = cameraPipeline.whitePixels < 1500;
//
//        //forwards
//        bot.straight(16, 1, .2);
//        sleep(100);
//
//        //release
//        bot.claw();
//
//        sleep(100);
//
//        //back
//        bot.straight(7, -1, .2);
//        sleep(100);
//
//        //lower
//        bot.fourBar.setPower(0.2);
//        bot.fourBar.setTargetPosition(Math.round(0));
//
//        sleep(100);
//
//
//        if (orange) {
//            //orange
//            bot.Strafe(36, 0.4, -1);
//            sleep(100);
//
//        } else {
//            bot.Strafe(36, 0.4, 1);
//            sleep(100);
//
//        }
//
//        bot.turnTo(0, 0.4);
//
//        bot.straight(73, 1, 0.4);
//
//        sleep(100);
//
//        bot.turnTo(0, 0.4);
//        if (orange) {
//            bot.turnTo(-90, 0.4);
//
//        } else {
//            bot.turnTo(-90, 0.4);
//        }

        //pick up the ball from parking if it takes less tahn 5 seconds
        bot.pickupTimer.reset();
            while (bot.dist.getDistance(DistanceUnit.INCH) > 3) {
                bot.straight(34, 1, 0.1);
                if(bot.pickupTimer.seconds()>5){break;}
            }
            bot.claw();
    }
}
