package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auton", group="auto")
public class auton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap, this);

        bot.initOpenCV();
        FtcDashboard.getInstance().startCameraStream(bot.webcam, 60);

        waitForStart();

        bot.claw();
        bot.fourBar.setPower(0.8);
        bot.fourBar.setTargetPosition(Math.round(200));

        sleep(1000);
        telemetry.addData("white pixels: ", cameraPipeline.whitePixels);
        telemetry.update();

        sleep(2000);

        if(cameraPipeline.whitePixels < 30000) {
            bot.Strafe(24, 0.4, 1);
        }
        else{
            bot.Strafe(36, 0.4, -1);
        }




//        if(cameraPipeline.whitePixels < 30000) {
//            bot.fourBar.setPower(0.4);
//            bot.fourBar.setTargetPosition(180);
//        }
//        else{
//            bot.claw();
//            bot.claw();
//            bot.claw();
//        }


//        bot.claw();
//        bot.moveArm();
//        sleep(1000);
//        bot.initOpenCV();
//        telemetry.addData("white pixels: ", cameraPipeline.whitePixels);
//        telemetry.update();
//        sleep(2000);







//        if (!bot.clawClosed) {
//            bot.claw();
//        }
//        bot.moveArm();
//        bot.straight(4, 1, .2);
//        bot.claw();
//        bot.straight(4, -1, .2);
//        bot.moveArm();
//        if (cameraPipeline.whitePixels < 40000) {
//            //orange
//            bot.turnTo(90, .2);
//            bot.straight(16, 1, 0.4);
//            bot.turnTo(0, .4);
//            bot.straight(16, 1, 0.7);
//        }
//        else {
//            bot.turnTo(-90, .2);
//            bot.straight(16, 1, 0.4);
//            bot.turnTo(0, .4);
//            bot.straight(16, 1, 0.7);
//        }

//        bot.Strafe(24,0.4, 1);
    }
}

