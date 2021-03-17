/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="VtechAutoOp", group="Linear Opmode")
//Disabled
public class Forward extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        VikingTechRobot robot = new VikingTechRobot(hardwareMap);
        VikingTechOps autoOps = new VikingTechOps();
        autoOps.driveFront(robot,0.5d);
        //run for 2 seconds
        sleep(2000);
        autoOps.driveFront(robot,0.1d);
        sleep(1000);
        Stop(robot);
        autoOps.driveBack(robot,.1d);
        autoOps.driveBack(robot,.5d);
        //run for 2 seconds
        sleep(2000);
        Stop(robot);
        autoOps.turn(robot,.3d, 90);
        autoOps.turn(robot,.3d, -90);
        autoOps.turn(robot,.3d, 45);
        autoOps.turn(robot, 0.3,  45);
        //run for .875 seconds
      //  sleep(2350);
         Stop(robot);

        telemetry.update();


    }

    //Override
    public void driveFront(VikingTechRobot robot, double power) {


        //rightMotor = hardwareMap.dcMotor.get("RMF");
        try {
            robot.rightMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);

            robot.leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

            robot.leftMotorFront.setPower(power);
            robot.rightMotorFront.setPower(power);
            robot.leftMotorBack.setPower(power);
            robot.rightMotorBack.setPower(power);


        } catch (Exception e) {
            e.printStackTrace();

        }
    }
    public void driveBack(VikingTechRobot robot, double power)  {

        try {

            //rightMotor = hardwareMap.dcMotor.get("RMF");
            robot.rightMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

            robot.leftMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);

            robot.leftMotorFront.setPower(power);
            robot.rightMotorFront.setPower(power);
            robot.leftMotorBack.setPower(power);
            robot.rightMotorBack.setPower(power);
        }catch (Exception e)
        {
            e.printStackTrace();
        }


    }

    public void Stop(VikingTechRobot robot) {
        double power;
        power = 0.0;
        robot.leftMotorFront.setPower(power);
        robot.rightMotorFront.setPower(power);
        robot.leftMotorBack.setPower(power);
        robot.rightMotorBack.setPower(power);
    }



}

