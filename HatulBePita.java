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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.ViewerParameters;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HatulBePita
{
    /* Public OpMode members. */
    public DcMotor  motor_left_back = null;
    public DcMotor  motor_right_front = null;
    public DcMotor  motor_right_back = null;
    public DcMotor  motor_left_front = null;
    public Servo    grip    = null;
    public TouchSensor sensor_button=null;
    public ColorSensor sensor_color=null;
    public DistanceSensor sensor_distance=null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HatulBePita(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motor_left_back  = hwMap.get(DcMotor.class, "motor_left_back");
        motor_right_front = hwMap.get(DcMotor.class, "motor_right_front");
        motor_right_back  = hwMap.get(DcMotor.class, "motor_right_back");
        motor_left_front = hwMap.get(DcMotor.class, "motor_left_front");

        motor_left_back.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motor_right_front.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motor_right_back.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motor_left_front.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        motor_left_back.setPower(0);
        motor_right_front.setPower(0);
        motor_right_back.setPower(0);
        motor_left_front.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motor_left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        grip = hwMap.get(Servo.class, "left_hand");

        grip.setPosition(MID_SERVO);






        sensor_button = hwMap.get(TouchSensor.class, "B");
        sensor_color =hwMap.get(ColorSensor.class, "C");
        sensor_distance=hwMap.get(DistanceSensor.class, "D");
    }
 }

