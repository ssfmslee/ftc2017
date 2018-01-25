package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
@Autonomous
public class OfficialAutonomousBlue extends LinearOpMode {
    
    //-----------------Robot Controller Camera
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    //====
    
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Motor1 = null;
    private DcMotor Motor2 = null;
    private DcMotor Motor3 = null;
    private DcMotor Motor4 = null;
    private ColorSensor colorSensor;
    private Servo SensorServo =null;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;
    private boolean isScanDone = false;
    
    @Override
    public void runOpMode () {
        
        //---------------Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //South City Robotics 2018 VuforiaLicenseKey 
        //https://developer.vuforia.com/license-manager
        //email darrendotng@gmail.com to obtain username and password to manage license keys
        parameters.vuforiaLicenseKey = "ATbZfEb/////AAAAmQZejrUuP0+4ilV5yBR2nxhz5jMi9FeNdT1dA+wRG/tlwKxFyLVR+WzYN+H7YngjFFE60ojETbbaieEC5Jqa7yJu7qaDsCj6a1biqAafZL6iGrSCE5218seIoIINnYAnWXIGCZD8lSjRPQ0oK6m5cpDdjDF9+ghVmDcBwkDRZk4kRWnYsQIBYlfHDvu6Ct9q6Yso+zQNn2PSrXA2rZg01eDMZz2MS5Fb+Oi9LHKmdG22oXejb6xd4MwxW60m4uxlcDdqcve7wOv9D3oC6y4qY1pjitP1YpQ4tjK0qG285tdMj+1WPIMHJ8/96XOa29CvOjbGXZft71L7rwxy2r+iE3hq/V+0sHCtYB1KOu2pquIx";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();
        //====
        
        boolean bLedOn = true;
        boolean isOpModeActive = true;
        
        //Ititialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
        Motor4 = hardwareMap.get(DcMotor.class, "Motor4");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        SensorServo  = hardwareMap.get(Servo.class, "SensorServo");
        
        Motor1.setDirection(DcMotor.Direction.FORWARD);
        Motor2.setDirection(DcMotor.Direction.FORWARD);
        Motor3.setDirection(DcMotor.Direction.FORWARD);
        Motor4.setDirection(DcMotor.Direction.FORWARD);
        colorSensor.enableLed(bLedOn);
       
        //Wait for the game to start
        waitForStart();
        runtime.reset();
        
        //Gradual movement down so that the sensor doesn't get damaged due to momentum
        SensorServo.setPosition(0.2);
        while ((runtime.seconds() <= 1)&&(opModeIsActive())&&(isOpModeActive == true)) {
        }
        SensorServo.setPosition(0.015);
        while ((runtime.seconds() <= 3)&&(opModeIsActive())&&(isOpModeActive == true)&&(isScanDone == false)) {
            if(colorSensor.red()>0&&colorSensor.blue()==0){ //if red
                //pivot robot 
                while(runtime.seconds() <=1.7){ 
                Motor1.setPower(0.1);
                Motor2.setPower(0.1);//change to move direction rather than pivot
                Motor3.setPower(0.1);
                Motor4.setPower(0.1);
                }
                Motor1.setPower(0);
                Motor2.setPower(0);
                Motor3.setPower(0);
                Motor4.setPower(0);
                
                isScanDone = true;
            }
            else if(colorSensor.blue()>0&&colorSensor.red()==0){
                while(runtime.seconds() <=1.7){
                Motor1.setPower(-0.1);
                Motor2.setPower(-0.1);//change to move direction rather than pivot
                Motor3.setPower(-0.1);
                Motor4.setPower(-0.1);
                }
                Motor1.setPower(0);
                Motor2.setPower(0);
                Motor3.setPower(0);
                Motor4.setPower(0);
            }
        }
        SensorServo.setPosition(1);
        while ((runtime.seconds() <= 30)&&(opModeIsActive())&&(isOpModeActive == true)) {
                
                Motor1.setPower(0);//move towards pictograph
                Motor2.setPower(0);
                Motor3.setPower(0);
                Motor4.setPower(0);
            
                Motor1.setPower(0);//pivot to scan
                Motor2.setPower(0);
                Motor3.setPower(0);
                Motor4.setPower(0);
            
                
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
              //  telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);
                    //this code may not get to run at the beginning of the 
                    //automonous due to all the while loops
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
            if(vuMark == "LEFT"){
                //requires distance sensor
                //drive towards cryptobox 
                //drive along the column separaters
                //scan num of times distance sensor detects a spike(column)
                //num should be 4, robot should stop at 4 
                //inch back slowly so that robot is in the middle of 3 and 4
                //bank gylph in
                //celebrate
            }else if(vuMark == "RIGHT"){
                //num should be 2, robot should stop at 2
            }else if(vuMark == "MIDDLE"){
                //num should be 3, robot should stop at 3
            }

            telemetry.update();
        }
    }
    }
