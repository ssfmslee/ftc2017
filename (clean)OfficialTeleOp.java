package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name="OfficialTeleOp", group="Linear Opmode")

public class OfficialTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Servo leftGrip      = null;
    private Servo rightGrip     = null;
    private DcMotor Extend      = null;
    private DcMotor Chain       = null;
    private DcMotor Motor1      = null;
    private DcMotor Motor2      = null;
    private DcMotor Motor3      = null;
    private DcMotor Motor4      = null;
    private double left_pos     = 0.5;
    private double right_pos    = 0.5;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;
    private double ExtendPower;
    private double ChainPower;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    
    //Hardware Map    
    leftGrip  = hardwareMap.get(Servo.class, "left_servo");
    rightGrip = hardwareMap.get(Servo.class, "right_servo");
    Extend    = hardwareMap.get(DcMotor.class, "MotorExtend");
    Chain     = hardwareMap.get(DcMotor.class, "MotorLift");
    Motor1    = hardwareMap.get(DcMotor.class, "Motor1");
    Motor2    = hardwareMap.get(DcMotor.class, "Motor2");
    Motor3    = hardwareMap.get(DcMotor.class, "Motor3");
    Motor4    = hardwareMap.get(DcMotor.class, "Motor4");
    
    //Setup
    leftGrip.scaleRange(MIN_POS,MAX_POS);
    rightGrip.scaleRange(MIN_POS,MAX_POS);
    leftGrip.setPosition(left_pos);
    rightGrip.setPosition(right_pos);
    Extend.setDirection(DcMotor.Direction.FORWARD);
    Chain.setDirection(DcMotor.Direction.FORWARD);
    Motor1.setDirection(DcMotor.Direction.FORWARD);
    Motor2.setDirection(DcMotor.Direction.FORWARD);
    Motor3.setDirection(DcMotor.Direction.FORWARD);
    Motor4.setDirection(DcMotor.Direction.FORWARD);
        
        // Wait for start
        waitForStart();
        runtime.reset();
        
        //Main
        while (opModeIsActive()) {
        
        //Grabber
        if (gamepad2.dpad_down){
            left_pos +=0.05;
            right_pos -=0.05;
            }
        if (gamepad2.dpad_up){
            left_pos -=0.05;
            right_pos +=0.05;
            }
        if (right_pos >=1.0){
            right_pos =0.99;
            }   
        if (left_pos <=0){
            left_pos =0.01;
            }
        if (left_pos >=1.0){
            left_pos =0.99;
            }   
        if (right_pos <=0){
            right_pos =0.01;
            }
        leftGrip.setPosition(left_pos);
        rightGrip.setPosition(right_pos);
            
        //Chain and Extend
        ExtendPower  = gamepad2.left_stick_y ;
        if(gamepad2.right_trigger == 0)
        Chain.setPower(-gamepad2.left_trigger);
        
        if(gamepad2.left_trigger == 0)
        Chain.setPower(gamepad2.right_trigger);
    
        Extend.setPower(ExtendPower/3);
        Chain.setPower(ChainPower/3);
        
        //Drive/Pivot
        double pivotR;
        double verticalDrvF;
        double verticalDrvR;
        double verticalDrv;
        double horizontalDrv;
    
        pivotR  = -gamepad1.left_stick_x ;
        verticalDrvF = gamepad1.right_trigger;
        verticalDrvR = gamepad1.left_trigger;
       
        //x' = xcos(angle)-ysin(angle)
        horizontalDrv = (gamepad1.right_stick_x)*Math.cos(Math.toRadians(-45))-(-gamepad1.right_stick_y)*(Math.sin(Math.toRadians(-45)));
        //y' = ycos(angle)+xsin(angle)
        verticalDrv = (-gamepad1.right_stick_y)*Math.cos(Math.toRadians(-45))+(gamepad1.right_stick_x)*(Math.sin(Math.toRadians(-45)));
       
        if((gamepad1.right_trigger == 0.0) && (gamepad1.left_trigger == 0.0) && (gamepad1.right_stick_y == 0.0) && (gamepad1.right_stick_x == 0.0)){
        Motor1.setPower(pivotR/2);
        Motor2.setPower(pivotR/2);
        Motor3.setPower(pivotR/2);
        Motor4.setPower(pivotR/2);
        }
        
        if((gamepad1.left_stick_x == 0.0)&&(gamepad1.left_trigger == 0.0)&&(gamepad1.right_trigger == 0.0)){
        Motor2.setPower(verticalDrv);
        Motor4.setPower(-verticalDrv);
        Motor1.setPower(horizontalDrv);
        Motor3.setPower(-horizontalDrv); 
        }
        //End of Main
        
        }
    }
}
