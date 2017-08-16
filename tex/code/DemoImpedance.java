package me.nicholasnadeau.robot.kukalbr.demo;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class DemoImpedance extends RoboticsAPIApplication {

	final static double offsetAxis2And4=Math.toRadians(10);

	private static final int stiffnessZ = 2500;
	private static final int stiffnessY = 700;
	private static final int stiffnessX = 1500;
	private LBR lbr;
	private static double[] startPosition=new double[]{0,offsetAxis2And4,0,offsetAxis2And4-Math.toRadians(90),0,Math.toRadians(90),0};


	public void initialize() {
		lbr = getContext().getDeviceFromType(LBR.class);
	}

	public void run() {
		getLogger().info("Move to start position");
		PTP ptpToStartPosition = ptp(startPosition);
		ptpToStartPosition.setJointVelocityRel(0.25);
		lbr.move(ptpToStartPosition);

		getLogger().info("Hold position in impedance control mode");
		CartesianImpedanceControlMode impedanceControlMode = 	new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);

		// The robot is set to position hold and impedance control mode gets activated without a timeout. 
		IMotionContainer positionHoldContainer = lbr.moveAsync((new PositionHold(impedanceControlMode, -1, null)));

		getLogger().info("Show modal dialog while executing position hold");
		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok to finish the application.", "OK");

		// As soon as the modal dialog returns, the motion container will be cancelled. This finishes the position hold. 
		positionHoldContainer.cancel();
		
		getLogger().info("App finished");
	}
}
