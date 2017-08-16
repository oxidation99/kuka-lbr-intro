package me.nicholasnadeau.robot.kukalbr.demo;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import me.nicholasnadeau.robot.kukalbr.utilities.LBRUtilities;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a
 * {@link RoboticsAPITask#run()} method, which will be called successively in
 * the application lifecycle. The application will terminate automatically after
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an
 * exception is thrown during initialization or run.
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the
 * {@link RoboticsAPITask#dispose()} method.</b>
 *
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class DemoRedundancyImpedance extends RoboticsAPIApplication {
	private Controller sunrise;
	private LBR lbr;
	private Tool pointer;

	public void initialize() {
		sunrise = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(sunrise, "LBR_iiwa_7_R800_1");
		pointer = getApplicationData().createFromTemplate("RobotiqPointer");

	}

	public void run() {

		getLogger().info("[+] Loading tool data");
		pointer.attachTo(lbr.getFlange());

		getLogger().info("[+] Moving to forward start");
		lbr.move(LBRUtilities.ptpToHandGuidanceStart
				.setJointVelocityRel(0.25));

		getLogger().info("[+] Moving to redundancy start");
		double offsetAxis2And4 = Math.toRadians(20);
		double offsetAxis4And6 = Math.toRadians(-40);
		double[] loopCenterPosition = new double[] { 0, offsetAxis2And4, 0,
				offsetAxis2And4 + offsetAxis4And6 - Math.toRadians(90), 0,
				offsetAxis4And6, Math.toRadians(90) };
		lbr.move(ptp(loopCenterPosition).setJointVelocityRel(0.25));

		// LBR limit of 5000 N/m trans, 300 Nm/rad rot
		double cartStiffnessTrans = 5000.0;
		double cartStiffnessRot = 300.0;

		CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.TRANSL).setStiffness(
				cartStiffnessTrans);
		impedanceControlMode.parametrize(CartDOF.ROT).setStiffness(
				cartStiffnessRot);
		impedanceControlMode.setNullSpaceStiffness(5.0);

		// The robot is set to position hold and impedance control mode gets
		// activated without a timeout.
		getLogger().info("[+] Hold position in impedance control mode");
		IMotionContainer positionHoldContainer = pointer
				.moveAsync((new PositionHold(impedanceControlMode, -1, null)));

		getApplicationUI().displayModalDialog(
				ApplicationDialogType.INFORMATION,
				"Press ok to finish the application.", "OK");

		// As soon as the modal dialog returns, the motion container will be
		// cancelled. This finishes the position hold.
		positionHoldContainer.cancel();

		getLogger().info("App finished");
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		DemoRedundancyImpedance app = new DemoRedundancyImpedance();
		app.runApplication();
	}
}
