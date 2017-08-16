package me.nicholasnadeau.robot.kukalbr.demo;

import me.nicholasnadeau.robot.kukalbr.utilities.LBRUtilities;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.LIN;

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
public class DemoForceContactRetract extends RoboticsAPIApplication {
	private Controller sunriseController;
	private LBR lbr;
	private Tool pointer;

	public void initialize() {
		sunriseController = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(sunriseController, "LBR_iiwa_7_R800_1");
		pointer = getApplicationData().createFromTemplate("RobotiqPointer");
	}

	public void run() {
		// load tool
		pointer.attachTo(lbr.getFlange());
		getLogger().info("Pointer Tool Data Loaded");

		// move to forward starting pose
		getLogger().info("Moving to forward start position");
		lbr.move(LBRUtilities.ptpToHandGuidanceStart
				.setJointVelocityRel(LBRUtilities.SAFE_REL_JOINT_VELOCITY));

		// get start and end frame
		Frame startFrame = lbr.getCurrentCartesianPosition(pointer
				.getDefaultMotionFrame());
		Frame endFrame = startFrame.copy();
		endFrame.setZ(0);

		// create force condition
		double maxForce = 10;
		ForceCondition forceDetected = ForceCondition
				.createSpatialForceCondition(pointer.getDefaultMotionFrame(),
						maxForce);

		// create motions
		LIN upMotion = lin(startFrame).setCartVelocity(
				LBRUtilities.SAFE_ABS_LIN_VELOCITY);
		LIN downMotion = lin(endFrame).setCartVelocity(
				LBRUtilities.SAFE_ABS_LIN_VELOCITY).breakWhen(forceDetected);

		// execute motion loop
		for (int i = 0; i < 10; i++) {
			pointer.move(downMotion);
			pointer.move(upMotion);
		}

		getLogger().info("App finished");
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		DemoForceContactRetract app = new DemoForceContactRetract();
		app.runApplication();
	}
}
