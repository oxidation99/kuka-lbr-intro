package me.nicholasnadeau.robot.kukalbr.demo;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import me.nicholasnadeau.robot.kukalbr.utilities.LBRUtilities;

import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPITask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

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
public class DemoPegInHole extends RoboticsAPIApplication {
	private Controller sunriseController;
	private LBR lbr;
	private Tool pointer;
	static final double MAX_CART_VEL = 50;
	static final double MAX_JOINT_REL_VEL = 0.25;
	static final double FORCE_CONTACT = 10;
	static final double FREQUENCY = 0.25;
	static final double AMPLITUDE = 70;
	static final double SEC_TO_NANO = 1e-9;
	static final double POINTER_DIAMETER = 30;

	@Override
	public void initialize() {
		sunriseController = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(sunriseController, "LBR_iiwa_7_R800_1");
		pointer = getApplicationData().createFromTemplate("RobotiqPointer");
	}

	@Override
	public void run() {
		// load tool
		pointer.attachTo(lbr.getFlange());
		getLogger().info("Pointer Tool Data Loaded");

		// move to forward starting pose
		getLogger().info("Moving to forward start position");
		lbr.move(LBRUtilities.ptpToHandGuidanceStart
				.setJointVelocityRel(MAX_JOINT_REL_VEL));

		// get start and end frame
		Frame startFrame = lbr.getCurrentCartesianPosition(pointer
				.getDefaultMotionFrame());
		Frame endFrame = startFrame.copy();
		endFrame.setZ(0);

		// move pointer to start
		pointer.move(lin(endFrame).setCartVelocity(MAX_CART_VEL));

		// set up impedance control
		final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
		cartImp.parametrize(CartDOF.X).setStiffness(500);
		cartImp.parametrize(CartDOF.Y).setStiffness(500);
		cartImp.parametrize(CartDOF.Z).setStiffness(500);

		// servo move
		AbstractFrame initialPosition = lbr.getCurrentCartesianPosition(pointer
				.getDefaultMotionFrame());
		SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(initialPosition);
		aSmartServoLINMotion.setMaxTranslationVelocity(new double[] { 100, 100,
				100 });
		aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);
		IMotionContainer servoMove = pointer.moveAsync(aSmartServoLINMotion
				.setMode(cartImp));
		ISmartServoLINRuntime smartServoLINRuntime = aSmartServoLINMotion
				.getRuntime();

		Frame aFrame = smartServoLINRuntime
				.getCurrentCartesianDestination(pointer.getDefaultMotionFrame());

		long startTimeStamp = System.nanoTime();
		double curTime;
		Vector externalForces;

		// search for hole outer
		getLogger().info("Searching for hole");
		while (true) {
			// Update the smart servo LIN runtime
			smartServoLINRuntime.updateWithRealtimeSystem();

			externalForces = smartServoLINRuntime.getExtForceVector().withZ(0);
			double magnitude = Math.sqrt(externalForces
					.dotProduct(externalForces));
			if (magnitude >= FORCE_CONTACT) {
				break;
			}

			// Compute and set the sine function
			curTime = System.nanoTime() - startTimeStamp;
			double omega = FREQUENCY * 2 * Math.PI * SEC_TO_NANO;
			double sinArgument = omega * curTime;
			Frame destFrame = new Frame(aFrame);
			destFrame.setX(AMPLITUDE * Math.sin(sinArgument));
			destFrame.setY(-MAX_CART_VEL * SEC_TO_NANO * curTime);

			// Set new destination
			smartServoLINRuntime.setDestination(destFrame);
		}

		// lift until no force
		getLogger().info("Hole found, lifting.");
		aFrame = smartServoLINRuntime.getCurrentCartesianDestination(pointer
				.getDefaultMotionFrame());
		startTimeStamp = System.nanoTime();
		double z = 0;
		while (true) {
			// Update the smart servo LIN runtime
			smartServoLINRuntime.updateWithRealtimeSystem();

			externalForces = smartServoLINRuntime.getExtForceVector().withZ(0);
			double magnitude = Math.sqrt(externalForces
					.dotProduct(externalForces));
			if (magnitude < FORCE_CONTACT / 2 && Math.abs(z) > 30) {
				break;
			}

			// Move up
			curTime = System.nanoTime() - startTimeStamp;
			Frame destFrame = new Frame(aFrame);
			Vector direction = externalForces.normalize();
			direction = direction.multiply(-POINTER_DIAMETER);
			destFrame.setX(direction.getX());
			destFrame.setY(direction.getY());
			z = -MAX_CART_VEL * SEC_TO_NANO * curTime;
			destFrame.setZ(z);

			// Set new destination
			smartServoLINRuntime.setDestination(destFrame);
		}

		// move in old force direction (or opposite?), 1x diameter distance
		getLogger().info("Move to centre of hole.");
		Vector direction = externalForces.normalize();
		direction = direction.multiply(-POINTER_DIAMETER * 0.8);
		aFrame = smartServoLINRuntime.getCurrentCartesianDestination(pointer
				.getDefaultMotionFrame());
		Frame destFrame = new Frame(aFrame);
		destFrame.setX(direction.getX());
		destFrame.setY(direction.getY());
		smartServoLINRuntime.setDestination(destFrame);
		while (!smartServoLINRuntime.isDestinationReached()) {

		}

		// move down into hole
		getLogger().info("Descend into hole.");
		aFrame = smartServoLINRuntime.getCurrentCartesianDestination(pointer
				.getDefaultMotionFrame());
		startTimeStamp = System.nanoTime();
		z = 0;
		while (true) {
			// Update the smart servo LIN runtime
			smartServoLINRuntime.updateWithRealtimeSystem();

			externalForces = smartServoLINRuntime.getExtForceVector();
			if (Math.abs(externalForces.getZ()) >= FORCE_CONTACT
					&& Math.abs(z) > 20) {
				break;
			}

			// Move down
			curTime = System.nanoTime() - startTimeStamp;
			destFrame = new Frame(aFrame);
			z = MAX_CART_VEL * SEC_TO_NANO * curTime;
			destFrame.setZ(z);
			smartServoLINRuntime.setDestination(destFrame);
		}

		// complete, move straight up and back to home
		getLogger().info("Peg in hole complete, moving home.");
		servoMove.cancel();
		pointer.move(linRel(0, 0, -100).setCartVelocity(MAX_CART_VEL));
		lbr.move(LBRUtilities.ptpToHandGuidanceStart
				.setJointVelocityRel(MAX_JOINT_REL_VEL));

		getLogger().info("App finished");
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		DemoPegInHole app = new DemoPegInHole();
		app.runApplication();
	}
}
