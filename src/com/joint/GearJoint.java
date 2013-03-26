
package com.joint;

import org.andengine.engine.camera.Camera;
import org.andengine.engine.options.EngineOptions;
import org.andengine.engine.options.ScreenOrientation;
import org.andengine.engine.options.resolutionpolicy.RatioResolutionPolicy;
import org.andengine.entity.primitive.Rectangle;
import org.andengine.entity.scene.Scene;
import org.andengine.extension.physics.box2d.PhysicsConnector;
import org.andengine.extension.physics.box2d.PhysicsFactory;
import org.andengine.extension.physics.box2d.PhysicsWorld;
import org.andengine.extension.physics.box2d.util.constants.PhysicsConstants;
import org.andengine.ui.activity.SimpleBaseGameActivity;
import org.andengine.util.color.Color;
import org.andengine.util.math.MathUtils;

import android.hardware.SensorManager;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.BodyDef.BodyType;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.Joint;
import com.badlogic.gdx.physics.box2d.joints.GearJointDef;
import com.badlogic.gdx.physics.box2d.joints.PrismaticJointDef;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJointDef;

/**
 * This class demonstrates a gear joint.
 * There is a large blue square and a small yellow square.
 * The rotation of the blue square causes the yellow square to move.
 *
 */
public class GearJoint extends SimpleBaseGameActivity {

	private final int CAMERA_WIDTH = 800; // right edge of screen
	private final int CAMERA_HEIGHT = 480; // bottom of screen

	private Scene mScene;

	private PhysicsWorld mPhysicsWorld;

	private Body yellowRectangleBody;
	private Body blueRectangleBody;

	private Joint prismaticJoint;
	private Joint revoluteJoint;

	private Vector2 mWorldAxis;
	private Body mGroundBody;

	public EngineOptions onCreateEngineOptions() {

		final Camera camera = new Camera(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT);

		final EngineOptions engineOptions = new EngineOptions(true, ScreenOrientation.LANDSCAPE_FIXED, new RatioResolutionPolicy(CAMERA_WIDTH, CAMERA_HEIGHT), camera);
		return engineOptions;

	}

	@Override
	protected void onCreateResources() {

	}

	@Override
	public Scene onCreateScene() {

		mScene = new Scene();

		this.mPhysicsWorld = new PhysicsWorld(new Vector2(0, SensorManager.GRAVITY_EARTH), false);
		this.mGroundBody = this.mPhysicsWorld.createBody(new BodyDef());

		FixtureDef fixtureDef = PhysicsFactory.createFixtureDef(1.0f, 0.0f, 1.0f);

		Rectangle blueRectangle = new Rectangle(200, 200, 150, 150, this.getVertexBufferObjectManager());
		blueRectangle.setColor(Color.BLUE);
		blueRectangleBody = PhysicsFactory.createBoxBody(this.mPhysicsWorld, blueRectangle, BodyType.DynamicBody, fixtureDef);
		blueRectangle.setUserData(blueRectangleBody);
		this.mPhysicsWorld.registerPhysicsConnector(new PhysicsConnector(blueRectangle, blueRectangleBody, true, true));

		Rectangle yellowRectangle = new Rectangle(550, 200, 30, 30, this.getVertexBufferObjectManager());
		yellowRectangle.setColor(Color.YELLOW);
		yellowRectangleBody = PhysicsFactory.createBoxBody(this.mPhysicsWorld, yellowRectangle, BodyType.DynamicBody, fixtureDef);
		yellowRectangle.setUserData(yellowRectangleBody);
		this.mPhysicsWorld.registerPhysicsConnector(new PhysicsConnector(yellowRectangle, yellowRectangleBody, true, true));

		createRevoluteJoint();

		mWorldAxis = new Vector2(1.0f, 0.0f); // constrain movement to x direction

		createPrismaticJoint();		

		setGearJoint();

		mScene.attachChild(blueRectangle);
		mScene.attachChild(yellowRectangle);

		this.mScene.registerUpdateHandler(this.mPhysicsWorld);

		return mScene;

	}

	private void createPrismaticJoint() {

		PrismaticJointDef prismaticJointDef = new PrismaticJointDef();
		prismaticJointDef.initialize(mGroundBody, yellowRectangleBody, yellowRectangleBody.getWorldCenter(), mWorldAxis);  //ground must be the first body
		prismaticJointDef.lowerTranslation = -15.0f; // -x, in this case
		prismaticJointDef.upperTranslation = 5.0f;
		prismaticJointDef.enableLimit = true;
		prismaticJointDef.maxMotorForce = 1.0f;
		prismaticJointDef.motorSpeed = 0.0f;
		prismaticJointDef.enableMotor = true;

		prismaticJoint = this.mPhysicsWorld.createJoint(prismaticJointDef);

	}

	private void createRevoluteJoint() {

		RevoluteJointDef revoluteJointDef = new RevoluteJointDef();
		revoluteJointDef.initialize(mGroundBody, blueRectangleBody, blueRectangleBody.getWorldCenter());  //ground must be the first body
		//revoluteJointDef.lowerAngle = (float) (MathUtils.degToRad(360));
		//revoluteJointDef.upperAngle = (float) (MathUtils.degToRad(360)); 
		//revoluteJointDef.enableLimit = false; //true;
		revoluteJointDef.localAnchorA.set(0,0);
		revoluteJointDef.localAnchorB.set(0,0);

		revoluteJointDef.initialize(mGroundBody, blueRectangleBody, blueRectangleBody.getWorldCenter());
		
		revoluteJointDef.enableMotor = true;
		revoluteJointDef.maxMotorTorque = 20;
		revoluteJointDef.motorSpeed = MathUtils.degToRad(360); //1 turn per second counter-clockwise
		revoluteJoint = this.mPhysicsWorld.createJoint(revoluteJointDef);

	}

	private void setGearJoint() {

		GearJointDef gearJointDef = new GearJointDef();

		gearJointDef.joint1 = revoluteJoint;
		gearJointDef.joint2 = prismaticJoint;
		gearJointDef.bodyA = blueRectangleBody;
		gearJointDef.bodyB = yellowRectangleBody;
		gearJointDef.ratio = (float) (2.0 * Math.PI / (300 / PhysicsConstants.PIXEL_TO_METER_RATIO_DEFAULT));

		this.mPhysicsWorld.createJoint(gearJointDef);

	}

}