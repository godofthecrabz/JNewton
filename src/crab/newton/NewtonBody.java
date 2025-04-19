package crab.newton;


import crab.newton.callbacks.NewtonApplyForceAndTorque;
import crab.newton.callbacks.NewtonBodyDestructor;
import crab.newton.callbacks.NewtonSetTransform;
import crab.newton.internal.*;
import java.lang.foreign.*;

public record NewtonBody(MemorySegment address, int bodyType) {

	public static final int DYNAMIC_BODY = Newton.NEWTON_DYNAMIC_BODY(),
			KINEMATIC_BODY = Newton.NEWTON_KINEMATIC_BODY(),
			DYNAMIC_ASYMETRIC_BODY = Newton.NEWTON_DYNAMIC_ASYMETRIC_BODY();

	/**
	 * public constructor of the {@code NewtonBody} class.
	 * This constructor is meant to be used in
	 * callback methods such as in {@code NewtonApplyForceAndTorque} when the NewtonBody is a raw address.
	 * Creation of NewtonBodies should only happen with the {@code NewtonWorld} body create methods.
	 * Supplying an address that isn't an already existing NewtonBody instance may result in a crash or
	 * undefined behavior.
	 * @param address address of the NewtonBody
	 */
	public NewtonBody(MemorySegment address) {
		this(address, Newton.NewtonBodyGetType(address));
	}

	public NewtonBody {}

	/**
	 * Test if object is equal to this NewtonBody.
	 * @param object
	 * @return true if object is an instance of NewtonBody with the same address and bodyType.
	 */
	@Override
	public boolean equals(Object object) {
		return object instanceof NewtonBody body &&
				this.address.equals(body.address) &&
				this.bodyType == body.bodyType;
	}
	
	/**
	 * Gets the simulation state of the body
	 * @return 0 if body is disabled or 1 if body is active
	 */
	public int getSimulationState() {
		return Newton.NewtonBodyGetSimulationState(address);
	}
	
	/**
	 * Sets the simulation state of the body
	 * @param state 0 to disable body | 1 to activate body
	 */
	public void setSimulationState(int state) {
		Newton.NewtonBodySetSimulationState(address, state);
	}

	/**
	 * Checks if the NewtonBody is collidable
	 * @return 1 if true | 0 if false
	 */
	public int getCollidable() {
		return Newton.NewtonBodyGetCollidable(address);
	}

	/**
	 * Sets the collidable state of the NewtonBody
	 * @param collidableState 1 for true | 0 for false
	 */
	public void setCollidable(int collidableState) {
		Newton.NewtonBodySetCollidable(address, collidableState);
	}

	/**
	 * Adds the net force applied to the NewtonBody.
	 * The MemorySegment holding the force vector is expected to have an equivalent layout
	 * to the {@code Newton.VEC3F} MemoryLayout.
	 * This method is only effective when called from {@code NewtonApplyForceAndTorque} callback.
	 * @param force {@code MemorySegment} containing the force vector
	 */
	public void addForce(MemorySegment force) {
		Newton.NewtonBodyAddForce(address, force);
	}

	/**
	 * Adds the net force applied to the NewtonBody.
	 * The float array must have a length greater than or equal to 3.
	 * This method is only effective when called from {@code NewtonApplyForceAndTorque} callback.
	 * @param force {@code float[]} containing the force vector
	 */
	public void addForce(float[] force) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment forceSeg = arena.allocateArray(Newton.C_FLOAT, force);
			Newton.NewtonBodyAddForce(address, forceSeg);
		}
	}

	/**
	 * Adds the net torque applied to the NewtonBody.
	 * The MemorySegment holding the torque vector is expected to have an equivalent layout
	 * to the {@code Newton.VEC3F} MemoryLayout.
	 * This method is only effective when called from {@code NewtonApplyForceAndTorque} callback.
	 * @param torque {@code MemorySegment} containing the torque vector
	 */
	public void addTorque(MemorySegment torque) {
		Newton.NewtonBodyAddTorque(address, torque);
	}

	/**
	 * Adds the net torque applied to the NewtonBody.
	 * The float array must have a length greater than or equal to 3.
	 * This method is only effective when called from {@code NewtonApplyForceAndTorque} callback.
	 * @param torque {@code float[]} containing the torque vector
	 */
	public void addTorque(float[] torque) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment torqueSeg = arena.allocateArray(Newton.C_FLOAT, torque);
			Newton.NewtonBodyAddTorque(address, torqueSeg);
		}
	}

	/**
	 * Sets the relative position of the center of mass.
	 * The MemorySegment holding the center of mass is expected to have an equivalent layout
	 * to the {@code Newton.VEC3F} MemoryLayout.
	 * @param center {@code MemorySegment} containing the center of mass
	 */
	public void setCenterOfMass(MemorySegment center) {
		Newton.NewtonBodySetCentreOfMass(address, center);
	}

	/**
	 * Sets the relative position of the center of mass.
	 * The float array must have a length greater than or equal to 3.
	 * @param center {@code float[]} containing the center of mass
	 */
	public void setCenterOfMass(float[] center) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment centerSeg = arena.allocateArray(Newton.C_FLOAT, center);
			Newton.NewtonBodySetCentreOfMass(address, centerSeg);
		}
	}

	/**
	 * Sets the mass matrix of the NewtonBody
	 * @param mass the mass of the NewtonBody
	 * @param Ixx moment of inertia of the first principal axis
	 * @param Iyy moment of inertia of the second principal axis
	 * @param Izz moment of inertia of the third principal axis
	 */
	public void setMassMatrix(float mass, float Ixx, float Iyy, float Izz) {
		Newton.NewtonBodySetMassMatrix(address, mass, Ixx, Iyy, Izz);
	}

	/**
	 * Sets the mass matrix of the NewtonBody. The MemorySegment containing the matrix should have an
	 * equivalent layout to the {@code Newton.MAT4F} MemoryLayout. The matrix should be in column-major order.
	 * @param mass the mass of the NewtonBody
	 * @param inertiaMatrix {@code MemorySegment} containing the inertia matrix
	 */
	public void setFullMassMatrix(float mass, MemorySegment inertiaMatrix) {
		Newton.NewtonBodySetFullMassMatrix(address, mass, inertiaMatrix);
	}

	/**
	 * Sets the mass matrix of the NewtonBody.
	 * The float array must have a length greater than or equal to 16.
	 * The matrix should be in column-major order.
	 * @param mass the mass of the NewtonBody
	 * @param inertiaMatrix {@code float[]} containing the inertia matrix
	 */
	public void setFullMassMatrix(float mass, float[] inertiaMatrix) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, inertiaMatrix);
			Newton.NewtonBodySetFullMassMatrix(address, mass, matrix);
		}
	}

	/**
	 * Sets the mass property of the NewtonBody based of the NewtonCollision and mass.
	 * @param mass the mass of the NewtonBody
	 * @param collision {@code NewtonCollision}
	 */
	public void setMassProperties(float mass, NewtonCollision collision) {
		Newton.NewtonBodySetMassProperties(address, mass, collision.address());
	}

	/**
	 * Sets the transformation matrix of the NewtonBody.
	 * The MemorySegment containing the matrix should have an
	 * equivalent layout to the {@code Newton.MAT4F} MemoryLayout.
	 * The matrix should be in column-major order.
	 * @param matrix {@code MemorySegment} containing the transformation matrix
	 */
	public void setMatrix(MemorySegment matrix) {
		Newton.NewtonBodySetMatrix(address, matrix);
	}

	/**
	 * Sets the transformation matrix of the NewtonBody.
	 * The float array must have a length greater than or equal to 16.
	 * The matrix should be in column-major order.
	 * @param matrix {@code float[]} containing the transformation matrix
	 */
	public void setMatrix(float[] matrix) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocateArray(Newton.C_FLOAT, matrix);
			Newton.NewtonBodySetMatrix(address, matrixSeg);
		}
	}

	/**
	 * Sets the transformation matrix of the NewtonBody.
	 * The MemorySegment containing the matrix should have an
	 * equivalent layout to the {@code Newton.MAT4F} MemoryLayout.
	 * The matrix should be in column-major order.
	 * @param matrix {@code MemorySegment} containing the transformation matrix
	 */
	public void setMatrixNoSleep(MemorySegment matrix) {
		Newton.NewtonBodySetMatrixNoSleep(address, matrix);
	}

	/**
	 * Sets the transformation matrix of the NewtonBody.
	 * The float array must have a length greater than or equal to 16.
	 * The matrix should be in column-major order.
	 * @param matrix {@code float[]} containing the transformation matrix
	 */
	public void setMatrixNoSleep(float[] matrix) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocateArray(Newton.C_FLOAT, matrix);
			Newton.NewtonBodySetMatrixNoSleep(address, matrixSeg);
		}
	}

	/**
	 * Applies hierarchical transformation to the NewtonBody.
	 * The MemorySegment containing the matrix should have an
	 * equivalent layout to the {@code Newton.MAT4F} MemoryLayout.
	 * The matrix should be in column-major order.
	 * @param matrix {@code MemorySegment} containing the transformation matrix
	 */
	public void setMatrixRecursive(MemorySegment matrix) {
		Newton.NewtonBodySetMatrixRecursive(address, matrix);
	}

	/**
	 * Applies hierarchical transformation to the NewtonBody.
	 * The float array must have a length greater than or equal to 16.
	 * The matrix should be in column-major order.
	 * @param matrix {@code float[]} containing the transformation matrix
	 */
	public void setMatrixRecursive(float[] matrix) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocateArray(Newton.C_FLOAT, matrix);
			Newton.NewtonBodySetMatrixRecursive(address, matrixSeg);
		}
	}

	/**
	 * Assigns a material group id to the NewtonBody
	 * @param id material group id
	 */
	public void setMaterialGroupdID(int id) {
		Newton.NewtonBodySetMaterialGroupID(address, id);
	}

	/**
	 * Sets the continuous collision mode for the NewtonBody. The continuous collision mode for
	 * the NewtonBody is set to off by default.
	 * If continuous collision is activated the physics engine will perform predictive calculations
	 * to prevent high speed bodies from clipping. This is recommended to be activated for small, high speed bodies.
	 * @param state 1 for active | 0 for disabled
	 */
	public void setContinuousCollisionMode(int state) {
		Newton.NewtonBodySetContinuousCollisionMode(address, state);
	}

	/**
	 * Sets the collision state flag of this NewtonBody when the body is connected to another body
	 * by a hierarchy of joints.
	 * @param state 1 this body will collide with any linked body | 0 disable collisions with body connected to
	 *              this one by joints
	 */
	public void setJointRecursiveCollision(int state) {
		Newton.NewtonBodySetJointRecursiveCollision(address, state);
	}

	/**
	 * Sets the angular velocity of the NewtonBody.
	 * The MemorySegment holding the angular velocity is expected to have an equivalent layout
	 * to the {@code Newton.VEC3F} MemoryLayout.
	 * @param omega {@code MemorySegment} containing the angular velocity
	 */
	public void setOmega(MemorySegment omega) {
		Newton.NewtonBodySetOmega(address, omega);
	}

	/**
	 * Sets the angular velocity of the NewtonBody.
	 * The float array must have a length greater than or equal to 3.
	 * @param omega {@code float[]} containing the angular velocity
	 */
	public void setOmega(float[] omega) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment omegaSeg = arena.allocateArray(Newton.C_FLOAT, omega);
			Newton.NewtonBodySetOmega(address, omegaSeg);
		}
	}

	/**
	 * Sets the angular velocity of the NewtonBody.
	 * The MemorySegment holding the angular velocity is expected to have an equivalent layout
	 * to the {@code Newton.VEC3F} MemoryLayout.
	 * @param omega {@code MemorySegment} containing the angular velocity
	 */
	public void setOmegaNoSleep(MemorySegment omega) {
		Newton.NewtonBodySetOmegaNoSleep(address, omega);
	}

	/**
	 * Sets the angular velocity of the NewtonBody.
	 * The float array must have a length greater than or equal to 3.
	 * @param omega {@code float[]} containing the angular velocity
	 */
	public void setOmegaNoSleep(float[] omega) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment omegaSeg = arena.allocateArray(Newton.C_FLOAT, omega);
			Newton.NewtonBodySetOmegaNoSleep(address, omegaSeg);
		}
	}

	/**
	 * Sets the global linear velocity of the NewtonBody.
	 * The MemorySegment holding the linear velocity is expected to have an equivalent layout
	 * to the {@code Newton.VEC3F} MemoryLayout.
	 * @param velocity {@code MemorySegment} containing the linear velocity
	 */
	public void setVelocity(MemorySegment velocity) {
		Newton.NewtonBodySetVelocity(address, velocity);
	}

	/**
	 * Sets the global linear velocity of the NewtonBody.
	 * The float array must have a length greater than or equal to 3.
	 * @param velocity {@code float[]} containing the linear velocity
	 */
	public void setVelocity(float[] velocity) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment velSeg = arena.allocateArray(Newton.C_FLOAT, velocity);
			Newton.NewtonBodySetVelocity(address, velSeg);
		}
	}

	/**
	 * Sets the global linear velocity of the NewtonBody.
	 * The MemorySegment holding the linear velocity is expected to have an equivalent layout
	 * to the {@code Newton.VEC3F} MemoryLayout.
	 * @param velocity {@code MemorySegment} containing the linear velocity
	 */
	public void setVelocityNoSleep(MemorySegment velocity) {
		Newton.NewtonBodySetVelocityNoSleep(address, velocity);
	}

	/**
	 * Sets the global linear velocity of the NewtonBody.
	 * The float array must have a length greater than or equal to 3.
	 * @param velocity {@code float[]} containing the linear velocity
	 */
	public void setVelocityNoSleep(float[] velocity) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment velSeg = arena.allocateArray(Newton.C_FLOAT, velocity);
			Newton.NewtonBodySetVelocityNoSleep(address, velSeg);
		}
	}

	/**
	 * Sets the net force applied to the NewtonBody.
	 * The MemorySegment holding the force is expected to have an equivalent layout
	 * to the {@code Newton.VEC3F} MemoryLayout.
	 * This method is only effective when called from {@code NewtonApplyForceAndTorque} callback.
	 * @param force {@code MemorySegment} containing the net force
	 */
	public void setForce(MemorySegment force) {
		Newton.NewtonBodySetForce(address, force);
	}

	/**
	 * Sets the net force applied to the NewtonBody.
	 * The float array must have a length greater than or equal to 3.
	 * This method is only effective when called from {@code NewtonApplyForceAndTorque} callback.
	 * @param force {@code float[]} containing the net force
	 */
	public void setForce(float[] force) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment forceSeg = arena.allocateArray(Newton.C_FLOAT, force);
			Newton.NewtonBodySetForce(address, forceSeg);
		}
	}

	/**
	 * Sets the net torque applied to the NewtonBody.
	 * The MemorySegment holding the torque is expected to have an equivalent layout
	 * to the {@code Newton.VEC3F} MemoryLayout.
	 * This method is only effective when called from {@code NewtonApplyForceAndTorque} callback.
	 * @param torque {@code MemorySegment} containing the net torque
	 */
	public void setTorque(MemorySegment torque) {
		Newton.NewtonBodySetTorque(address, torque);
	}

	/**
	 * Sets the net torque applied to the NewtonBody.
	 * The float array must have a length greater than or equal to 3.
	 * This method is only effective when called from {@code NewtonApplyForceAndTorque} callback.
	 * @param torque {@code float[]} containing the net torque
	 */
	public void setTorque(float[] torque) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment torqueSeg = arena.allocateArray(Newton.C_FLOAT, torque);
			Newton.NewtonBodySetTorque(address, torqueSeg);
		}
	}

	/**
	 * Applies the linear viscous dampening coefficient to the NewtonBody.
	 * @param linearDamp the linear damping coefficient. The value is clamped between 0.0 and 1.0.
	 *                   The default value is 0.1
	 */
	public void setLinearDamping(float linearDamp) {
		Newton.NewtonBodySetLinearDamping(address, linearDamp);
	}

	/**
	 * Applies the angular viscous dampening coefficient to the NewtonBody.
	 * @param angularDamping {@code MemorySegment} containing the angular damping coefficients of the
	 *                                            principal axis of the NewtonBody
	 */
	public void setAngularDamping(MemorySegment angularDamping) {
		Newton.NewtonBodySetAngularDamping(address, angularDamping);
	}

	/**
	 * Applies the angular viscous dampening coefficient to the NewtonBody.
	 * @param angularDamp {@code float[]} containing the damping coefficients of the
	 *                                    principal axis of the NewtonBody
	 */
	public void setAngularDamping(float[] angularDamp) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment dampSeg = arena.allocateArray(Newton.C_FLOAT, angularDamp);
			Newton.NewtonBodySetAngularDamping(address, dampSeg);
		}
	}

	/**
	 * Assigns the collision primitive of the NewtonBody.
	 * @param collision {@NewtonCollision} to used by the body
	 */
	public void setCollision(NewtonCollision collision) {
		Newton.NewtonBodySetCollision(address, collision.address());
	}

	/**
	 * Sets the collision scale of the collision used by the NewtonBody.
	 * @param scaleX scale for the x-axis
	 * @param scaleY scale for the y-axis
	 * @param scaleZ scale for the z-axis
	 */
	public void setCollisionScale(float scaleX, float scaleY, float scaleZ) {
		Newton.NewtonBodySetCollisionScale(address, scaleX, scaleY, scaleZ);
	}

	/**
	 * Gets the sleep state of the NewtonBody.
	 * @return 0 for active | 1 for sleeping
	 */
	public int getSleepState() {
		return Newton.NewtonBodyGetSleepState(address);
	}

	/**
	 * Sets the sleep state of the NewtonBody.
	 * @param state 0 for active | 1 for sleeping
	 */
	public void setSleepState(int state) {
		Newton.NewtonBodySetSleepState(address, state);
	}

	/**
	 * Gets the auto-activation state of the NewtonBody.
	 * Auto-activation is enabled by default.
	 * @return 1 for auto-activation on | 0 for auto-activation off
	 */
	public int getAutoSleepState() {
		return Newton.NewtonBodyGetAutoSleep(address);
	}

	/**
	 * Sets the auto-activation state of the NewtonBody.
	 * Auto-activation is enabled by default.
	 * @param state 1 for auto-activation on | 0 for auto-activation off
	 */
	public void setAutoSleepState(int state) {
		Newton.NewtonBodySetAutoSleep(address, state);
	}

	/**
	 * Gets the freeze state of the NewtonBody.
	 * The freeze attribute determines whether the NewtonBody is actively simulated.
	 * @return 1 for frozen | 0 for unfrozen
	 */
	public int getFreezeState() {
		return Newton.NewtonBodyGetFreezeState(address);
	}

	/**
	 * Sets the freeze state of the NewtonBody.
	 * The freeze attribute determines whether the NewtonBody is actively simulated.
	 * @param state 1 for frozen | 0 for unfrozen
	 */
	public void setFreezeState(int state) {
		Newton.NewtonBodySetFreezeState(address, state);
	}

	/**
	 * Gets the gyroscopic torque of the NewtonBody.
	 * @return gyroscopic torque
	 */
	public int getGyroscopicTorque() {
		return Newton.NewtonBodyGetGyroscopicTorque(address);
	}

	/**
	 * Sets the gyroscopic torque of the NewtonBody.
	 * @param state gyroscopic torque
	 */
	public void setGyroscopicTorque(int state) {
		Newton.NewtonBodySetGyroscopicTorque(address, state);
	}

	/**
	 * Assign a method to be called when the NewtonBody is destroyed.
	 * @param callback {@code MemorySegment} that points to the callback method
	 */
	public void setDestructorCallback(MemorySegment callback) {
		Newton.NewtonBodySetDestructorCallback(address, callback);
	}

	/**
	 * Assign a method to be called when the NewtonBody is destroyed.
	 * @param callback {@code NewtonBodyDestructor} method to be assigned to the body
	 * @param scope {@code SegmentScope} for allocating the {@code MemorySegment} pointing to the callback
	 */
	public void setDestructorCallback(NewtonBodyDestructor callback, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonBodyDestructor.allocate(callback, scope);
		Newton.NewtonBodySetDestructorCallback(address, callbackFunc);
	}

	/**
	 * Gets the destructor callback assigned to the NewtonBody as a raw pointer.
	 * @return {@code MemorySegment} that points to the callback
	 */
	public MemorySegment getRawDestructorCallback() {
		return Newton.NewtonBodyGetDestructorCallback(address);
	}

	/**
	 * Gets the destructor callback assigned to the NewtonBody.
	 * This method wraps the function pointer in a {@code NewtonBodyDestructor}
	 * with the global scope.
	 * @return {@code NewtonBodyDestructor} callback assigned to the body
	 */
	public NewtonBodyDestructor getDestructorCallback() {
		MemorySegment funcAddress = Newton.NewtonBodyGetDestructorCallback(address);
		return NewtonBodyDestructor.ofAddress(funcAddress, SegmentScope.global());
	}

	/**
	 * Gets the destructor callback assigned to the NewtonBody.
	 * This method wraps the function pointer in a {@code NewtonBodyDestructor}
	 * with the same lifetime as the provided scope.
	 * @param scope {@code SegmentScope} that controls the lifetime of the callback
	 * @return {@code NewtonBodyDestructor} callback assigned to the body
	 */
	public NewtonBodyDestructor getDestructorCallback(SegmentScope scope) {
		MemorySegment funcAddress = Newton.NewtonBodyGetDestructorCallback(address);
		return NewtonBodyDestructor.ofAddress(funcAddress, scope);
	}

	/**
	 * Assign a method to be called to update the visual representation of the NewtonBody.
	 * @param callback {@code MemorySegment} that points to the callback method
	 */
	public void setTransformCallback(MemorySegment callback) {
		Newton.NewtonBodySetTransformCallback(address, callback);
	}

	/**
	 * Assign a method to be called to update the visual representation of the NewtonBody.
	 * @param callback {@code NewtonSetTransform} method to be assigned to the NewtonBody
	 * @param scope {@code SegmentScope} that controls the lifetime of the callback
	 */
	public void setTransformCallback(NewtonSetTransform callback, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonSetTransform.allocate(callback, scope);
		Newton.NewtonBodySetTransformCallback(address, callbackFunc);
	}

	/**
	 * Gets the transformation callback assigned to the NewtonBody as a raw pointer.
	 * @return {@code MemorySegment} that points to the callback
	 */
	public MemorySegment getRawTransformCallback() {
		return Newton.NewtonBodyGetTransformCallback(address);
	}

	/**
	 * Gets the transformation callback assigned to the NewtonBody.
	 * This method wraps the function pointer in a {@code NewtonSetTransform}
	 * with the global scope.
	 * @return {@code NewtonSetTransform} callback assigned to the body
	 */
	public NewtonSetTransform getTransformCallback() {
		MemorySegment funcAddress = Newton.NewtonBodyGetTransformCallback(address);
		return NewtonSetTransform.ofAddress(funcAddress, SegmentScope.global());
	}

	/**
	 * Gets the transformation callback assigned to the NewtonBody.
	 * This method wraps the function pointer in a {@code NewtonSetTransform}
	 * with the same lifetime as the provided scope.
	 * @param scope {@code SegmentScope} that controls the lifetime of the callback
	 * @return {@code NewtonSetTransform} callback assigned to the body
	 */
	public NewtonSetTransform getTransformCallback(SegmentScope scope) {
		MemorySegment funcAddress = Newton.NewtonBodyGetTransformCallback(address);
		return NewtonSetTransform.ofAddress(funcAddress, scope);
	}

	/**
	 * Assign a method for applying an external force and torque to the body.
	 * @param callback {@code MemorySegment} that points to the callback method
	 */
	public void setForceAndTorqueCallback(MemorySegment callback) {
		Newton.NewtonBodySetForceAndTorqueCallback(address, callback);
	}

	/**
	 * Assign a method for applying an external force and torque to the body.
	 * @param callback {@code NewtonApplyForceAndTorque} method to be assigned to the NewtonBody
	 * @param scope {@code SegmentScope} that controls the lifetime of the callback
	 */
	public void setForceAndTorqueCallback(NewtonApplyForceAndTorque callback, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonApplyForceAndTorque.allocate(callback, scope);
		Newton.NewtonBodySetForceAndTorqueCallback(address, callbackFunc);
	}

	/**
	 * Gets the force and torque callback assigned to the NewtonBody as a raw pointer.
	 * @return {@code MemorySegment} that points to the callback
	 */
	public MemorySegment getRawForceAndTorqueCallback() {
		return Newton.NewtonBodyGetForceAndTorqueCallback(address);
	}

	/**
	 * Gets the force and torque callback assigned to the NewtonBody.
	 * This method wraps the function pointer in a {@code NewtonSetTransform}
	 * with the global scope.
	 * @return {@code NewtonApplyForceAndTorque} callback assigned to the body
	 */
	public NewtonApplyForceAndTorque getForceAndTorqueCallback() {
		MemorySegment funcAddress = Newton.NewtonBodyGetForceAndTorqueCallback(address);
		return NewtonApplyForceAndTorque.ofAddress(funcAddress, SegmentScope.global());
	}

	/**
	 * Gets the transformation callback assigned to the NewtonBody.
	 * This method wraps the function pointer in a {@code NewtonApplyForceAndTorque}
	 * with the same lifetime as the provided scope.
	 * @param scope {@code SegmentScope} that controls the lifetime of the callback
	 * @return {@code NewtonApplyForceAndTorque} callback assigned to the body
	 */
	public NewtonApplyForceAndTorque getForceAndTorqueCallback(SegmentScope scope) {
		MemorySegment funcAddress = Newton.NewtonBodyGetForceAndTorqueCallback(address);
		return NewtonApplyForceAndTorque.ofAddress(funcAddress, scope);
	}

	/**
	 * Gets the ID of the NewtonBody.
	 * @return bodyID
	 */
	public int getID() {
		return Newton.NewtonBodyGetID(address);
	}

	/**
	 * Set a user defined value to the NewtonBody.
	 * @param data user data to assign to the body
	 */
	public void setUserData(MemorySegment data) {
		Newton.NewtonBodySetUserData(address, data);
	}

	/**
	 * Get the user defined value stored with the NewtonBody.
	 * @return {@code MemorySegment} containing the user defined value
	 */
	public MemorySegment getUserData() {
		return Newton.NewtonBodyGetUserData(address);
	}

	/**
	 * Returns a raw pointer to the NewtonWorld object that owns this NewtonBody.
	 * @return {@code MemorySegment} pointing to the NewtonWorld
	 */
	public MemorySegment getRawWorld() {
		return Newton.NewtonBodyGetWorld(address);
	}

	/**
	 * Returns the {@code NewtonWorld} that owns this NewtonBody.
	 * @return {@code NewtonWorld} that owns this body
	 */
	public NewtonWorld getWorld() {
		return new NewtonWorld(Newton.NewtonBodyGetWorld(address));
	}


	/**
	 * Returns a raw pointer to the collision primitive of this NewtonBody.
	 * @return {@code MemorySegment} that points to the collision primitive
	 */
	public MemorySegment getRawCollision() {
		return Newton.NewtonBodyGetCollision(address);
	}

	/**
	 * Gets the collision primitive of the NewtonBody.
	 * @return {@code NewtonCollision} of the body
	 */
	public NewtonCollision getCollision() {
		return new NewtonCollision(Newton.NewtonBodyGetCollision(address));
	}

	/**
	 * Gets the material group ID of the NewtonBody.
	 * @return material group ID
	 */
	public int getMaterialGroupID() {
		return Newton.NewtonBodyGetMaterialGroupID(address);
	}

	/**
	 * Gets the serialized ID of the NewtonBody.
	 * @return serialized ID
	 */
	public int getSerializedID() {
		return Newton.NewtonBodyGetSerializedID(address);
	}

	/**
	 * Gets the continuous state mode for the NewtonBody.
	 * This is disabled by default.
	 * @return 1 for active | 0 for disabled
	 */
	public int getContinuousCollisionMode() {
		return Newton.NewtonBodyGetContinuousCollisionMode(address);
	}

	/**
	 * Gets the collision state flag of this NewtonBody when the body is connected to another body
	 * by a hierarchy of joints.
	 * @return 1 this body will collide with any linked body | 0 disable collisions with body connected
	 *         this one by joints
	 */
	public int getJointRecursiveCollision() {
		return Newton.NewtonBodyGetJointRecursiveCollision(address);
	}

	/**
	 * Gets the position of the NewtonBody in the NewtonWorld and stores it in
	 * the provided {@code MemorySegment}. The {@code MemorySegment} should have an equivalent
	 * layout to the {@code Newton.VEC3F} MemoryLayout.
	 * @param position {@code MemorySegment} to hold the position
	 */
	public void getPosition(MemorySegment position) {
		Newton.NewtonBodyGetPosition(address, position);
	}

	/**
	 * Gets the position of the NewtonBody in the NewtonWorld and returns it in a {@code float[]}.
	 * The float array will contain the x, y, z of the position in that order.
	 * @return {@code float[]} containing the body position
	 */
	public float[] getPosition() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment posSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetPosition(address, posSeg);
			return posSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 * Gets the transformation matrix of the NewtonBody and stores it in
	 * the provided {@code MemorySegment}. The {@code MemorySegment} should have an equivalent
	 * layout to the {@code Newton.MAT4F} MemoryLayout. The matrix will be stored in column-major order.
	 * @param matrix {@code MemorySegment} to hold the matrix
	 */
	public void getMatrix(MemorySegment matrix) {
		Newton.NewtonBodyGetMatrix(address, matrix);
	}

	/**
	 * Gets the transformation matrix of the NewtonBody and returns it in a {@code float[]}.
	 * The float array will hold the matrix in column-major order
	 * @return {@code float[]} containing the body transformation matrix
	 */
	public float[] getMatrix() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocate(Newton.MAT4F);
			Newton.NewtonBodyGetMatrix(address, matrixSeg);
			return matrixSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 * Gets the rotation part of the transformation matrix as a unit quaternion and stores it
	 * in the provided {@code MemorySegment}. The {@code MemorySegment} should have an equivalent
	 * layout to the {@code Newton.VEC4F} MemoryLayout.
	 * @param rotation {@code MemorySegment} to hold the rotation
	 */
	public void getRotation(MemorySegment rotation) {
		Newton.NewtonBodyGetRotation(address, rotation);
	}

	/**
	 * Gets the rotation part of the transformation matrix as a unit quaternion and returns
	 * it in a {@code float[]}. The float array will contain the x, y, z, w of the rotation in that order.
	 * @return {@code float[]} containing the rotation
	 */
	public float[] getRotation() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment rotSeg = arena.allocate(Newton.VEC4F);
			Newton.NewtonBodyGetRotation(address, rotSeg);
			return rotSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 * Gets the mass matrix of the NewtonBody.
	 * All {@code MemorySegment} parameters should have a layout of {@code Newton.C_FLOAT}.
	 * @param mass {@code MemorySegment} to hold the mass
	 * @param Ixx {@code MemorySegment} to hold the moment of inertia for the first principal axis of inertia
	 * @param Iyy {@code MemorySegment} to hold the moment of inertia for the second principal axis of inertia
	 * @param Izz {@code MemorySegment} to hold the moment of inertia for the third principal axis of inertia
	 */
	public void getMass(MemorySegment mass, MemorySegment Ixx, MemorySegment Iyy, MemorySegment Izz) {
		Newton.NewtonBodyGetMass(address, mass, Ixx, Iyy, Izz);
	}

	/**
	 * Gets the mass matrix of the NewtonBody.
	 * This method doesn't return a full matrix but the mass and first, second, and third
	 * principal axis of inertia as four separate floats in the array.
	 * @return {@code float[]} containing the mass matrix
	 */
	public float[] getMass() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment massSeg = arena.allocate(Newton.VEC4F);
			Newton.NewtonBodyGetMass(address,
					massSeg.asSlice(0L, Newton.C_FLOAT.byteSize()), 
					massSeg.asSlice(4L, Newton.C_FLOAT.byteSize()), 
					massSeg.asSlice(8L, Newton.C_FLOAT.byteSize()), 
					massSeg.asSlice(12L, Newton.C_FLOAT.byteSize()));
			return massSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 * Gets the inverse mass matrix of the NewtonBody.
	 * All {@code MemorySegment} parameters should have a layout of {@code Newton.C_FLOAT}.
	 * @param invMass {@code MemorySegment} to hold the inverse mass
	 * @param invIxx {@code MemorySegment} to hold the moment of inertia inverse for the first principal axis of inertia
	 * @param invIyy {@code MemorySegment} to hold the moment of inertia inverse for the second principal axis of inertia
	 * @param invIzz {@code MemorySegment} to hold the moment of inertia inverse for the third principal axis of inertia
	 */
	public void getInverseMass(MemorySegment invMass, MemorySegment invIxx, MemorySegment invIyy, MemorySegment invIzz) {
		Newton.NewtonBodyGetInvMass(address, invMass, invIxx, invIyy, invIzz);
	}

	/**
	 * Gets the inverse mass matrix of the NewtonBody.
	 * This method doesn't return a full matrix but the mass and first, second, and third
	 * principal axis of inertia as four separate floats in the array.
	 * @return {@code float[]} containing the inverse mass matrix
	 */
	public float[] getInverseMass() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment massSeg = arena.allocate(Newton.VEC4F);
			Newton.NewtonBodyGetInvMass(address,
					massSeg.asSlice(0L, Newton.C_FLOAT.byteSize()), 
					massSeg.asSlice(4L, Newton.C_FLOAT.byteSize()), 
					massSeg.asSlice(8L, Newton.C_FLOAT.byteSize()), 
					massSeg.asSlice(12L, Newton.C_FLOAT.byteSize()));
			return massSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 * Gets the inertia matrix of the NewtonBody and stores it
	 * in the provided {@code MemorySegment}. The {@code MemorySegment} should have an equivalent
	 * layout to the {@code Newton.MAT4F} MemoryLayout. The matrix will be stored in column-major order.
	 * @param matrix {@code MemorySegment} to hold the matrix
	 */
	public void getInertiaMatrix(MemorySegment matrix) {
		Newton.NewtonBodyGetInertiaMatrix(address, matrix);
	}

	/**
	 * Gets the inertia matrix of the NewtonBody and returns it in a {@code float[]}.
	 * The float array will hold the matrix in column-major order.
	 * @return {@code float[]} containing the body inertia matrix
	 */
	public float[] getInertiaMatrix() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocate(Newton.MAT4F);
			Newton.NewtonBodyGetInertiaMatrix(address, matrixSeg);
			return matrixSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 * Gets the inverse inertia matrix of the NewtonBody and stores it
	 * in the provided {@code MemorySegment}. The {@code MemorySegment} should have an equivalent
	 * layout to the {@code Newton.MAT4F} MemoryLayout. The matrix will be stored in column-major order.
	 * @param matrix {@code MemorySegment} to hold the matrix
	 */
	public void getInverseInertiaMatrix(MemorySegment matrix) {
		Newton.NewtonBodyGetInvInertiaMatrix(address, matrix);
	}

	/**
	 * Gets the inverse inertia matrix of the NewtonBody and returns it in a {@code float[]}.
	 * The float array will hold the matrix in column-major order.
	 * @return {@code float[]} containing the body inverse inertia matrix
	 */
	public float[] getInverseInertiaMatrix() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocate(Newton.MAT4F);
			Newton.NewtonBodyGetInvInertiaMatrix(address, matrixSeg);
			return matrixSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 * Gets the global angular velocity of the NewtonBody and stores it
	 * in the provided {@code MemorySegment}. The {@code MemorySegment} should have an equivalent
	 * layout to the {@code Newton.VEC3F} MemoryLayout.
	 * @param omega {@code MemorySegment} to hold the angular velocity
	 */
	public void getOmega(MemorySegment omega) {
		Newton.NewtonBodyGetOmega(address, omega);
	}

	/**
	 * Gets the global angular velocity of the NewtonBody and returns it in a {@code float[]}.
	 * The float array will contain the x, y, z of the angular velocity in that order.
	 * @return {@code float[]} containing the body angular velocity
	 */
	public float[] getOmega() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment omegaSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetOmega(address, omegaSeg);
			return omegaSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 * Gets the global linear velocity of the NewtonBody and stores it
	 * in the provided {@code MemorySegment}. The {@code MemorySegment} should have an equivalent
	 * layout to the {@code Newton.VEC3F} MemoryLayout.
	 * @param velocity {@code MemorySegment} to hold the linear velocity
	 */
	public void getVelocity(MemorySegment velocity) {
		Newton.NewtonBodyGetVelocity(address, velocity);
	}

	/**
	 * Gets the global linear velocity of the NewtonBody and returns it in a {@code float[]}.
	 * The float array will contain the x, y, z of the linear velocity in that order.
	 * @return {@code float[]} containing the body linear velocity
	 */
	public float[] getVelocity() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment velSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetVelocity(address, velSeg);
			return velSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 * Gets the global angular acceleration of the NewtonBody and stores it
	 * in the provided {@code MemorySegment}. The {@code MemorySegment} should have an equivalent
	 * layout to the {@code Newton.VEC3F} MemoryLayout.
	 * @param alpha {@code MemorySegment} to hold the angular acceleration
	 */
	public void getAlpha(MemorySegment alpha) {
		Newton.NewtonBodyGetAlpha(address, alpha);
	}

	/**
	 * Gets the global angular acceleration of the NewtonBody and returns it in a {@code float[]}.
	 * The float array will contain the x, y, z of the angular acceleration in that order.
	 * @return {@code float[]} containing the body angular acceleration
	 */
	public float[] getAlpha() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment alphaSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetAlpha(address, alphaSeg);
			return alphaSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 * Gets the global linear acceleration of the NewtonBody and stores it
	 * in the provided {@code MemorySegment}. The {@code MemorySegment} should have an equivalent
	 * layout to the {@code Newton.VEC3F} MemoryLayout.
	 * @param acceleration {@code MemorySegment} to hold the linear acceleration
	 */
	public void getAcceleration(MemorySegment acceleration) {
		Newton.NewtonBodyGetAcceleration(address, acceleration);
	}

	/**
	 * Gets the global linear acceleration of the NewtonBody and returns it in a {@code float[]}.
	 * The float array will contain the x, y, z of the linear velocity in that order.
	 * @return {@code float[]} containing the body linear acceleration
	 */
	public float[] getAcceleration() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment accSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetAcceleration(address, accSeg);
			return accSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 * Gets the net force applied to the NewtonBody after the last {@code NewtonWorld.update()} and
	 * stores it in the provided {@code MemorySegment}. The {@code MemorySegment} should have an equivalent
	 * layout to the {@code Newton.VEC3F} MemoryLayout.
	 * @param force {@code MemorySegment} to hold the applied net force
	 */
	public void getForce(MemorySegment force) {
		Newton.NewtonBodyGetForce(address, force);
	}

	/**
	 * Gets the net force applied to the NewtonBody after the last {@code NewtonWorld.update()} and
	 * returns it in a {@code float[]}. The float array will contain the x, y, z of the applied net force
	 * in that order.
	 * @return {@code float[]} containing the applied net force
	 */
	public float[] getForce() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment forceSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetForce(address, forceSeg);
			return forceSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 * Gets the net torque applied to the NewtonBody after the last {@code NewtonWorld.update()} and
	 * stores it in the provided {@code MemorySegment}. The {@code MemorySegment} should have an equivalent
	 * layout to the {@code Newton.VEC3F} MemoryLayout.
	 * @param torque {@code MemorySegment} to hold the applied net torque
	 */
	public void getTorque(MemorySegment torque) {
		Newton.NewtonBodyGetTorque(address, torque);
	}

	/**
	 * Gets the net torque applied to the NewtonBody after the last {@code NewtonWorld.update()} and
	 * returns it in a {@code float[]}. The float array will contain the x, y, z of the applied net torque
	 * in that order.
	 * @return {@code float[]} containing the applied net torque
	 */
	public float[] getTorque() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment torqueSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetTorque(address, torqueSeg);
			return torqueSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 * Gets the relative position of the center of mass of the NewtonBody and
	 * stores it in the provided {@code MemorySegment}. The {@code MemorySegment} should have an equivalent
	 * layout to the {@code Newton.VEC3F} MemoryLayout.
	 * @param center {@code MemorySegment} to hold the applied net torque
	 */
	public void getCenterOfMass(MemorySegment center) {
		Newton.NewtonBodyGetCentreOfMass(address, center);
	}

	/**
	 * Gets the relative position of the center of mass of the NewtonBody and
	 * returns it in a {@code float[]}. The float array will contain the x, y, z of the center of mass
	 * in that order.
	 * @return {@code float[]} containing the center of mass
	 */
	public float[] getCenterOfMass() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment comSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetCentreOfMass(address, comSeg);
			return comSeg.toArray(Newton.C_FLOAT);
		}
	}

	/**
	 *
	 * @param point
	 * @param velocity
	 */
	public void getPointVelocity(MemorySegment point, MemorySegment velocity) {
		Newton.NewtonBodyGetPointVelocity(address, point, velocity);
	}

	public void getPointVelocity(float[] point, MemorySegment velocity) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment pointSeg = arena.allocateArray(Newton.C_FLOAT, point);
			Newton.NewtonBodyGetPointVelocity(address, pointSeg, velocity);
		}
	}

	public float[] getPointVelocity(float[] point) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment pointSeg = arena.allocateArray(Newton.C_FLOAT, point);
			MemorySegment velSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetPointVelocity(address, pointSeg, velSeg);
			return velSeg.toArray(Newton.C_FLOAT);
		}
	}
	
	public void addImpulsePair(MemorySegment linearImpulse, MemorySegment angularImpulse, float timestep) {
		Newton.NewtonBodyApplyImpulsePair(address, linearImpulse, angularImpulse, timestep);
	}

	public void addImpulsePair(float[] linearImpulse, float[] angularImpulse, float timestep) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment linearSegment = arena.allocateArray(Newton.C_FLOAT, linearImpulse);
			MemorySegment angularSegment = arena.allocateArray(Newton.C_FLOAT, angularImpulse);
			Newton.NewtonBodyApplyImpulsePair(address, linearSegment, angularSegment, timestep);
		}
	}

	public void addImpulse(MemorySegment deltaVelocity, MemorySegment point, float timestep) {
		Newton.NewtonBodyAddImpulse(address, deltaVelocity, point, timestep);
	}

	public void addImpulse(float[] deltaVelocity, float[] point, float timestep) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment velSeg = arena.allocateArray(Newton.C_FLOAT, deltaVelocity);
			MemorySegment pointSeg = arena.allocateArray(Newton.C_FLOAT, point);
			Newton.NewtonBodyAddImpulse(address, velSeg, pointSeg, timestep);
		}
	}

	public void addImpulseArray(int impulseCount, int strideInBytes, MemorySegment impulseArray, MemorySegment pointArray, float timestep) {
		Newton.NewtonBodyApplyImpulseArray(address, impulseCount, strideInBytes, impulseArray, pointArray, timestep);
	}

	public void addImpulseArray(int impulseCount, MemorySegment impulseArray, MemorySegment pointArray, float timestep) {
		int strideInBytes = (int) Newton.VEC3F.byteSize();
		Newton.NewtonBodyApplyImpulseArray(address, impulseCount, strideInBytes, impulseArray, pointArray, timestep);
	}

	public void addImpulseArray(int impulseCount, float[] impulseArray, float[] pointArray, float timestep) {
		try (Arena arena = Arena.openConfined()) {
			int strideInBytes = (int) Newton.VEC3F.byteSize();
			MemorySegment impulseSegment = arena.allocateArray(Newton.C_FLOAT, impulseArray);
			MemorySegment pointSegment = arena.allocateArray(Newton.C_FLOAT, pointArray);
			Newton.NewtonBodyApplyImpulseArray(address, impulseCount, strideInBytes, impulseSegment, pointSegment, timestep);
		}
	}

	public void integrateVelocity(float timestep) {
		Newton.NewtonBodyIntegrateVelocity(address, timestep);
	}

	public float getLinearDamping() {
		return Newton.NewtonBodyGetLinearDamping(address);
	}

	public void getAngularDamping(MemorySegment damping) {
		Newton.NewtonBodyGetAngularDamping(address, damping);
	}

	public float[] getAngularDamping() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment vecSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetAngularDamping(address, vecSeg);
			return vecSeg.toArray(Newton.C_FLOAT);
		}
	}

	public void getAABB(MemorySegment aabb) {
		Newton.NewtonBodyGetAABB(address,
				aabb.asSlice(0L, Newton.VEC3F.byteSize()),
				aabb.asSlice(Newton.VEC3F.byteSize()));
	}

	public float[] getAABB() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment aabb = arena.allocate(Newton.AABBF);
			Newton.NewtonBodyGetAABB(address,
					aabb.asSlice(0L, Newton.VEC3F.byteSize()),
					aabb.asSlice(Newton.VEC3F.byteSize()));
			return aabb.toArray(Newton.C_FLOAT);
		}
	}

	public void destroy() {
		Newton.NewtonDestroyBody(address);
	}
}
