package crab.newton;


import crab.newton.callbacks.NewtonApplyForceAndTorque;
import crab.newton.callbacks.NewtonBodyDestructor;
import crab.newton.callbacks.NewtonSetTransform;
import crab.newton.internal.*;
import java.lang.foreign.*;

public final class NewtonBody {

	public static final int DYNAMIC_BODY = Newton.NEWTON_DYNAMIC_BODY(),
			KINEMATIC_BODY = Newton.NEWTON_KINEMATIC_BODY(),
			DYNAMIC_ASYMETRIC_BODY = Newton.NEWTON_DYNAMIC_ASYMETRIC_BODY();
	protected final MemorySegment address;
	/**
	 * int representing type of the NewtonBody
	 *
	 */
	public final int bodyType;
	
	public NewtonBody(MemorySegment address) {
		this(address, Newton.NewtonBodyGetType(address));
	}

	protected NewtonBody(MemorySegment address, int bodyType) {
		this.address = address;
		this.bodyType = bodyType;
	}

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
	 * Adds the force vector to the NewtonBody.
	 * The MemorySegment holding the force vector is expected to have an equivalent layout
	 * to the {@code VEC3F} MemoryLayout.
	 * This method is only effective when called from {@code NewtonApplyForceAndTorque} callback.
	 * @param force {@code MemorySegment} containing the force vector
	 */
	public void addForce(MemorySegment force) {
		Newton.NewtonBodyAddForce(address, force);
	}

	/**
	 * Adds the force vector to the NewtonBody.
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
	 * Adds the torque vector to the NewtonBody.
	 * The MemorySegment holding the torque vector is expected to have an equivalent layout
	 * to the {@code VEC3F} MemoryLayout.
	 * This method is only effective when called from {@code NewtonApplyForceAndTorque} callback.
	 * @param torque
	 */
	public void addTorque(MemorySegment torque) {
		Newton.NewtonBodyAddTorque(address, torque);
	}

	/**
	 * Adds the torque vector to the NewtonBody.
	 * The float array must have a length greater than or equal to 3.
	 * This method is only effective when called from {@code NewtonApplyForceAndTorque} callback.
	 * @param torque
	 */
	public void addTorque(float[] torque) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment torqueSeg = arena.allocateArray(Newton.C_FLOAT, torque);
			Newton.NewtonBodyAddTorque(address, torqueSeg);
		}
	}

	public void setCenterOfMass(MemorySegment center) {
		Newton.NewtonBodySetCentreOfMass(address, center);
	}

	public void setCenterOfMass(float[] center) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment centerSeg = arena.allocateArray(Newton.C_FLOAT, center);
			Newton.NewtonBodySetCentreOfMass(address, centerSeg);
		}
	}

	public void setMassMatrix(float mass, float Ixx, float Iyy, float Izz) {
		Newton.NewtonBodySetMassMatrix(address, mass, Ixx, Iyy, Izz);
	}

	public void setFullMassMatrix(float mass, MemorySegment inertiaMatrix) {
		Newton.NewtonBodySetFullMassMatrix(address, mass, inertiaMatrix);
	}

	public void setFullMassMatrix(float mass, float[] inertiaMatrix) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, inertiaMatrix);
			Newton.NewtonBodySetFullMassMatrix(address, mass, matrix);
		}
	}

	public void setMassProperties(float mass, NewtonCollision collision) {
		Newton.NewtonBodySetMassProperties(address, mass, collision.address);
	}

	public void setMatrix(MemorySegment matrix) {
		Newton.NewtonBodySetMatrix(address, matrix);
	}

	public void setMatrix(float[] matrix) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocateArray(Newton.C_FLOAT, matrix);
			Newton.NewtonBodySetMatrix(address, matrixSeg);
		}
	}

	public void setMatrixNoSleep(MemorySegment matrix) {
		Newton.NewtonBodySetMatrixNoSleep(address, matrix);
	}

	public void setMatrixNoSleep(float[] matrix) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocateArray(Newton.C_FLOAT, matrix);
			Newton.NewtonBodySetMatrixNoSleep(address, matrixSeg);
		}
	}

	public void setMatrixRecursive(MemorySegment matrix) {
		Newton.NewtonBodySetMatrixRecursive(address, matrix);
	}

	public void setMatrixRecursive(float[] matrix) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocateArray(Newton.C_FLOAT, matrix);
			Newton.NewtonBodySetMatrixRecursive(address, matrixSeg);
		}
	}

	public void setMaterialGroupdID(int id) {
		Newton.NewtonBodySetMaterialGroupID(address, id);
	}

	public void setContinuousCollisionMode(int state) {
		Newton.NewtonBodySetContinuousCollisionMode(address, state);
	}

	public void setJointRecursiveCollision(int state) {
		Newton.NewtonBodySetJointRecursiveCollision(address, state);
	}

	public void setOmega(MemorySegment omega) {
		Newton.NewtonBodySetOmega(address, omega);
	}

	public void setOmega(float[] omega) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment omegaSeg = arena.allocateArray(Newton.C_FLOAT, omega);
			Newton.NewtonBodySetOmega(address, omegaSeg);
		}
	}

	public void setOmegaNoSleep(MemorySegment omega) {
		Newton.NewtonBodySetOmegaNoSleep(address, omega);
	}

	public void setOmegaNoSleep(float[] omega) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment omegaSeg = arena.allocateArray(Newton.C_FLOAT, omega);
			Newton.NewtonBodySetOmegaNoSleep(address, omegaSeg);
		}
	}

	public void setVelocity(MemorySegment velocity) {
		Newton.NewtonBodySetVelocity(address, velocity);
	}

	public void setVelocity(float[] velocity) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment velSeg = arena.allocateArray(Newton.C_FLOAT, velocity);
			Newton.NewtonBodySetVelocity(address, velSeg);
		}
	}

	public void setVelocityNoSleep(MemorySegment velocity) {
		Newton.NewtonBodySetVelocityNoSleep(address, velocity);
	}

	public void setVelocityNoSleep(float[] velocity) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment velSeg = arena.allocateArray(Newton.C_FLOAT, velocity);
			Newton.NewtonBodySetVelocityNoSleep(address, velSeg);
		}
	}

	public void setForce(MemorySegment force) {
		Newton.NewtonBodySetForce(address, force);
	}

	public void setForce(float[] force) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment forceSeg = arena.allocateArray(Newton.C_FLOAT, force);
			Newton.NewtonBodySetForce(address, forceSeg);
		}
	}

	public void setTorque(MemorySegment torque) {
		Newton.NewtonBodySetTorque(address, torque);
	}

	public void setTorque(float[] torque) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment torqueSeg = arena.allocateArray(Newton.C_FLOAT, torque);
			Newton.NewtonBodySetTorque(address, torqueSeg);
		}
	}

	public void setLinearDamping(float linearDamp) {
		Newton.NewtonBodySetLinearDamping(address, linearDamp);
	}

	public void setAngularDamping(MemorySegment angularDamping) {
		Newton.NewtonBodySetAngularDamping(address, angularDamping);
	}

	public void setAngularDamping(float[] angularDamp) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment dampSeg = arena.allocateArray(Newton.C_FLOAT, angularDamp);
			Newton.NewtonBodySetAngularDamping(address, dampSeg);
		}
	}

	public void setCollision(NewtonCollision collision) {
		Newton.NewtonBodySetCollision(address, collision.address);
	}

	public void setCollisionScale(float scaleX, float scaleY, float scaleZ) {
		Newton.NewtonBodySetCollisionScale(address, scaleX, scaleY, scaleZ);
	}

	public int getSleepState() {
		return Newton.NewtonBodyGetSleepState(address);
	}

	public void setSleepState(int state) {
		Newton.NewtonBodySetSleepState(address, state);
	}

	public int getAutoSleepState() {
		return Newton.NewtonBodyGetAutoSleep(address);
	}

	public void setAutoSleepState(int state) {
		Newton.NewtonBodySetAutoSleep(address, state);
	}

	public int getFreezeState() {
		return Newton.NewtonBodyGetFreezeState(address);
	}

	public void setFreezeState(int state) {
		Newton.NewtonBodySetFreezeState(address, state);
	}

	public int getGyroscopicTorque() {
		return Newton.NewtonBodyGetGyroscopicTorque(address);
	}

	public void setGyroscopicTorque(int state) {
		Newton.NewtonBodySetGyroscopicTorque(address, state);
	}
	
	public void setDestructorCallback(MemorySegment callback) {
		Newton.NewtonBodySetDestructorCallback(address, callback);
	}

	public void setDestructorCallback(NewtonBodyDestructor callback, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonBodyDestructor.allocate(callback, scope);
		Newton.NewtonBodySetDestructorCallback(address, callbackFunc);
	}
	
	public MemorySegment getDestructorCallback() {
		return Newton.NewtonBodyGetDestructorCallback(address);
	}

	public NewtonBodyDestructor getDestructorCallback(SegmentScope scope) {
		MemorySegment funcAddress = Newton.NewtonBodyGetDestructorCallback(address);
		return NewtonBodyDestructor.ofAddress(funcAddress, scope);
	}
	
	public void setTransformCallback(MemorySegment callback) {
		Newton.NewtonBodySetTransformCallback(address, callback);
	}

	public void setTransformCallback(NewtonSetTransform callback, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonSetTransform.allocate(callback, scope);
		Newton.NewtonBodySetTransformCallback(address, callbackFunc);
	}
	
	public MemorySegment getTransformCallback() {
		return Newton.NewtonBodyGetTransformCallback(address);
	}

	public NewtonSetTransform getTransformCallback(SegmentScope scope) {
		MemorySegment funcAddress = Newton.NewtonBodyGetTransformCallback(address);
		return NewtonSetTransform.ofAddress(funcAddress, scope);
	}
	
	public void setForceAndTorqueCallback(MemorySegment callback) {
		Newton.NewtonBodySetForceAndTorqueCallback(address, callback);
	}

	public void setForceAndTorqueCallback(NewtonApplyForceAndTorque callback, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonApplyForceAndTorque.allocate(callback, scope);
		Newton.NewtonBodySetForceAndTorqueCallback(address, callbackFunc);
	}

	public MemorySegment getForceAndTorqueCallback() {
		return Newton.NewtonBodyGetForceAndTorqueCallback(address);
	}

	public NewtonApplyForceAndTorque getForceAndTorqueCallback(SegmentScope scope) {
		MemorySegment funcAddress = Newton.NewtonBodyGetForceAndTorqueCallback(address);
		return NewtonApplyForceAndTorque.ofAddress(funcAddress, scope);
	}

	public int getID() {
		return Newton.NewtonBodyGetID(address);
	}

	public void setUserData(MemorySegment data) {
		Newton.NewtonBodySetUserData(address, data);
	}

	public MemorySegment getUserData() {
		return Newton.NewtonBodyGetUserData(address);
	}

	public NewtonWorld getWorld() {
		return NewtonWorld.wrap(Newton.NewtonBodyGetWorld(address));
	}

	public NewtonCollision getCollision() {
		return NewtonCollision.wrap(Newton.NewtonBodyGetCollision(address));
	}

	public int getMaterialGroupID() {
		return Newton.NewtonBodyGetMaterialGroupID(address);
	}

	public int getSerializedID() {
		return Newton.NewtonBodyGetSerializedID(address);
	}

	public int getContinuousCollisionMode() {
		return Newton.NewtonBodyGetContinuousCollisionMode(address);
	}

	public int getJointRecursiveCollision() {
		return Newton.NewtonBodyGetJointRecursiveCollision(address);
	}

	public void getPosition(MemorySegment position) {
		Newton.NewtonBodyGetPosition(address, position);
	}

	public float[] getPosition() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment posSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetPosition(address, posSeg);
			return posSeg.toArray(Newton.C_FLOAT);
		}
	}

	public void getMatrix(MemorySegment matrix) {
		Newton.NewtonBodyGetMatrix(address, matrix);
	}

	public float[] getMatrix() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocate(Newton.MAT4F);
			Newton.NewtonBodyGetMatrix(address, matrixSeg);
			return matrixSeg.toArray(Newton.C_FLOAT);
		}
	}

	public void getRotation(MemorySegment rotation) {
		Newton.NewtonBodyGetRotation(address, rotation);
	}

	public float[] getRotation() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment rotSeg = arena.allocate(Newton.VEC4F);
			Newton.NewtonBodyGetRotation(address, rotSeg);
			return rotSeg.toArray(Newton.C_FLOAT);
		}
	}

	public void getMass(MemorySegment mass) {
		Newton.NewtonBodyGetMass(address,
				mass.asSlice(0L, Newton.C_FLOAT.byteSize()),
				mass.asSlice(4L, Newton.C_FLOAT.byteSize()),
				mass.asSlice(8L, Newton.C_FLOAT.byteSize()),
				mass.asSlice(12L, Newton.C_FLOAT.byteSize()));
	}

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

	public void getInverseMass(MemorySegment invMass) {
		Newton.NewtonBodyGetInvMass(address,
				invMass.asSlice(0L, Newton.C_FLOAT.byteSize()),
				invMass.asSlice(4L, Newton.C_FLOAT.byteSize()),
				invMass.asSlice(8L, Newton.C_FLOAT.byteSize()),
				invMass.asSlice(12L, Newton.C_FLOAT.byteSize()));
	}

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

	public void getInertiaMatrix(MemorySegment matrix) {
		Newton.NewtonBodyGetInertiaMatrix(address, matrix);
	}

	public float[] getInertiaMatrix() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocate(Newton.MAT4F);
			Newton.NewtonBodyGetInertiaMatrix(address, matrixSeg);
			return matrixSeg.toArray(Newton.C_FLOAT);
		}
	}

	public void getInverseInertiaMatrix(MemorySegment matrix) {
		Newton.NewtonBodyGetInvInertiaMatrix(address, matrix);
	}

	public float[] getInverseInertiaMatrix() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocate(Newton.MAT4F);
			Newton.NewtonBodyGetInvInertiaMatrix(address, matrixSeg);
			return matrixSeg.toArray(Newton.C_FLOAT);
		}
	}

	public void getOmega(MemorySegment omega) {
		Newton.NewtonBodyGetOmega(address, omega);
	}

	public float[] getOmega() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment omegaSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetOmega(address, omegaSeg);
			return omegaSeg.toArray(Newton.C_FLOAT);
		}
	}

	public void getVelocity(MemorySegment velocity) {
		Newton.NewtonBodyGetVelocity(address, velocity);
	}

	public float[] getVelocity() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment velSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetVelocity(address, velSeg);
			return velSeg.toArray(Newton.C_FLOAT);
		}
	}

	public void getAlpha(MemorySegment alpha) {
		Newton.NewtonBodyGetAlpha(address, alpha);
	}

	public float[] getAlpha() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment alphaSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetAlpha(address, alphaSeg);
			return alphaSeg.toArray(Newton.C_FLOAT);
		}
	}

	public void getAcceleration(MemorySegment acceleration) {
		Newton.NewtonBodyGetAcceleration(address, acceleration);
	}

	public float[] getAcceleration() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment accSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetAcceleration(address, accSeg);
			return accSeg.toArray(Newton.C_FLOAT);
		}
	}

	public void getForce(MemorySegment force) {
		Newton.NewtonBodyGetForce(address, force);
	}

	public float[] getForce() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment forceSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetForce(address, forceSeg);
			return forceSeg.toArray(Newton.C_FLOAT);
		}
	}

	public void getTorque(MemorySegment torque) {
		Newton.NewtonBodyGetTorque(address, torque);
	}

	public float[] getTorque() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment torqueSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetTorque(address, torqueSeg);
			return torqueSeg.toArray(Newton.C_FLOAT);
		}
	}

	public void getCenterOfMass(MemorySegment center) {
		Newton.NewtonBodyGetCentreOfMass(address, center);
	}

	public float[] getCenterOfMass() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment comSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonBodyGetCentreOfMass(address, comSeg);
			return comSeg.toArray(Newton.C_FLOAT);
		}
	}

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
	
	public static NewtonBody wrap(MemorySegment address) {
		int bodyType = Newton.NewtonBodyGetType(address);
		return new NewtonBody(address, bodyType);
	}
}
