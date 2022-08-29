package crab.newton;


import crab.newton.callbacks.NewtonApplyForceAndTorque;
import crab.newton.callbacks.NewtonBodyDestructor;
import crab.newton.callbacks.NewtonSetTransform;
import crab.newton.internal.*;
import jdk.incubator.foreign.*;

public sealed interface NewtonBody permits NewtonAsymetricDynamicBody, NewtonKinematicBody, NewtonDynamicBody {
	
	/**
	 * Gets the address of the NewtonBody
	 * @return
	 */
	MemoryAddress address();
	
	/**
	 * Gets the simulation state of the body
	 * @return 0 if body is disabled or 1 if body is active
	 */
	default int getSimulationState() {
		return Newton_h.NewtonBodyGetSimulationState(address());
	}
	
	/**
	 * Sets the simulation state of the body
	 * @param state - 0 to disable body | 1 to activate body
	 */
	default void setSimulationState(int state) {
		Newton_h.NewtonBodySetSimulationState(address(), state);
	}
	
	/**
	 * Gets the type of {@code this} body
	 * @return 0 for dynamic body, 1 for kinematic body, 2 for
	 */
	default int getType() {
		return Newton_h.NewtonBodyGetType(address());
	}
	
	default int getCollidable() {
		return Newton_h.NewtonBodyGetCollidable(address());
	}
	
	default void setCollidable(int collidableState) {
		Newton_h.NewtonBodySetCollidable(address(), collidableState);
	}
	
	default void addForce(float[] force) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment forceSeg = allocator.allocateArray(Newton_h.C_FLOAT, force);
			Newton_h.NewtonBodyAddForce(address(), forceSeg);
		}
	}
	
	default void addTorque(float[] torque) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment torqueSeg = allocator.allocateArray(Newton_h.C_FLOAT, torque);
			Newton_h.NewtonBodyAddTorque(address(), torqueSeg);
		}
	}
	
	default void setCenterOfMass(float[] center) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment centerSeg = allocator.allocateArray(Newton_h.C_FLOAT, center);
			Newton_h.NewtonBodySetCentreOfMass(address(), centerSeg);
		}
	}
	
	default void setMassMatrix(float mass, float Ixx, float Iyy, float Izz) {
		Newton_h.NewtonBodySetMassMatrix(address(), mass, Ixx, Iyy, Izz);
	}
	
	default void setFullMassMatrix(float mass, float[] inertiaMatrix) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment matrix = allocator.allocateArray(Newton_h.C_FLOAT, inertiaMatrix);
			Newton_h.NewtonBodySetFullMassMatrix(address(), mass, matrix);
		}
	}
	
	default void setMassProperties(float mass, NewtonCollision collision) {
		Newton_h.NewtonBodySetMassProperties(address(), mass, collision.address());
	}
	
	default void setMatrix(float[] matrix) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment matrixSeg = allocator.allocateArray(Newton_h.C_FLOAT, matrix);
			Newton_h.NewtonBodySetMatrix(address(), matrixSeg);
		}
	}
	
	default void setMatrixNoSleep(float[] matrix) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment matrixSeg = allocator.allocateArray(Newton_h.C_FLOAT, matrix);
			Newton_h.NewtonBodySetMatrixNoSleep(address(), matrixSeg);
		}
	}
	
	default void setMatrixRecursive(float[] matrix) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment matrixSeg = allocator.allocateArray(Newton_h.C_FLOAT, matrix);
			Newton_h.NewtonBodySetMatrixRecursive(address(), matrixSeg);
		}
	}
	
	default void setMaterialGroupdID(int id) {
		Newton_h.NewtonBodySetMaterialGroupID(address(), id);
	}
	
	default void setContinuousCollisionMode(int state) {
		Newton_h.NewtonBodySetContinuousCollisionMode(address(), state);
	}
	
	default void setJointRecursiveCollision(int state) {
		Newton_h.NewtonBodySetJointRecursiveCollision(address(), state);
	}
	
	default void setOmega(float[] omega) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment omegaSeg = allocator.allocateArray(Newton_h.C_FLOAT, omega);
			Newton_h.NewtonBodySetOmega(address(), omegaSeg);
		}
	}
	
	default void setOmegaNoSleep(float[] omega) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment omegaSeg = allocator.allocateArray(Newton_h.C_FLOAT, omega);
			Newton_h.NewtonBodySetOmegaNoSleep(address(), omegaSeg);
		}
	}
	
	default void setVelocity(float[] velocity) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment velSeg = allocator.allocateArray(Newton_h.C_FLOAT, velocity);
			Newton_h.NewtonBodySetVelocity(address(), velSeg);
		}
	}
	
	default void setVelocityNoSleep(float[] velocity) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment velSeg = allocator.allocateArray(Newton_h.C_FLOAT, velocity);
			Newton_h.NewtonBodySetVelocityNoSleep(address(), velSeg);
		}
	}
	
	default void setForce(float[] force) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment forceSeg = allocator.allocateArray(Newton_h.C_FLOAT, force);
			Newton_h.NewtonBodySetForce(address(), forceSeg);
		}
	}
	
	default void setTorque(float[] torque) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment torqueSeg = allocator.allocateArray(Newton_h.C_FLOAT, torque);
			Newton_h.NewtonBodySetTorque(address(), torqueSeg);
		}
	}
	
	default void setLinearDamping(float linearDamp) {
		Newton_h.NewtonBodySetLinearDamping(address(), linearDamp);
	}
	
	default void setAngularDamping(float[] angularDamp) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment dampSeg = allocator.allocateArray(Newton_h.C_FLOAT, angularDamp);
			Newton_h.NewtonBodySetAngularDamping(address(), dampSeg);
		}
	}
	
	default void setCollision(NewtonCollision collision) {
		Newton_h.NewtonBodySetCollision(address(), collision.address());
	}
	
	default void setCollisionScale(float scaleX, float scaleY, float scaleZ) {
		Newton_h.NewtonBodySetCollisionScale(address(), scaleX, scaleY, scaleZ);
	}
	
	default int getSleepState() {
		return Newton_h.NewtonBodyGetSleepState(address());
	}
	
	default void setSleepState(int state) {
		Newton_h.NewtonBodySetSleepState(address(), state);
	}
	
	default int getAutoSleepState() {
		return Newton_h.NewtonBodyGetAutoSleep(address());
	}
	
	default void setAutoSleepState(int state) {
		Newton_h.NewtonBodySetAutoSleep(address(), state);
	}
	
	default int getFreezeState() {
		return Newton_h.NewtonBodyGetFreezeState(address());
	}
	
	default void setFreezeState(int state) {
		Newton_h.NewtonBodySetFreezeState(address(), state);
	}
	
	default int getGyroscopicTorque() {
		return Newton_h.NewtonBodyGetGyroscopicTorque(address());
	}
	
	default void setGyroscopicTorque(int state) {
		Newton_h.NewtonBodySetGyroscopicTorque(address(), state);
	}
	
	default void setDestructorCallback(NewtonBodyDestructor callback, ResourceScope scope) {
		NativeSymbol callbackFunc = NewtonBodyDestructor.allocate(callback, scope);
		Newton_h.NewtonBodySetDestructorCallback(address(), callbackFunc);
	}
	
	default NewtonBodyDestructor getDestructorCallback(ResourceScope scope) {
		MemoryAddress funcAddress = Newton_h.NewtonBodyGetDestructorCallback(address());
		return NewtonBodyDestructor.ofAddress(funcAddress, scope);
	}
	
	default void setTransformCallback(NewtonSetTransform callback, ResourceScope scope) {
		NativeSymbol callbackFunc = NewtonSetTransform.allocate(callback, scope);
		Newton_h.NewtonBodySetTransformCallback(address(), callbackFunc);
	}
	
	default NewtonSetTransform getTransformCallback(ResourceScope scope) {
		MemoryAddress funcAddress = Newton_h.NewtonBodyGetTransformCallback(address());
		return NewtonSetTransform.ofAddress(funcAddress, scope);
	}
	
	default void setForceAndTorqueCallback(NewtonApplyForceAndTorque callback, ResourceScope scope) {
		NativeSymbol callbackFunc = NewtonApplyForceAndTorque.allocate(callback, scope);
		Newton_h.NewtonBodySetForceAndTorqueCallback(address(), callbackFunc);
	}
	
	default NewtonApplyForceAndTorque getForceAndTorqueCallback(ResourceScope scope) {
		MemoryAddress funcAddress = Newton_h.NewtonBodyGetForceAndTorqueCallback(address());
		return NewtonApplyForceAndTorque.ofAddress(funcAddress, scope);
	}
	
	default int getID() {
		return Newton_h.NewtonBodyGetID(address());
	}
	
	default void setUserData(Addressable data) {
		Newton_h.NewtonBodySetUserData(address(), data);
	}
	
	default MemoryAddress getUserData() {
		return Newton_h.NewtonBodyGetUserData(address());
	}
	
	default NewtonWorld getWorld() {
		return NewtonWorld.wrap(Newton_h.NewtonBodyGetWorld(address()));
	}
	
	default NewtonCollision getCollision() {
		return NewtonCollision.wrap(Newton_h.NewtonBodyGetCollision(address()));
	}
	
	default int getMaterialGroupID() {
		return Newton_h.NewtonBodyGetMaterialGroupID(address());
	}
	
	default int getSerializedID() {
		return Newton_h.NewtonBodyGetSerializedID(address());
	}
	
	default int getContinuousCollisionMode() {
		return Newton_h.NewtonBodyGetContinuousCollisionMode(address());
	}
	
	default int getJointRecursiveCollision() {
		return Newton_h.NewtonBodyGetJointRecursiveCollision(address());
	}
	
	default float[] getPosition() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment posSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.VEC3F);
			Newton_h.NewtonBodyGetPosition(address(), posSeg);
			return posSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getMatrix() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment matrixSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.MAT4F);
			Newton_h.NewtonBodyGetMatrix(address(), matrixSeg);
			return matrixSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getRotation() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment rotSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.VEC4F);
			Newton_h.NewtonBodyGetRotation(address(), rotSeg);
			return rotSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getMass() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment massSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.VEC4F);
			Newton_h.NewtonBodyGetMass(address(), 
					massSeg.asSlice(0L, Newton_h.C_FLOAT.byteSize()), 
					massSeg.asSlice(4L, Newton_h.C_FLOAT.byteSize()), 
					massSeg.asSlice(8L, Newton_h.C_FLOAT.byteSize()), 
					massSeg.asSlice(12L, Newton_h.C_FLOAT.byteSize()));
			return massSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getInverseMass() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment massSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.VEC4F);
			Newton_h.NewtonBodyGetInvMass(address(), 
					massSeg.asSlice(0L, Newton_h.C_FLOAT.byteSize()), 
					massSeg.asSlice(4L, Newton_h.C_FLOAT.byteSize()), 
					massSeg.asSlice(8L, Newton_h.C_FLOAT.byteSize()), 
					massSeg.asSlice(12L, Newton_h.C_FLOAT.byteSize()));
			return massSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getInertiaMatrix() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment matrixSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.MAT4F);
			Newton_h.NewtonBodyGetInertiaMatrix(address(), matrixSeg);
			return matrixSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getInverseInertiaMatrix() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment matrixSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.MAT4F);
			Newton_h.NewtonBodyGetInvInertiaMatrix(address(), matrixSeg);
			return matrixSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getOmega() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment omegaSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.VEC3F);
			Newton_h.NewtonBodyGetOmega(address(), omegaSeg);
			return omegaSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getVelocity() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment velSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.VEC3F);
			Newton_h.NewtonBodyGetVelocity(address(), velSeg);
			return velSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getAlpha() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment alphaSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.VEC3F);
			Newton_h.NewtonBodyGetAlpha(address(), alphaSeg);
			return alphaSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getAcceleration() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment accSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.VEC3F);
			Newton_h.NewtonBodyGetAcceleration(address(), accSeg);
			return accSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getForce() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment forceSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.VEC3F);
			Newton_h.NewtonBodyGetForce(address(), forceSeg);
			return forceSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getTorque() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment torqueSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.VEC3F);
			Newton_h.NewtonBodyGetTorque(address(), torqueSeg);
			return torqueSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getCenterOfMass() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment comSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.VEC3F);
			Newton_h.NewtonBodyGetCentreOfMass(address(), comSeg);
			return comSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getPointVelocity(float[] point) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment pointSeg = allocator.allocateArray(Newton_h.C_FLOAT, point);
			MemorySegment velSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.VEC3F);
			Newton_h.NewtonBodyGetPointVelocity(address(), pointSeg, velSeg);
			return velSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default void addImpulsePair(float[] linearImpulse, float[] angularImpulse, float timeStep) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment linearSegment = allocator.allocateArray(Newton_h.C_FLOAT, linearImpulse);
			MemorySegment angularSegment = allocator.allocateArray(Newton_h.C_FLOAT, angularImpulse);
			Newton_h.NewtonBodyApplyImpulsePair(address(), linearSegment, angularSegment, timeStep);
		}
	}
	
	default void addImpulse(float[] deltaVelocity, float[] point, float timestep) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment velSeg = allocator.allocateArray(Newton_h.C_FLOAT, deltaVelocity);
			MemorySegment pointSeg = allocator.allocateArray(Newton_h.C_FLOAT, point);
			Newton_h.NewtonBodyAddImpulse(address(), velSeg, pointSeg, timestep);
		}
	}
	
	default void addImpulseArray(int impulseCount, float[] impulseArray, float[] pointArray, float timestep) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			int strideInBytes = (int) Newton_h.C_FLOAT.byteSize() * 3;
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment impulseSegment = allocator.allocateArray(Newton_h.C_FLOAT, impulseArray);
			MemorySegment pointSegment = allocator.allocateArray(Newton_h.C_FLOAT, pointArray);
			Newton_h.NewtonBodyApplyImpulseArray(address(), impulseCount, strideInBytes, impulseSegment, pointSegment, timestep);
		}
	}
	
	default void integrateVelocity(float timestep) {
		Newton_h.NewtonBodyIntegrateVelocity(address(), timestep);
	}
	
	default float getLinearDamping() {
		return Newton_h.NewtonBodyGetLinearDamping(address());
	}
	
	default float[] getAngularDamping() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment vecSeg = allocator.allocateArray(Newton_h.C_FLOAT, Newton.VEC3F);
			Newton_h.NewtonBodyGetAngularDamping(address(), vecSeg);
			return vecSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default float[] getAABB() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment aabb = allocator.allocateArray(Newton_h.C_FLOAT, Newton.AABBF);
			Newton_h.NewtonBodyGetAABB(address(), 
					aabb.asSlice(0L, Newton_h.C_FLOAT.byteSize() * 3), 
					aabb.asSlice(Newton_h.C_FLOAT.byteSize() * 3));
			return aabb.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default void destroy() {
		Newton_h.NewtonDestroyBody(address());
	}
	
	public static NewtonBody wrap(MemoryAddress address) {
		int bodyType = Newton_h.NewtonBodyGetType(address);
		return switch (bodyType) {
			case 0 -> new NewtonDynamicBody(address);
			case 1 -> new NewtonKinematicBody(address);
			case 2 -> new NewtonAsymetricDynamicBody(address);
			default -> throw new RuntimeException("Error wrapping MemoryAddress");
		};
	}
}
