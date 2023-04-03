package crab.newton;

import java.lang.foreign.*;

public class NewtonMaterial {
	
	protected final MemorySegment address;
	
	protected NewtonMaterial(MemorySegment address) {
		this.address = address;
	}
	
	public MemorySegment getMaterialPairUserData() {
		return Newton.NewtonMaterialGetMaterialPairUserData(address);
	}
	
	public int getContactFaceAttribute() {
		return Newton.NewtonMaterialGetContactFaceAttribute(address);
	}
	
	public NewtonCollision getBodyCollidingShape(NewtonBody body) {
		MemorySegment collisionPtr = Newton.NewtonMaterialGetBodyCollidingShape(address, body.address);
		return NewtonCollision.wrap(collisionPtr);
	}
	
	public float getContactNormalSpeed() {
		return Newton.NewtonMaterialGetContactNormalSpeed(address);
	}

	public void getContactForce(NewtonBody body, MemorySegment force) {
		Newton.NewtonMaterialGetContactForce(address, body.address, force);
	}
	
	public float[] getContactForce(NewtonBody body) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment forceSegment = arena.allocate(Newton.VEC3F);
			Newton.NewtonMaterialGetContactForce(address, body.address, forceSegment);
			return forceSegment.toArray(Newton.C_FLOAT);
		}
	}

	public void getContactPositionAndNormal(NewtonBody body, MemorySegment positionNormal) {
		Newton.NewtonMaterialGetContactPositionAndNormal(address, body.address,
				positionNormal.asSlice(0L, Newton.VEC3F.byteSize()),
				positionNormal.asSlice(Newton.VEC3F.byteSize()));
	}
	
	public float[] getContactPositionAndNormal(NewtonBody body) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment positionNormal = arena.allocate(Newton.AABBF);
			Newton.NewtonMaterialGetContactPositionAndNormal(address, body.address,
					positionNormal.asSlice(0L, Newton.VEC3F.byteSize()),
					positionNormal.asSlice(Newton.VEC3F.byteSize()));
			return positionNormal.toArray(Newton.C_FLOAT);
		}
	}

	public void getContactTangentDirections(NewtonBody body, MemorySegment direction) {
		Newton.NewtonMaterialGetContactTangentDirections(address, body.address,
				direction.asSlice(0L, Newton.VEC3F.byteSize()),
				direction.asSlice(Newton.VEC3F.byteSize()));
	}
	
	public float[] getContactTangentDirections(NewtonBody body) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment dir = arena.allocate(Newton.AABBF);
			Newton.NewtonMaterialGetContactTangentDirections(address, body.address,
					dir.asSlice(0L, Newton.C_FLOAT.byteSize() * 3), 
					dir.asSlice(Newton.C_FLOAT.byteSize() * 3));
			return dir.toArray(Newton.C_FLOAT);
		}
	}
	
	public float getContactTangentSpeed(int index) {
		return Newton.NewtonMaterialGetContactTangentSpeed(address, index);
	}
	
	public float getContactMaxNormalImpact() {
		return Newton.NewtonMaterialGetContactMaxNormalImpact(address);
	}
	
	public float getContactMaxTangentImpact(int index) {
		return Newton.NewtonMaterialGetContactMaxTangentImpact(address, index);
	}
	
	public float getContactPenetration() {
		return Newton.NewtonMaterialGetContactPenetration(address);
	}
	
	public void setAsSoftContact(float relaxation) {
		Newton.NewtonMaterialSetAsSoftContact(address, relaxation);
	}
	
	public void setContactSoftness(float softness) {
		Newton.NewtonMaterialSetContactSoftness(address, softness);
	}
	
	public void setContactThickness(float thickness) {
		Newton.NewtonMaterialSetContactThickness(address, thickness);
	}
	
	public void setContactElasticity(float elasticity) {
		Newton.NewtonMaterialSetContactElasticity(address, elasticity);
	}
	
	public void setContactFrictionState(int state, int index) {
		Newton.NewtonMaterialSetContactFrictionState(address, state, index);
	}
	
	public void setContactFrictionCoefficient(float staticFrictionCoef, float kineticFrictionCoef, int index) {
		Newton.NewtonMaterialSetContactFrictionCoef(address, staticFrictionCoef, kineticFrictionCoef, index);
	}
	
	public void setContactNormalAcceleration(float accel) {
		Newton.NewtonMaterialSetContactNormalAcceleration(address, accel);
	}

	public void setContactNormalDirection(MemorySegment direction) {
		Newton.NewtonMaterialSetContactNormalDirection(address, direction);
	}
	
	public void setContactNormalDirection(float[] direction) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment dirSegment = arena.allocateArray(Newton.C_FLOAT, direction);
			Newton.NewtonMaterialSetContactNormalDirection(address, dirSegment);
		}
	}

	public void setContactPosition(MemorySegment position) {
		Newton.NewtonMaterialSetContactPosition(address, position);
	}
	
	public void setContactPosition(float[] position) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment positionSegment = arena.allocateArray(Newton.C_FLOAT, position);
			Newton.NewtonMaterialSetContactPosition(address, positionSegment);
		}
	}
	
	public void setContactTangentFriction(float friction, int index) {
		Newton.NewtonMaterialSetContactTangentFriction(address, friction, index);
	}
	
	public void setContactTangentAcceleration(float accel, int index) {
		Newton.NewtonMaterialSetContactTangentAcceleration(address, accel, index);
	}

	public void rotateContactTangentDirections(MemorySegment alignVector) {
		Newton.NewtonMaterialContactRotateTangentDirections(address, alignVector);
	}
	
	public void rotateContactTangentDirections(float[] alignVector) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment alignVecSegment = arena.allocateArray(Newton.C_FLOAT, alignVector);
			Newton.NewtonMaterialContactRotateTangentDirections(address, alignVecSegment);
		}
	}
}
