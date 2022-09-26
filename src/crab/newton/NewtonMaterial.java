package crab.newton;

import crab.newton.internal.Newton_h;
import java.lang.foreign.*;

public class NewtonMaterial {
	
	protected final MemoryAddress address;
	
	protected NewtonMaterial(MemoryAddress address) {
		this.address = address;
	}
	
	public MemoryAddress getMaterialPairUserData() {
		return Newton_h.NewtonMaterialGetMaterialPairUserData(address);
	}
	
	public int getContactFaceAttribute() {
		return Newton_h.NewtonMaterialGetContactFaceAttribute(address);
	}
	
	public NewtonCollision getBodyCollidingShape(NewtonBody body) {
		MemoryAddress collisionPtr = Newton_h.NewtonMaterialGetBodyCollidingShape(address, body.address());
		return NewtonCollision.wrap(collisionPtr);
	}
	
	public float getContactNormalSpeed() {
		return Newton_h.NewtonMaterialGetContactNormalSpeed(address);
	}
	
	public float[] getContactForce(NewtonBody body) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment forceSegment = session.allocateArray(Newton_h.C_FLOAT, Newton.VEC3F);
			Newton_h.NewtonMaterialGetContactForce(address, body.address(), forceSegment);
			return forceSegment.toArray(Newton_h.C_FLOAT);
		}
	}
	
	public float[] getContactPositionAndNormal(NewtonBody body) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment positionNormal = session.allocateArray(Newton_h.C_FLOAT, Newton.AABBF);
			Newton_h.NewtonMaterialGetContactPositionAndNormal(address, body.address(), 
					positionNormal.asSlice(0L, Newton_h.C_FLOAT.byteSize() * 3), 
					positionNormal.asSlice(Newton_h.C_FLOAT.byteSize() * 3));
			return positionNormal.toArray(Newton_h.C_FLOAT);
		}
	}
	
	public float[] getContactTangentDirections(NewtonBody body) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment dir = session.allocateArray(Newton_h.C_FLOAT, Newton.AABBF);
			Newton_h.NewtonMaterialGetContactTangentDirections(address, body.address(), 
					dir.asSlice(0L, Newton_h.C_FLOAT.byteSize() * 3), 
					dir.asSlice(Newton_h.C_FLOAT.byteSize() * 3));
			return dir.toArray(Newton_h.C_FLOAT);
		}
	}
	
	public float getContactTangentSpeed(int index) {
		return Newton_h.NewtonMaterialGetContactTangentSpeed(address, index);
	}
	
	public float getContactMaxNormalImpact() {
		return Newton_h.NewtonMaterialGetContactMaxNormalImpact(address);
	}
	
	public float getContactMaxTangentImpact(int index) {
		return Newton_h.NewtonMaterialGetContactMaxTangentImpact(address, index);
	}
	
	public float getContactPenetration() {
		return Newton_h.NewtonMaterialGetContactPenetration(address);
	}
	
	public void setAsSoftContact(float relaxation) {
		Newton_h.NewtonMaterialSetAsSoftContact(address, relaxation);
	}
	
	public void setContactSoftness(float softness) {
		Newton_h.NewtonMaterialSetContactSoftness(address, softness);
	}
	
	public void setContactThickness(float thickness) {
		Newton_h.NewtonMaterialSetContactThickness(address, thickness);
	}
	
	public void setContactElasticity(float elasticity) {
		Newton_h.NewtonMaterialSetContactElasticity(address, elasticity);
	}
	
	public void setContactFrictionState(int state, int index) {
		Newton_h.NewtonMaterialSetContactFrictionState(address, state, index);
	}
	
	public void setContactFrictionCoefficient(float staticFrictionCoef, float kineticFrictionCoef, int index) {
		Newton_h.NewtonMaterialSetContactFrictionCoef(address, staticFrictionCoef, kineticFrictionCoef, index);
	}
	
	public void setContactNormalAcceleration(float accel) {
		Newton_h.NewtonMaterialSetContactNormalAcceleration(address, accel);
	}
	
	public void setContactNormalDirection(float[] direction) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment dirSegment = session.allocateArray(Newton_h.C_FLOAT, direction);
			Newton_h.NewtonMaterialSetContactNormalDirection(address, dirSegment);
		}
	}
	
	public void setContactPosition(float[] position) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment positionSegment = session.allocateArray(Newton_h.C_FLOAT, position);
			Newton_h.NewtonMaterialSetContactPosition(address, positionSegment);
		}
	}
	
	public void setContactTangentFriction(float friction, int index) {
		Newton_h.NewtonMaterialSetContactTangentFriction(address, friction, index);
	}
	
	public void setContactTangentAcceleration(float accel, int index) {
		Newton_h.NewtonMaterialSetContactTangentAcceleration(address, accel, index);
	}
	
	public void rotateContactTangentDirections(float[] alignVector) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment alignVecSegment = session.allocateArray(Newton_h.C_FLOAT, alignVector);
			Newton_h.NewtonMaterialContactRotateTangentDirections(address, alignVecSegment);
		}
	}
}
