package crab.newton;

import java.lang.foreign.MemorySegment;

public record NewtonJoint(MemorySegment address) {
	
	public NewtonJoint {}
	
	public float getContactPruningTolerance() {
		return Newton.NewtonMaterialGetContactPruningTolerance(address);
	}
	
	public void setContactPruningTolerance(float tolerance) {
		Newton.NewtonMaterialSetContactPruningTolerance(address, tolerance);
	}
}
