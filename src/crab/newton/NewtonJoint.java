package crab.newton;

import java.lang.foreign.MemorySegment;

public class NewtonJoint {
	
	protected final MemorySegment address;
	
	protected NewtonJoint(MemorySegment address) {
		this.address = address;
	}
	
	public float getContactPruningTolerance() {
		return Newton.NewtonMaterialGetContactPruningTolerance(address);
	}
	
	public void setContactPruningTolerance(float tolerance) {
		Newton.NewtonMaterialSetContactPruningTolerance(address, tolerance);
	}
}
