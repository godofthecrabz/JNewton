package crab.newton;

import crab.newton.internal.Newton_h;
import java.lang.foreign.*;

public class NewtonJoint {
	
	protected final MemoryAddress address;
	
	protected NewtonJoint(MemoryAddress address) {
		this.address = address;
	}
	
	public float getContactPruningTolerance() {
		return Newton_h.NewtonMaterialGetContactPruningTolerance(address);
	}
	
	public void setContactPruningTolerance(float tolerance) {
		Newton_h.NewtonMaterialSetContactPruningTolerance(address, tolerance);
	}
}
