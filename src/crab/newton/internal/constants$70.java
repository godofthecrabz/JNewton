// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class constants$70 {

	public static final FunctionDescriptor NewtonBodyAddImpulse$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
	public static final MethodHandle NewtonBodyAddImpulse$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyAddImpulse",
        constants$70.NewtonBodyAddImpulse$FUNC, false
    );
	public static final FunctionDescriptor NewtonBodyApplyImpulseArray$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
	public static final MethodHandle NewtonBodyApplyImpulseArray$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyApplyImpulseArray",
        constants$70.NewtonBodyApplyImpulseArray$FUNC, false
    );
	public static final FunctionDescriptor NewtonBodyIntegrateVelocity$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
	public static final MethodHandle NewtonBodyIntegrateVelocity$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyIntegrateVelocity",
        constants$70.NewtonBodyIntegrateVelocity$FUNC, false
    );
	public static final FunctionDescriptor NewtonBodyGetLinearDamping$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonBodyGetLinearDamping$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetLinearDamping",
        constants$70.NewtonBodyGetLinearDamping$FUNC, false
    );
	public static final FunctionDescriptor NewtonBodyGetAngularDamping$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonBodyGetAngularDamping$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetAngularDamping",
        constants$70.NewtonBodyGetAngularDamping$FUNC, false
    );
	public static final FunctionDescriptor NewtonBodyGetAABB$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonBodyGetAABB$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetAABB",
        constants$70.NewtonBodyGetAABB$FUNC, false
    );
}


