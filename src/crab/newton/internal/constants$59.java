// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class constants$59 {

	public static final FunctionDescriptor NewtonBodyGetSimulationState$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonBodyGetSimulationState$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetSimulationState",
        constants$59.NewtonBodyGetSimulationState$FUNC, false
    );
	public static final FunctionDescriptor NewtonBodySetSimulationState$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
	public static final MethodHandle NewtonBodySetSimulationState$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetSimulationState",
        constants$59.NewtonBodySetSimulationState$FUNC, false
    );
	public static final FunctionDescriptor NewtonBodyGetType$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonBodyGetType$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetType",
        constants$59.NewtonBodyGetType$FUNC, false
    );
	public static final FunctionDescriptor NewtonBodyGetCollidable$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonBodyGetCollidable$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetCollidable",
        constants$59.NewtonBodyGetCollidable$FUNC, false
    );
	public static final FunctionDescriptor NewtonBodySetCollidable$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
	public static final MethodHandle NewtonBodySetCollidable$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetCollidable",
        constants$59.NewtonBodySetCollidable$FUNC, false
    );
	public static final FunctionDescriptor NewtonBodyAddForce$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonBodyAddForce$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyAddForce",
        constants$59.NewtonBodyAddForce$FUNC, false
    );
}


