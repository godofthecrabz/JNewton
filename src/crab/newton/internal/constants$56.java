// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class constants$56 {

	public static final FunctionDescriptor NewtonCollisionCollideContinue$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
	public static final MethodHandle NewtonCollisionCollideContinue$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionCollideContinue",
        constants$56.NewtonCollisionCollideContinue$FUNC, false
    );
	public static final FunctionDescriptor NewtonCollisionSupportVertex$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonCollisionSupportVertex$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionSupportVertex",
        constants$56.NewtonCollisionSupportVertex$FUNC, false
    );
	public static final FunctionDescriptor NewtonCollisionRayCast$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonCollisionRayCast$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionRayCast",
        constants$56.NewtonCollisionRayCast$FUNC, false
    );
	public static final FunctionDescriptor NewtonCollisionCalculateAABB$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonCollisionCalculateAABB$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionCalculateAABB",
        constants$56.NewtonCollisionCalculateAABB$FUNC, false
    );
	public static final FunctionDescriptor NewtonCollisionForEachPolygonDo$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonCollisionForEachPolygonDo$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionForEachPolygonDo",
        constants$56.NewtonCollisionForEachPolygonDo$FUNC, false
    );
	public static final FunctionDescriptor NewtonCollisionAggregateCreate$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonCollisionAggregateCreate$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionAggregateCreate",
        constants$56.NewtonCollisionAggregateCreate$FUNC, false
    );
}

