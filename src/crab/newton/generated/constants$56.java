// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$56 {

    static final FunctionDescriptor NewtonCollisionCollideContinue$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
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
    static final MethodHandle NewtonCollisionCollideContinue$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionCollideContinue",
        constants$56.NewtonCollisionCollideContinue$FUNC, false
    );
    static final FunctionDescriptor NewtonCollisionSupportVertex$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCollisionSupportVertex$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionSupportVertex",
        constants$56.NewtonCollisionSupportVertex$FUNC, false
    );
    static final FunctionDescriptor NewtonCollisionRayCast$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCollisionRayCast$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionRayCast",
        constants$56.NewtonCollisionRayCast$FUNC, false
    );
    static final FunctionDescriptor NewtonCollisionCalculateAABB$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCollisionCalculateAABB$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionCalculateAABB",
        constants$56.NewtonCollisionCalculateAABB$FUNC, false
    );
    static final FunctionDescriptor NewtonCollisionForEachPolygonDo$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCollisionForEachPolygonDo$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionForEachPolygonDo",
        constants$56.NewtonCollisionForEachPolygonDo$FUNC, false
    );
    static final FunctionDescriptor NewtonCollisionAggregateCreate$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCollisionAggregateCreate$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionAggregateCreate",
        constants$56.NewtonCollisionAggregateCreate$FUNC, false
    );
}


