// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$55 {

    static final FunctionDescriptor NewtonCollisionGetSkinThickness$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCollisionGetSkinThickness$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionGetSkinThickness",
        constants$55.NewtonCollisionGetSkinThickness$FUNC, false
    );
    static final FunctionDescriptor NewtonCollisionSetSkinThickness$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    static final MethodHandle NewtonCollisionSetSkinThickness$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionSetSkinThickness",
        constants$55.NewtonCollisionSetSkinThickness$FUNC, false
    );
    static final FunctionDescriptor NewtonCollisionIntersectionTest$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonCollisionIntersectionTest$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionIntersectionTest",
        constants$55.NewtonCollisionIntersectionTest$FUNC, false
    );
    static final FunctionDescriptor NewtonCollisionPointDistance$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonCollisionPointDistance$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionPointDistance",
        constants$55.NewtonCollisionPointDistance$FUNC, false
    );
    static final FunctionDescriptor NewtonCollisionClosestPoint$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
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
    static final MethodHandle NewtonCollisionClosestPoint$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionClosestPoint",
        constants$55.NewtonCollisionClosestPoint$FUNC, false
    );
    static final FunctionDescriptor NewtonCollisionCollide$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
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
    static final MethodHandle NewtonCollisionCollide$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionCollide",
        constants$55.NewtonCollisionCollide$FUNC, false
    );
}


