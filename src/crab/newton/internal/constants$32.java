// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$32 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$32() {}
    public static final FunctionDescriptor NewtonAtomicSwap$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonAtomicSwap$MH = RuntimeHelper.downcallHandle(
        "NewtonAtomicSwap",
        constants$32.NewtonAtomicSwap$FUNC
    );
    public static final FunctionDescriptor NewtonYield$FUNC = FunctionDescriptor.ofVoid();
    public static final MethodHandle NewtonYield$MH = RuntimeHelper.downcallHandleVariadic(
        "NewtonYield",
        constants$32.NewtonYield$FUNC
    );
    public static final FunctionDescriptor NewtonSetIslandUpdateEvent$islandUpdate$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final FunctionDescriptor NewtonSetIslandUpdateEvent$islandUpdate_UP$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonSetIslandUpdateEvent$islandUpdate_UP$MH = RuntimeHelper.upcallHandle(NewtonSetIslandUpdateEvent$islandUpdate.class, "apply", constants$32.NewtonSetIslandUpdateEvent$islandUpdate_UP$FUNC);
    public static final FunctionDescriptor NewtonSetIslandUpdateEvent$islandUpdate_DOWN$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonSetIslandUpdateEvent$islandUpdate_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$32.NewtonSetIslandUpdateEvent$islandUpdate_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonSetIslandUpdateEvent$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonSetIslandUpdateEvent$MH = RuntimeHelper.downcallHandle(
        "NewtonSetIslandUpdateEvent",
        constants$32.NewtonSetIslandUpdateEvent$FUNC
    );
    public static final FunctionDescriptor NewtonWorldForEachJointDo$callback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonWorldForEachJointDo$callback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldForEachJointDo$callback_UP$MH = RuntimeHelper.upcallHandle(NewtonWorldForEachJointDo$callback.class, "apply", constants$32.NewtonWorldForEachJointDo$callback_UP$FUNC);
}


