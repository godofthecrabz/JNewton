// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$43 {

    static final FunctionDescriptor NewtonCompoundCollisionRemoveSubCollisionByIndex$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonCompoundCollisionRemoveSubCollisionByIndex$MH = RuntimeHelper.downcallHandle(
        "NewtonCompoundCollisionRemoveSubCollisionByIndex",
        constants$43.NewtonCompoundCollisionRemoveSubCollisionByIndex$FUNC, false
    );
    static final FunctionDescriptor NewtonCompoundCollisionSetSubCollisionMatrix$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCompoundCollisionSetSubCollisionMatrix$MH = RuntimeHelper.downcallHandle(
        "NewtonCompoundCollisionSetSubCollisionMatrix",
        constants$43.NewtonCompoundCollisionSetSubCollisionMatrix$FUNC, false
    );
    static final FunctionDescriptor NewtonCompoundCollisionEndAddRemove$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCompoundCollisionEndAddRemove$MH = RuntimeHelper.downcallHandle(
        "NewtonCompoundCollisionEndAddRemove",
        constants$43.NewtonCompoundCollisionEndAddRemove$FUNC, false
    );
    static final FunctionDescriptor NewtonCompoundCollisionGetFirstNode$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCompoundCollisionGetFirstNode$MH = RuntimeHelper.downcallHandle(
        "NewtonCompoundCollisionGetFirstNode",
        constants$43.NewtonCompoundCollisionGetFirstNode$FUNC, false
    );
    static final FunctionDescriptor NewtonCompoundCollisionGetNextNode$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCompoundCollisionGetNextNode$MH = RuntimeHelper.downcallHandle(
        "NewtonCompoundCollisionGetNextNode",
        constants$43.NewtonCompoundCollisionGetNextNode$FUNC, false
    );
    static final FunctionDescriptor NewtonCompoundCollisionGetNodeByIndex$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonCompoundCollisionGetNodeByIndex$MH = RuntimeHelper.downcallHandle(
        "NewtonCompoundCollisionGetNodeByIndex",
        constants$43.NewtonCompoundCollisionGetNodeByIndex$FUNC, false
    );
}

