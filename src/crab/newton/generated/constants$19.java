// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$19 {

    static final FunctionDescriptor NewtonGetMemoryUsed$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT);
    static final MethodHandle NewtonGetMemoryUsed$MH = RuntimeHelper.downcallHandle(
        "NewtonGetMemoryUsed",
        constants$19.NewtonGetMemoryUsed$FUNC, true
    );
    static final FunctionDescriptor NewtonSetMemorySystem$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonSetMemorySystem$MH = RuntimeHelper.downcallHandle(
        "NewtonSetMemorySystem",
        constants$19.NewtonSetMemorySystem$FUNC, false
    );
    static final FunctionDescriptor NewtonCreate$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT);
    static final MethodHandle NewtonCreate$MH = RuntimeHelper.downcallHandle(
        "NewtonCreate",
        constants$19.NewtonCreate$FUNC, true
    );
    static final FunctionDescriptor NewtonDestroy$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonDestroy$MH = RuntimeHelper.downcallHandle(
        "NewtonDestroy",
        constants$19.NewtonDestroy$FUNC, false
    );
    static final FunctionDescriptor NewtonDestroyAllBodies$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonDestroyAllBodies$MH = RuntimeHelper.downcallHandle(
        "NewtonDestroyAllBodies",
        constants$19.NewtonDestroyAllBodies$FUNC, false
    );
    static final FunctionDescriptor NewtonGetPostUpdateCallback$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonGetPostUpdateCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonGetPostUpdateCallback",
        constants$19.NewtonGetPostUpdateCallback$FUNC, false
    );
}


