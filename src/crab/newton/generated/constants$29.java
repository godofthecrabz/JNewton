// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$29 {

    static final FunctionDescriptor NewtonWorldListenerSetPostUpdateCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonWorldListenerSetPostUpdateCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldListenerSetPostUpdateCallback",
        constants$29.NewtonWorldListenerSetPostUpdateCallback$FUNC, false
    );
    static final FunctionDescriptor NewtonWorldListenerSetDestructorCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonWorldListenerSetDestructorCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldListenerSetDestructorCallback",
        constants$29.NewtonWorldListenerSetDestructorCallback$FUNC, false
    );
    static final FunctionDescriptor NewtonWorldListenerSetBodyDestroyCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonWorldListenerSetBodyDestroyCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldListenerSetBodyDestroyCallback",
        constants$29.NewtonWorldListenerSetBodyDestroyCallback$FUNC, false
    );
    static final FunctionDescriptor NewtonWorldListenerDebug$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonWorldListenerDebug$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldListenerDebug",
        constants$29.NewtonWorldListenerDebug$FUNC, false
    );
    static final FunctionDescriptor NewtonWorldGetListenerUserData$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonWorldGetListenerUserData$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldGetListenerUserData",
        constants$29.NewtonWorldGetListenerUserData$FUNC, false
    );
    static final FunctionDescriptor NewtonWorldListenerGetBodyDestroyCallback$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonWorldListenerGetBodyDestroyCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldListenerGetBodyDestroyCallback",
        constants$29.NewtonWorldListenerGetBodyDestroyCallback$FUNC, false
    );
}


