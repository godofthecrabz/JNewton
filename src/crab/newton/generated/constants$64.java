// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$64 {

    static final FunctionDescriptor NewtonBodySetAutoSleep$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonBodySetAutoSleep$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetAutoSleep",
        constants$64.NewtonBodySetAutoSleep$FUNC, false
    );
    static final FunctionDescriptor NewtonBodyGetFreezeState$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodyGetFreezeState$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetFreezeState",
        constants$64.NewtonBodyGetFreezeState$FUNC, false
    );
    static final FunctionDescriptor NewtonBodySetFreezeState$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonBodySetFreezeState$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetFreezeState",
        constants$64.NewtonBodySetFreezeState$FUNC, false
    );
    static final FunctionDescriptor NewtonBodyGetGyroscopicTorque$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodyGetGyroscopicTorque$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetGyroscopicTorque",
        constants$64.NewtonBodyGetGyroscopicTorque$FUNC, false
    );
    static final FunctionDescriptor NewtonBodySetGyroscopicTorque$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonBodySetGyroscopicTorque$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetGyroscopicTorque",
        constants$64.NewtonBodySetGyroscopicTorque$FUNC, false
    );
    static final FunctionDescriptor NewtonBodySetDestructorCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodySetDestructorCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetDestructorCallback",
        constants$64.NewtonBodySetDestructorCallback$FUNC, false
    );
}


