// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$60 {

    static final FunctionDescriptor NewtonBodyAddTorque$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodyAddTorque$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyAddTorque",
        constants$60.NewtonBodyAddTorque$FUNC, false
    );
    static final FunctionDescriptor NewtonBodySetCentreOfMass$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodySetCentreOfMass$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetCentreOfMass",
        constants$60.NewtonBodySetCentreOfMass$FUNC, false
    );
    static final FunctionDescriptor NewtonBodySetMassMatrix$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    static final MethodHandle NewtonBodySetMassMatrix$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetMassMatrix",
        constants$60.NewtonBodySetMassMatrix$FUNC, false
    );
    static final FunctionDescriptor NewtonBodySetFullMassMatrix$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodySetFullMassMatrix$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetFullMassMatrix",
        constants$60.NewtonBodySetFullMassMatrix$FUNC, false
    );
    static final FunctionDescriptor NewtonBodySetMassProperties$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodySetMassProperties$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetMassProperties",
        constants$60.NewtonBodySetMassProperties$FUNC, false
    );
    static final FunctionDescriptor NewtonBodySetMatrix$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodySetMatrix$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetMatrix",
        constants$60.NewtonBodySetMatrix$FUNC, false
    );
}


