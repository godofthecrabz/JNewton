// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$14 {

    static final FunctionDescriptor NewtonOnContactGeneration$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonOnContactGeneration$MH = RuntimeHelper.downcallHandle(
        constants$14.NewtonOnContactGeneration$FUNC, false
    );
    static final FunctionDescriptor NewtonBodyIterator$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodyIterator$MH = RuntimeHelper.downcallHandle(
        constants$14.NewtonBodyIterator$FUNC, false
    );
    static final FunctionDescriptor NewtonJointIterator$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonJointIterator$MH = RuntimeHelper.downcallHandle(
        constants$14.NewtonJointIterator$FUNC, false
    );
}

