// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$69 {

    static final FunctionDescriptor NewtonBodyGetAcceleration$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodyGetAcceleration$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetAcceleration",
        constants$69.NewtonBodyGetAcceleration$FUNC, false
    );
    static final FunctionDescriptor NewtonBodyGetForce$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodyGetForce$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetForce",
        constants$69.NewtonBodyGetForce$FUNC, false
    );
    static final FunctionDescriptor NewtonBodyGetTorque$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodyGetTorque$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetTorque",
        constants$69.NewtonBodyGetTorque$FUNC, false
    );
    static final FunctionDescriptor NewtonBodyGetCentreOfMass$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodyGetCentreOfMass$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetCentreOfMass",
        constants$69.NewtonBodyGetCentreOfMass$FUNC, false
    );
    static final FunctionDescriptor NewtonBodyGetPointVelocity$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodyGetPointVelocity$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetPointVelocity",
        constants$69.NewtonBodyGetPointVelocity$FUNC, false
    );
    static final FunctionDescriptor NewtonBodyApplyImpulsePair$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    static final MethodHandle NewtonBodyApplyImpulsePair$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyApplyImpulsePair",
        constants$69.NewtonBodyApplyImpulsePair$FUNC, false
    );
}


