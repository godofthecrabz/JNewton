// Generated by jextract
package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public class constants$69 {

    public static final FunctionDescriptor NewtonBodyGetAcceleration$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetAcceleration$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetAcceleration",
        constants$69.NewtonBodyGetAcceleration$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetForce$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetForce$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetForce",
        constants$69.NewtonBodyGetForce$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetTorque$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetTorque$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetTorque",
        constants$69.NewtonBodyGetTorque$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetCentreOfMass$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetCentreOfMass$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetCentreOfMass",
        constants$69.NewtonBodyGetCentreOfMass$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetPointVelocity$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetPointVelocity$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetPointVelocity",
        constants$69.NewtonBodyGetPointVelocity$FUNC
    );
    public static final FunctionDescriptor NewtonBodyApplyImpulsePair$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonBodyApplyImpulsePair$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyApplyImpulsePair",
        constants$69.NewtonBodyApplyImpulsePair$FUNC
    );
}


