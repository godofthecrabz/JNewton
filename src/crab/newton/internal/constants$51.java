// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$51 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$51() {}
    public static final FunctionDescriptor NewtonMaterialSetContactNormalAcceleration$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonMaterialSetContactNormalAcceleration$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetContactNormalAcceleration",
        constants$51.NewtonMaterialSetContactNormalAcceleration$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialSetContactNormalDirection$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialSetContactNormalDirection$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetContactNormalDirection",
        constants$51.NewtonMaterialSetContactNormalDirection$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialSetContactPosition$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialSetContactPosition$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetContactPosition",
        constants$51.NewtonMaterialSetContactPosition$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialSetContactTangentFriction$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonMaterialSetContactTangentFriction$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetContactTangentFriction",
        constants$51.NewtonMaterialSetContactTangentFriction$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialSetContactTangentAcceleration$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonMaterialSetContactTangentAcceleration$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetContactTangentAcceleration",
        constants$51.NewtonMaterialSetContactTangentAcceleration$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialContactRotateTangentDirections$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialContactRotateTangentDirections$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialContactRotateTangentDirections",
        constants$51.NewtonMaterialContactRotateTangentDirections$FUNC
    );
}


