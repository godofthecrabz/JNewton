// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$48 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$48() {}
    public static final FunctionDescriptor NewtonWorldGetNextBody$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldGetNextBody$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldGetNextBody",
        constants$48.NewtonWorldGetNextBody$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialGetMaterialPairUserData$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialGetMaterialPairUserData$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetMaterialPairUserData",
        constants$48.NewtonMaterialGetMaterialPairUserData$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialGetContactFaceAttribute$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialGetContactFaceAttribute$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetContactFaceAttribute",
        constants$48.NewtonMaterialGetContactFaceAttribute$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialGetBodyCollidingShape$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialGetBodyCollidingShape$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetBodyCollidingShape",
        constants$48.NewtonMaterialGetBodyCollidingShape$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialGetContactNormalSpeed$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialGetContactNormalSpeed$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetContactNormalSpeed",
        constants$48.NewtonMaterialGetContactNormalSpeed$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialGetContactForce$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialGetContactForce$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetContactForce",
        constants$48.NewtonMaterialGetContactForce$FUNC
    );
}


