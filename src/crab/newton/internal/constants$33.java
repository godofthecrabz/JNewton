// Generated by jextract
package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public class constants$33 {

    public static final FunctionDescriptor NewtonMaterialSetContactGenerationCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialSetContactGenerationCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetContactGenerationCallback",
        constants$33.NewtonMaterialSetContactGenerationCallback$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialSetCompoundCollisionCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialSetCompoundCollisionCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetCompoundCollisionCallback",
        constants$33.NewtonMaterialSetCompoundCollisionCallback$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialSetCollisionCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialSetCollisionCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetCollisionCallback",
        constants$33.NewtonMaterialSetCollisionCallback$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialSetDefaultSoftness$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonMaterialSetDefaultSoftness$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetDefaultSoftness",
        constants$33.NewtonMaterialSetDefaultSoftness$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialSetDefaultElasticity$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonMaterialSetDefaultElasticity$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetDefaultElasticity",
        constants$33.NewtonMaterialSetDefaultElasticity$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialSetDefaultCollidable$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonMaterialSetDefaultCollidable$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetDefaultCollidable",
        constants$33.NewtonMaterialSetDefaultCollidable$FUNC
    );
}


