// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class constants$37 {

	public static final FunctionDescriptor NewtonMaterialSetAsSoftContact$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
	public static final MethodHandle NewtonMaterialSetAsSoftContact$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetAsSoftContact",
        constants$37.NewtonMaterialSetAsSoftContact$FUNC, false
    );
	public static final FunctionDescriptor NewtonMaterialSetContactSoftness$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
	public static final MethodHandle NewtonMaterialSetContactSoftness$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetContactSoftness",
        constants$37.NewtonMaterialSetContactSoftness$FUNC, false
    );
	public static final FunctionDescriptor NewtonMaterialSetContactThickness$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
	public static final MethodHandle NewtonMaterialSetContactThickness$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetContactThickness",
        constants$37.NewtonMaterialSetContactThickness$FUNC, false
    );
	public static final FunctionDescriptor NewtonMaterialSetContactElasticity$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
	public static final MethodHandle NewtonMaterialSetContactElasticity$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetContactElasticity",
        constants$37.NewtonMaterialSetContactElasticity$FUNC, false
    );
	public static final FunctionDescriptor NewtonMaterialSetContactFrictionState$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
	public static final MethodHandle NewtonMaterialSetContactFrictionState$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetContactFrictionState",
        constants$37.NewtonMaterialSetContactFrictionState$FUNC, false
    );
	public static final FunctionDescriptor NewtonMaterialSetContactFrictionCoef$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
	public static final MethodHandle NewtonMaterialSetContactFrictionCoef$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetContactFrictionCoef",
        constants$37.NewtonMaterialSetContactFrictionCoef$FUNC, false
    );
}


