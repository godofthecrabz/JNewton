// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class constants$32 {

	public static final FunctionDescriptor NewtonMaterialCreateGroupID$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMaterialCreateGroupID$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialCreateGroupID",
        constants$32.NewtonMaterialCreateGroupID$FUNC, false
    );
	public static final FunctionDescriptor NewtonMaterialGetDefaultGroupID$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMaterialGetDefaultGroupID$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetDefaultGroupID",
        constants$32.NewtonMaterialGetDefaultGroupID$FUNC, false
    );
	public static final FunctionDescriptor NewtonMaterialDestroyAllGroupID$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMaterialDestroyAllGroupID$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialDestroyAllGroupID",
        constants$32.NewtonMaterialDestroyAllGroupID$FUNC, false
    );
	public static final FunctionDescriptor NewtonMaterialGetUserData$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
	public static final MethodHandle NewtonMaterialGetUserData$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetUserData",
        constants$32.NewtonMaterialGetUserData$FUNC, false
    );
	public static final FunctionDescriptor NewtonMaterialSetSurfaceThickness$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
	public static final MethodHandle NewtonMaterialSetSurfaceThickness$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetSurfaceThickness",
        constants$32.NewtonMaterialSetSurfaceThickness$FUNC, false
    );
	public static final FunctionDescriptor NewtonMaterialSetCallbackUserData$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMaterialSetCallbackUserData$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetCallbackUserData",
        constants$32.NewtonMaterialSetCallbackUserData$FUNC, false
    );
}


