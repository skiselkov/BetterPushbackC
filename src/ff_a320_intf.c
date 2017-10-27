/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <acfutils/assert.h>
#include <acfutils/avl.h>
#include <acfutils/dr.h>
#include <acfutils/geom.h>
#include <acfutils/log.h>
#include <acfutils/helpers.h>
#include <acfutils/perf.h>
#include <acfutils/types.h>
#include <acfutils/thread.h>

#if	!IBM && !defined(__stdcall)
#define	__stdcall
#endif
#include <FF_A320/SharedValue.h>

#include "xplane.h"

static bool_t			inited = B_FALSE;
static SharedValuesInterface	svi;

static struct {
	bool_t		inited;

	int		acf;
	int		start;
	int		can_start;
	int		stop;
	int		can_stop;
	int		start_planner;
	int		can_start_planner;
	int		cab_camera;
	int		can_cab_camera;
} ids;

static struct {
	bool_t		start;
	bool_t		can_start;

	bool_t		stop;
	bool_t		can_stop;

	bool_t		start_planner;
	bool_t		can_start_planner;

	bool_t		cab_camera;
	bool_t		can_cab_camera;
} cmds;

static void __stdcall ff_a320_update(double step, void *tag);
static const char *type2str(unsigned int t);
static float cmd_dispatch(float elapsed, float elapsed2, int counter,
    void *refcon);

bool_t
ff_a320_intf_init(void)
{
	XPLMPluginID plugin;
	char author[64];
	dr_t author_dr;

	/*
	 * For some reason X-Plane 10 can send us the PLANE_LOADED message
	 * multiple times. WTF X-Plane...
	 */
	if (inited)
		return (B_FALSE);

	fdr_find(&author_dr, "sim/aircraft/view/acf_author");
	dr_gets(&author_dr, author, sizeof (author));
	if (strcmp(author, "FlightFactor") != 0)
		return (B_FALSE);

	plugin = XPLMFindPluginBySignature(XPLM_FF_SIGNATURE);
	if (plugin == XPLM_NO_PLUGIN_ID)
		return (B_FALSE);

	XPLMSendMessageToPlugin(plugin, XPLM_FF_MSG_GET_SHARED_INTERFACE, &svi);
	if (svi.DataAddUpdate == NULL) {
		logMsg("FF A320 interface init failure: func vector empty");
		return (B_FALSE);
	}

	svi.DataAddUpdate((SharedDataUpdateProc)ff_a320_update, NULL);
	XPLMRegisterFlightLoopCallback(cmd_dispatch, -1, NULL);

	memset(&ids, 0, sizeof (ids));
	memset(&cmds, 0, sizeof (cmds));

	inited = B_TRUE;

	return (B_TRUE);
}

void
ff_a320_intf_fini(void)
{
	if (!inited)
		return;

	XPLMUnregisterFlightLoopCallback(cmd_dispatch, NULL);
	if (svi.DataDelUpdate != NULL)
		svi.DataDelUpdate((SharedDataUpdateProc)ff_a320_update, NULL);

	inited = B_FALSE;
}

static inline int32_t gets32(int id) __attribute__((unused));
static inline int32_t
gets32(int id)
{
	int val;
	unsigned int type = svi.ValueType(id);
	ASSERT_MSG(type >= Value_Type_sint8 && type <= Value_Type_uint32,
	    "%s isn't an integer type, instead it is %s",
	    svi.ValueName(id), type2str(type));
	svi.ValueGet(id, &val);
	return (val);
}

static inline void sets32(int id, int32_t val) __attribute__((unused));
static inline void
sets32(int id, int32_t val)
{
	unsigned int type = svi.ValueType(id);
	ASSERT_MSG(type >= Value_Type_sint8 && type <= Value_Type_uint32,
	    "%s isn't an integer type, instead it is %s",
	    svi.ValueName(id), type2str(type));
	svi.ValueSet(id, &val);
}

static const char *
type2str(unsigned int t)
{
	switch (t) {
	case Value_Type_Deleted:
		return "deleted";
	case Value_Type_Object:
		return "object";
	case Value_Type_sint8:
		return "sint8";
	case Value_Type_uint8:
		return "uint8";
	case Value_Type_sint16:
		return "sint16";
	case Value_Type_uint16:
		return "uint16";
	case Value_Type_sint32:
		return "sint32";
	case Value_Type_uint32:
		return "uint32";
	case Value_Type_float32:
		return "float32";
	case Value_Type_float64:
		return "float64";
	case Value_Type_String:
		return "string";
	case Value_Type_Time:
		return "time";
	default:
		return "unknown";
	}
}

static void units2str(unsigned int units, char buf[32]) __attribute__((unused));

static void
units2str(unsigned int units, char buf[32])
{
	snprintf(buf, 32, "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s"
	    "%s%s%s%s%s%s%s%s%s%s%s%s",
	    (units & Value_Unit_Object) ? "O" : "",
	    (units & Value_Unit_Failure) ? "F" : "",
	    (units & Value_Unit_Button) ? "B" : "",
	    (units & Value_Unit_Ratio) ? "R" : "",
	    (units & Value_Unit_State) ? "s" : "",
	    (units & Value_Unit_Flags) ? "f" : "",
	    (units & Value_Unit_Ident) ? "I" : "",
	    (units & Value_Unit_Length) ? "M" : "",
	    (units & Value_Unit_Speed) ? "S" : "",
	    (units & Value_Unit_Accel) ? "^" : "",
	    (units & Value_Unit_Force) ? "N" : "",
	    (units & Value_Unit_Weight) ? "K" : "",
	    (units & Value_Unit_Angle) ? "D" : "",
	    (units & Value_Unit_AngularSpeed) ? "@" : "",
	    (units & Value_Unit_AngularAccel) ? "c" : "",
	    (units & Value_Unit_Temperature) ? "t" : "",
	    (units & Value_Unit_Pressure) ? "P" : "",
	    (units & Value_Unit_Flow) ? "L" : "",
	    (units & Value_Unit_Voltage) ? "V" : "",
	    (units & Value_Unit_Frequency) ? "H" : "",
	    (units & Value_Unit_Current) ? "A" : "",
	    (units & Value_Unit_Power) ? "W": "",
	    (units & Value_Unit_Density) ? "d" : "",
	    (units & Value_Unit_Volume) ? "v" : "",
	    (units & Value_Unit_Conduction) ? "S" : "",
	    (units & Value_Unit_Capacity) ? "C" : "",
	    (units & Value_Unit_Heat) ? "T" : "",
	    (units & Value_Unit_Position) ? "r" : "",
	    (units & Value_Unit_TimeDelta) ? "'" : "",
	    (units & Value_Unit_TimeStart) ? "`" : "",
	    (units & Value_Unit_Label) ? "9" : "");
}

static int
val_id(const char *name)
{
	return (svi.ValueIdByName(name));
}

static void
ff_a320_ids_init(void)
{
	if (ids.inited)
		return;

	ids.acf = val_id("Aircraft");

	ids.start = svi.ValueObjectNewValue(ids.acf, "BetterPushbackStart",
	    "Start pushback operation", &cmds.start, Value_Type_uint32,
	    Value_Flag_External, Value_Unit_State);
	ids.can_start = svi.ValueObjectNewValue(ids.acf,
	    "BetterPushbackCanStart",
	    "Can the pushback operation be started now?", &cmds.can_start,
	    Value_Type_uint32, Value_Flag_External, Value_Unit_State);

	ids.stop = svi.ValueObjectNewValue(ids.acf, "BetterPushbackStop",
	    "Stop pushback operation", &cmds.stop, Value_Type_uint32,
	    Value_Flag_External, Value_Unit_State);
	ids.can_stop = svi.ValueObjectNewValue(ids.acf,
	    "BetterPushbackCanStop",
	    "Can the pushback operation be stopped now?", &cmds.can_stop,
	    Value_Type_uint32, Value_Flag_External, Value_Unit_State);

	ids.start_planner = svi.ValueObjectNewValue(ids.acf,
	    "BetterPushbackStartPlanner", "Start pushback path planner",
	    &cmds.start_planner, Value_Type_uint32, Value_Flag_External,
	    Value_Unit_State);
	ids.can_start_planner = svi.ValueObjectNewValue(ids.acf,
	    "BetterPushbackCanStartPlanner",
	    "Can the pushback planner be started now?", &cmds.can_start_planner,
	    Value_Type_uint32, Value_Flag_External, Value_Unit_State);

	ids.cab_camera = svi.ValueObjectNewValue(ids.acf,
	    "BetterPushbackCabCamera", "Switch to view from pushback tug cab",
	    &cmds.cab_camera, Value_Type_uint32, Value_Flag_External,
	    Value_Unit_State);
	ids.can_cab_camera = svi.ValueObjectNewValue(ids.acf,
	    "BetterPushbackCanCabCamera",
	    "Can the view be switched to the tug cab now?",
	    &cmds.can_cab_camera, Value_Type_uint32, Value_Flag_External,
	    Value_Unit_State);

	ids.inited = B_TRUE;
}

static void __stdcall
ff_a320_update(double step, void *tag)
{
	UNUSED(step);
	UNUSED(tag);

	VERIFY(svi.ValueIdByName != NULL);
	VERIFY(svi.ValueGet != NULL);
	VERIFY(svi.ValueType != NULL);
	VERIFY(svi.ValueName != NULL);

	ff_a320_ids_init();
}

static float
cmd_dispatch(float elapsed, float elapsed2, int counter, void *refcon)
{
	UNUSED(elapsed);
	UNUSED(elapsed2);
	UNUSED(counter);
	UNUSED(refcon);

	if (cmds.start) {
		XPLMCommandOnce(XPLMFindCommand("BetterPushback/start"));
		cmds.start = B_FALSE;
	}
	cmds.can_start = !bp_started;

	if (cmds.stop) {
		XPLMCommandOnce(XPLMFindCommand("BetterPushback/stop"));
		cmds.stop = B_FALSE;
	}
	cmds.can_stop = bp_started;

	if (cmds.start_planner) {
		XPLMCommandOnce(XPLMFindCommand(
		    "BetterPushback/start_planner"));
		cmds.start_planner = B_FALSE;
	}
	cmds.can_start_planner = !bp_started;

	if (cmds.cab_camera) {
		XPLMCommandOnce(XPLMFindCommand("BetterPushback/cab_camera"));
		cmds.cab_camera = B_FALSE;
	}
	cmds.can_cab_camera = bp_started;

	return (-1);
}
