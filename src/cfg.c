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

#include <string.h>
#include <errno.h>

#include <XPWidgets.h>
#include <XPStandardWidgets.h>

#include <acfutils/assert.h>
#include <acfutils/helpers.h>
#include <acfutils/intl.h>
#include <acfutils/widget.h>

#include "cfg.h"
#include "msg.h"
#include "xplane.h"

#define	CONF_FILENAME	"BetterPushback.cfg"
#define	CONF_DIRS	bp_xpdir, "Output", "preferences"

conf_t *bp_conf = NULL;

static bool_t inited = B_FALSE;
static XPWidgetID main_win = NULL;

#define	MARGIN			30

#define	BUTTON_HEIGHT		22
#define	BUTTON_WIDTH		150

#define	MAIN_WINDOW_WIDTH	\
	(MARGIN + BUTTON_WIDTH + MARGIN + 2 * BUTTON_WIDTH + MARGIN)
#define	MAIN_WINDOW_HEIGHT	\
	(MARGIN + 8 * BUTTON_HEIGHT + MARGIN)

static struct {
	XPWidgetID	xplang;
	XPWidgetID	german;
	XPWidgetID	english;
	XPWidgetID	french;
	XPWidgetID	russian;
	XPWidgetID	chinese;

	XPWidgetID	lang_pref_match_real;
	XPWidgetID	lang_pref_native;
	XPWidgetID	lang_pref_match_english;

	XPWidgetID	save_cfg;
} buttons;

/* Used to set a null tooltip in the button creation macros. */
const char *null_tooltip = NULL;
const char *null_tooltip_xlated[2] = { NULL, NULL };

const char *match_real_tooltip = "Ground crew speaks my language only if "
    "the country the airport is in speaks my language. Otherwise the ground "
    "crew speaks English with a local accent.";
const char *match_real_tooltip_xlated[2] = { NULL, NULL };
const char *native_tooltip = "Ground crew speaks my language regardless "
    "of what language the country the airport is in.";
const char *native_tooltip_xlated[2] = { NULL, NULL };
const char *match_english_tooltip = "Ground crew always speaks English "
    "with a local accent.";
const char *match_english_tooltip_xlated[2] = { NULL, NULL };
const char *save_prefs_tooltip = "Save current preferences to disk. Note: "
    "restart X-Plane for changes to take effect.";
const char *save_prefs_tooltip_xlated[2] = { NULL, NULL };

static void
lang_buttons_update(void)
{
	const char *lang = "XX";
	lang_pref_t lang_pref;

	(void) conf_get_str(bp_conf, "lang", &lang);
	logMsg("lang is %s", lang);
#define	SET_LANG_BTN(btn, l) \
	(XPSetWidgetProperty(buttons.btn, xpProperty_ButtonState, \
	    strcmp(lang, l) == 0))
	SET_LANG_BTN(xplang, "XX");
	SET_LANG_BTN(german, "de");
	SET_LANG_BTN(english, "en");
	SET_LANG_BTN(french, "fr");
	SET_LANG_BTN(russian, "ru");
	SET_LANG_BTN(chinese, "cn");
#undef	SET_LANG_BTN

	if (!conf_get_i(bp_conf, "lang_pref", (int *)&lang_pref))
		lang_pref = LANG_PREF_MATCH_REAL;
	XPSetWidgetProperty(buttons.lang_pref_match_real,
	    xpProperty_ButtonState, lang_pref == LANG_PREF_MATCH_REAL);
	XPSetWidgetProperty(buttons.lang_pref_native,
	    xpProperty_ButtonState, lang_pref == LANG_PREF_NATIVE);
	XPSetWidgetProperty(buttons.lang_pref_match_english,
	    xpProperty_ButtonState, lang_pref == LANG_PREF_MATCH_ENGLISH);
}

static int
main_window_cb(XPWidgetMessage msg, XPWidgetID widget, intptr_t param1,
    intptr_t param2)
{
	XPWidgetID btn = (XPWidgetID)param1;

	UNUSED(param2);

	if (msg == xpMessage_CloseButtonPushed && widget == main_win) {
		XPHideWidget(main_win);
		return (1);
	} else if (msg == xpMsg_PushButtonPressed) {
		if (btn == buttons.save_cfg) {
			logMsg("save");
			(void) bp_conf_save();
		}
		return (0);
	} else if (msg == xpMsg_ButtonStateChanged) {
		if (btn == buttons.xplang) {
			conf_set_str(bp_conf, "lang", NULL);
		} else if (btn == buttons.german) {
			conf_set_str(bp_conf, "lang", "de");
		} else if (btn== buttons.english) {
			conf_set_str(bp_conf, "lang", "en");
		} else if (btn == buttons.french) {
			conf_set_str(bp_conf, "lang", "fr");
		} else if (btn == buttons.russian) {
			conf_set_str(bp_conf, "lang", "ru");
		} else if (btn == buttons.chinese) {
			conf_set_str(bp_conf, "lang", "cn");
		} else if (btn == buttons.lang_pref_match_real) {
			conf_set_i(bp_conf, "lang_pref", LANG_PREF_MATCH_REAL);
		} else if (btn == buttons.lang_pref_native) {
			conf_set_i(bp_conf, "lang_pref", LANG_PREF_NATIVE);
		} else if (btn == buttons.lang_pref_match_english) {
			conf_set_i(bp_conf, "lang_pref",
			    LANG_PREF_MATCH_ENGLISH);
		}
		lang_buttons_update();
	}

	return (0);
}

static void
create_main_window(void)
{
	int x, y;
	tooltip_set_t *tts;

	main_win = create_widget_rel(100, 100, B_FALSE, MAIN_WINDOW_WIDTH,
	    MAIN_WINDOW_HEIGHT, 0, _("BetterPushback Preferences"), 1, NULL,
	    xpWidgetClass_MainWindow);
	XPSetWidgetProperty(main_win, xpProperty_MainWindowHasCloseBoxes, 1);
	XPAddWidgetCallback(main_win, main_window_cb);

	tts = tooltip_set_new(main_win);

#define	LAYOUT_BUTTON(var, text, width, type, behavior, tooltip) \
	do { \
		buttons.var = create_widget_rel(x, y + 2, B_FALSE, 20, \
		    BUTTON_HEIGHT - 5, 1, "", 0, main_win, \
		    xpWidgetClass_Button); \
		XPSetWidgetProperty(buttons.var, xpProperty_ButtonType, \
		    xpRadioButton); \
		XPSetWidgetProperty(buttons.var, xpProperty_ButtonBehavior, \
		    xpButtonBehavior ## behavior); \
		(void) create_widget_rel(x + 20, y, B_FALSE, \
		    width - 20, BUTTON_HEIGHT - 5, 1, text, 0, \
		    main_win, xpWidgetClass_Caption); \
		if (tooltip != NULL) { \
			tooltip ## _xlated[0] = _(tooltip); \
			tooltip_new(tts, x, y, width, BUTTON_HEIGHT, \
			    tooltip ## _xlated); \
		} \
		y += BUTTON_HEIGHT; \
	} while (0)

	x = MARGIN;
	y = MARGIN;

	(void) create_widget_rel(x, y, B_FALSE, BUTTON_WIDTH, BUTTON_HEIGHT, 1,
	    _("User interface"), 0, main_win, xpWidgetClass_Caption);

	y += BUTTON_HEIGHT;

	(void) create_widget_rel(x, y, B_FALSE, BUTTON_WIDTH,
	    6 * BUTTON_HEIGHT, 1, "", 0, main_win, xpWidgetClass_SubWindow);

	LAYOUT_BUTTON(xplang, _("X-Plane's language"), BUTTON_WIDTH,
	    PushButton, CheckBox, null_tooltip);
	LAYOUT_BUTTON(german, "Deutsch", BUTTON_WIDTH, PushButton, CheckBox,
	    null_tooltip);
	LAYOUT_BUTTON(english, "English", BUTTON_WIDTH, PushButton, CheckBox,
	    null_tooltip);
	LAYOUT_BUTTON(french, "Français", BUTTON_WIDTH, PushButton, CheckBox,
	    null_tooltip);
	LAYOUT_BUTTON(russian, "Русский", BUTTON_WIDTH, PushButton, CheckBox,
	    null_tooltip);
	LAYOUT_BUTTON(chinese, "中文", BUTTON_WIDTH, PushButton, CheckBox,
	    null_tooltip);

	x = MARGIN + BUTTON_WIDTH + MARGIN;
	y = MARGIN;

	(void) create_widget_rel(x, y, B_FALSE, 2 * BUTTON_WIDTH,
	    BUTTON_HEIGHT, 1, _("Ground crew audio"), 0, main_win,
	    xpWidgetClass_Caption);

	y += BUTTON_HEIGHT;

	(void) create_widget_rel(x, y, B_FALSE, 2 * BUTTON_WIDTH,
	    3 * BUTTON_HEIGHT, 1, "", 0, main_win, xpWidgetClass_SubWindow);

	LAYOUT_BUTTON(lang_pref_match_real,
	    _("My language only at domestic airports"), 2 * BUTTON_WIDTH,
	    PushButton, CheckBox, match_real_tooltip);
	LAYOUT_BUTTON(lang_pref_native, _("My language at all airports"),
	    2 * BUTTON_WIDTH, PushButton, CheckBox, native_tooltip);
	LAYOUT_BUTTON(lang_pref_match_english, _("English at all airports"),
	    2 * BUTTON_WIDTH, PushButton, CheckBox, match_english_tooltip);

#define LAYOUT_PUSH_BUTTON(var, x, y, w, h, label, tooltip) \
	do { \
		buttons.var = create_widget_rel(x, y, B_FALSE, w, h, 1, \
		    label, 0, main_win, xpWidgetClass_Button); \
		if (tooltip != NULL) { \
			tooltip ## _xlated[0] = _(tooltip); \
			tooltip_new(tts, x, y, w, h, tooltip ## _xlated); \
		} \
	} while (0)


	LAYOUT_PUSH_BUTTON(save_cfg, (MAIN_WINDOW_WIDTH - BUTTON_WIDTH) / 2,
	    MAIN_WINDOW_HEIGHT - MARGIN, BUTTON_WIDTH, BUTTON_HEIGHT,
	    "Save preferences", save_prefs_tooltip);
}

static void
destroy_main_window(void)
{
	XPDestroyWidget(main_win, 1);
	main_win = NULL;
}

bool_t
bp_conf_init(void)
{
	char *path;
	FILE *fp;

	ASSERT(!inited);

	path = mkpathname(CONF_DIRS, CONF_FILENAME, NULL);
	fp = fopen(path, "rb");
	if (fp != NULL) {
		int errline;

		bp_conf = conf_read(fp, &errline);
		if (bp_conf == NULL) {
			logMsg("Error parsing configuration %s: syntax error "
			    "on line %d.", path, errline);
			fclose(fp);
			free(path);
			return (B_FALSE);
		}
		fclose(fp);
	} else {
		bp_conf = conf_create_empty();
	}
	free(path);

	inited = B_TRUE;

	return (B_TRUE);
}

bool_t
bp_conf_save(void)
{
	char *path = mkpathname(CONF_DIRS, NULL);
	bool_t res = B_FALSE;
	FILE *fp;
	bool_t isdir;

	if ((!file_exists(path, &isdir) || !isdir) &&
	    !create_directory_recursive(path)) {
		free(path);
		logMsg("Error writing configuration: "
		    "can't create parent directory %s", path);
		return (B_FALSE);
	}
	free(path);

	path = mkpathname(CONF_DIRS, CONF_FILENAME, NULL);
	logMsg("opening %s", path);
	fp = fopen(path, "wb");
	if (fp != NULL && conf_write(bp_conf, fp)) {
		res = B_TRUE;
	} else {
		logMsg("Error writing configuration %s: %s", path,
		    strerror(errno));
	}

	if (fp != NULL)
		fclose(fp);
	free(path);

	return (res);
}

void
bp_conf_fini(void)
{
	if (!inited)
		return;

	if (main_win != NULL) {
		destroy_main_window();
		tooltip_fini();
	}
	conf_free(bp_conf);
	bp_conf = NULL;

	inited = B_FALSE;
}

void
bp_conf_open(void)
{
	ASSERT(inited);
	if (main_win == NULL) {
		tooltip_init();
		create_main_window();
		lang_buttons_update();
	}
	XPShowWidget(main_win);
}
