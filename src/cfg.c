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
 * Copyright 2022 Saso Kiselkov. All rights reserved.
 */

#include <string.h>
#include <errno.h>

#include <XPLMGraphics.h>
#include <XPWidgets.h>
#include <XPStandardWidgets.h>

#include <acfutils/assert.h>
#include <acfutils/helpers.h>
#include <acfutils/intl.h>
#include <acfutils/wav.h>
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
#define	BUTTON_WIDTH		200
#define	CHECKBOX_SIZE		20
#define	MIN_BOX_HEIGHT		45

#define	MAIN_WINDOW_HEIGHT	(MARGIN + 16 * BUTTON_HEIGHT + MARGIN)

static struct {
	XPWidgetID	chinese;
	XPWidgetID	english;
	XPWidgetID	french;
	XPWidgetID	german;
	XPWidgetID	portuguese;
	XPWidgetID	russian;
	XPWidgetID	spanish;
	XPWidgetID	xplang;

	XPWidgetID	lang_pref_match_real;
	XPWidgetID	lang_pref_native;
	XPWidgetID	lang_pref_match_english;

	XPWidgetID	disco_when_done;
	XPWidgetID	hide_xp11_tug;
	XPWidgetID	show_dev_menu;

	size_t		num_radio_boxes;
	XPWidgetID	*radio_boxes;
	size_t		num_radio_devs;
	char		**radio_devs;

	size_t		num_sound_boxes;
	XPWidgetID	*sound_boxes;
	size_t		num_sound_devs;
	char		**sound_devs;

	XPWidgetID	save_cfg;
} buttons;

typedef struct {
	const char	*string;
	XPWidgetID	*widget;
	const char	*tooltip;
} checkbox_t;

/* Used to set a null tooltip in the button creation macros. */
const char *null_tooltip = NULL;
const char *null_tooltip_xlated[2] = { NULL, NULL };

const char *match_real_tooltip =
    "Ground crew speaks my language only if the country the airport is\n"
    "in speaks my language. Otherwise the ground crew speaks English\n"
    "with a local accent.";
const char *native_tooltip = "Ground crew speaks my language irrespective "
    "of what country the airport is in.";
const char *match_english_tooltip = "Ground crew always speaks English "
    "with a local accent.";
const char *dev_menu_tooltip = "Show the developer menu options.";
const char *save_prefs_tooltip = "Save current preferences to disk.";
const char *disco_when_done_tooltip =
    "Never ask and always automatically disconnect\n"
    "the tug when the pushback operation is complete.";
const char *hide_xp11_tug_tooltip =
    "Hides default X-Plane 11 pushback tug.\n"
    "Restart X-Plane for this change to take effect.";

static void
buttons_update(void)
{
	const char *lang = "XX";
	lang_pref_t lang_pref;
	bool_t disco_when_done = B_FALSE;
	bool_t show_dev_menu = B_FALSE;
	const char *radio_dev = "", *sound_dev = "";

	(void) conf_get_str(bp_conf, "lang", &lang);
	(void) conf_get_b(bp_conf, "disco_when_done", &disco_when_done);
	(void) conf_get_b(bp_conf, "show_dev_menu", &show_dev_menu);
	(void) conf_get_str(bp_conf, "radio_device", &radio_dev);
	(void) conf_get_str(bp_conf, "sound_device", &sound_dev);

#define	SET_LANG_BTN(btn, l) \
	(XPSetWidgetProperty(buttons.btn, xpProperty_ButtonState, \
	    strcmp(lang, l) == 0))
	SET_LANG_BTN(chinese, "cn");
	SET_LANG_BTN(german, "de");
	SET_LANG_BTN(english, "en");
	SET_LANG_BTN(french, "fr");
	SET_LANG_BTN(portuguese, "pt");
	SET_LANG_BTN(spanish, "es");
	SET_LANG_BTN(russian, "ru");
	SET_LANG_BTN(xplang, "XX");
#undef	SET_LANG_BTN

	if (!conf_get_i(bp_conf, "lang_pref", (int *)&lang_pref))
		lang_pref = LANG_PREF_MATCH_REAL;
	XPSetWidgetProperty(buttons.lang_pref_match_real,
	    xpProperty_ButtonState, lang_pref == LANG_PREF_MATCH_REAL);
	XPSetWidgetProperty(buttons.lang_pref_native,
	    xpProperty_ButtonState, lang_pref == LANG_PREF_NATIVE);
	XPSetWidgetProperty(buttons.lang_pref_match_english,
	    xpProperty_ButtonState, lang_pref == LANG_PREF_MATCH_ENGLISH);
	XPSetWidgetProperty(buttons.disco_when_done,
	    xpProperty_ButtonState, disco_when_done);
	XPSetWidgetProperty(buttons.show_dev_menu, xpProperty_ButtonState,
	    show_dev_menu);
	if (bp_xp_ver >= 11000) {
		bool_t dont_hide = B_FALSE;
		(void) conf_get_b(bp_conf, "dont_hide_xp11_tug", &dont_hide);
		XPSetWidgetProperty(buttons.hide_xp11_tug,
		    xpProperty_ButtonState, !dont_hide);
	}

	XPSetWidgetProperty(buttons.radio_boxes[0], xpProperty_ButtonState,
	    *radio_dev == 0);
	for (size_t i = 0; i < buttons.num_radio_devs; i++) {
		XPSetWidgetProperty(buttons.radio_boxes[i + 1],
		    xpProperty_ButtonState,
		    strcmp(radio_dev, buttons.radio_devs[i]) == 0);
	}

	XPSetWidgetProperty(buttons.sound_boxes[0], xpProperty_ButtonState,
	    *sound_dev == 0);
	for (size_t i = 0; i < buttons.num_sound_devs; i++) {
		XPSetWidgetProperty(buttons.sound_boxes[i + 1],
		    xpProperty_ButtonState,
		    strcmp(sound_dev, buttons.sound_devs[i]) == 0);
	}
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
		if (btn == buttons.save_cfg && !bp_started) {
			(void) bp_conf_save();
			bp_sched_reload();
		}
		return (0);
	} else if (msg == xpMsg_ButtonStateChanged) {
		if (btn == buttons.xplang) {
			conf_set_str(bp_conf, "lang", NULL);
		} else if (btn == buttons.german) {
			conf_set_str(bp_conf, "lang", "de");
		} else if (btn== buttons.english) {
			conf_set_str(bp_conf, "lang", "en");
		} else if (btn== buttons.spanish) {
			conf_set_str(bp_conf, "lang", "es");
		} else if (btn == buttons.french) {
			conf_set_str(bp_conf, "lang", "fr");
		} else if (btn == buttons.portuguese) {
			conf_set_str(bp_conf, "lang", "pt");
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
		} else if (btn == buttons.disco_when_done) {
			conf_set_b(bp_conf, "disco_when_done",
			    XPGetWidgetProperty(buttons.disco_when_done,
				xpProperty_ButtonState, NULL));
		} else if (btn == buttons.show_dev_menu) {
			conf_set_b(bp_conf, "show_dev_menu",
			    XPGetWidgetProperty(buttons.show_dev_menu,
			    xpProperty_ButtonState, NULL));
		} else if (bp_xp_ver >= 11000 && btn == buttons.hide_xp11_tug) {
			conf_set_b(bp_conf, "dont_hide_xp11_tug",
			    !XPGetWidgetProperty(buttons.hide_xp11_tug,
			    xpProperty_ButtonState, NULL));
		}
		for (size_t i = 1; i < buttons.num_radio_boxes; i++) {
			if (btn == buttons.radio_boxes[i]) {
				conf_set_str(bp_conf, "radio_device",
				    buttons.radio_devs[i - 1]);
				break;
			}
		}
		if (btn == buttons.radio_boxes[0])
			conf_set_str(bp_conf, "radio_device", NULL);
		for (size_t i = 1; i < buttons.num_sound_boxes; i++) {
			if (btn == buttons.sound_boxes[i]) {
				conf_set_str(bp_conf, "sound_device",
				    buttons.sound_devs[i - 1]);
				break;
			}
		}
		if (btn == buttons.sound_boxes[0])
			conf_set_str(bp_conf, "sound_device", NULL);
		buttons_update();
	}

	return (0);
}

static int
measure_checkboxes_width(checkbox_t *checkboxes)
{
	int width = 0;
	for (int i = 0; checkboxes[i].string != NULL; i++) {
		int w = XPLMMeasureString(xplmFont_Proportional,
		    checkboxes[i].string, strlen(checkboxes[i].string));
		width = MAX(width, w);
	}
	return (width + CHECKBOX_SIZE);
}

static void
layout_checkboxes(checkbox_t *checkboxes, int x, int y, tooltip_set_t *tts)
{
	int width = measure_checkboxes_width(checkboxes);
	int n;

	for (n = 0; checkboxes[n].string != NULL; n++)
		;

	(void) create_widget_rel(x, y, B_FALSE, width, BUTTON_HEIGHT, 1,
	    checkboxes[0].string, 0, main_win, xpWidgetClass_Caption);
	y += BUTTON_HEIGHT;

	(void) create_widget_rel(x, y, B_FALSE, width + 7,
	    MAX((n - 1) * BUTTON_HEIGHT, MIN_BOX_HEIGHT), 1, "", 0, main_win,
	    xpWidgetClass_SubWindow);

	for (int i = 1; i < n; i++) {
		int off_x = x;
		if (checkboxes[i].widget != NULL) {
			*checkboxes[i].widget = create_widget_rel(x, y + 2,
			    B_FALSE, CHECKBOX_SIZE, CHECKBOX_SIZE, 1, "",
			    0, main_win, xpWidgetClass_Button);
			XPSetWidgetProperty(*checkboxes[i].widget,
			    xpProperty_ButtonType, xpRadioButton);
			XPSetWidgetProperty(*checkboxes[i].widget,
			    xpProperty_ButtonBehavior,
			    xpButtonBehaviorCheckBox);
			off_x += CHECKBOX_SIZE;
		}
		(void) create_widget_rel(off_x, y, B_FALSE,
		    width - (off_x - x), BUTTON_HEIGHT, 1,
		    checkboxes[i].string, 0, main_win, xpWidgetClass_Caption);
		if (checkboxes[i].tooltip != NULL) {
			tooltip_new(tts, x, y, CHECKBOX_SIZE + width,
			    BUTTON_HEIGHT, _(checkboxes[i].tooltip));
		}
		y += BUTTON_HEIGHT;
	}
}

static checkbox_t *
sound_checkboxes_init(const char *name, char ***devs_p, size_t *num_devs_p,
    XPWidgetID **boxes, size_t *num_boxes)
{
	size_t num_devs;
	char **devs = openal_list_output_devs(&num_devs);
	checkbox_t *cb;

	*devs_p = devs;
	*num_devs_p = num_devs;

	*num_boxes = num_devs + 1;
	*boxes = safe_calloc(*num_boxes, sizeof (XPWidgetID));
	cb = safe_calloc((*num_boxes) + 2, sizeof (*cb));

	cb[0].string = strdup(name);
	cb[1].string = strdup(_("Default output device"));
	cb[1].widget = *boxes;
	for (size_t i = 1; i < *num_boxes; i++) {
		if (strlen(devs[i - 1]) > 30) {
			const char *dev = devs[i - 1];
			char name[40] = { 0 };
			strncat(name, dev, 22);
			strcat(name, "...");
			strcat(name, &dev[strlen(dev) - 8]);
			cb[i + 1].string = strdup(name);
		} else {
			cb[i + 1].string = strdup(devs[i - 1]);
		}
		cb[i + 1].widget = (*boxes) + i;
	}

	return (cb);
}

static void
free_checkboxes(checkbox_t *boxes)
{
	for (checkbox_t *b = boxes; b->string != NULL; b++) {
		free((char *)b->string);
		free((char *)b->tooltip);
	}
	free(boxes);
}

static void
create_main_window(void)
{
	tooltip_set_t *tts;
	int col1_width, col2_width, col3_width, col4_width;
	int main_window_width, l;
	char *prefs_title;

	checkbox_t col1[] = {
	    { _("User interface"), NULL, NULL },
	    { _("X-Plane's language"), &buttons.xplang, NULL },
	    { "Deutsch", &buttons.german, NULL },
	    { "English", &buttons.english, NULL },
	    { "Español", &buttons.spanish, NULL},
	    { "Français", &buttons.french, NULL },
	    { "Português", &buttons.portuguese, NULL},
	    { "Русский", &buttons.russian, NULL },
	    { "中文", &buttons.chinese, NULL },
	    { NULL, NULL, NULL }
	};
	checkbox_t col2[] = {
	    { _("Ground crew audio"), NULL, NULL },
	    {
		_("My language only at domestic airports"),
		&buttons.lang_pref_match_real, match_real_tooltip
	    },
	    {
		_("My language at all airports"),
		&buttons.lang_pref_native, native_tooltip
	    },
	    {
		_("English at all airports"),
		&buttons.lang_pref_match_english, match_english_tooltip
	    },
	    { NULL, NULL, NULL }
	};
	checkbox_t *radio_out = sound_checkboxes_init(_("Radio output device"),
	    &buttons.radio_devs, &buttons.num_radio_devs,
	    &buttons.radio_boxes, &buttons.num_radio_boxes);
	checkbox_t *sound_out = sound_checkboxes_init(_("Sound output device"),
	    &buttons.sound_devs, &buttons.num_sound_devs,
	    &buttons.sound_boxes, &buttons.num_sound_boxes);
	checkbox_t other[5] = {
	    { _("Miscellaneous"), NULL, NULL },
	    {
		_("Auto disconnect when done"),
		&buttons.disco_when_done, disco_when_done_tooltip
	    },
	    {
		_("Show developer menu"),
		&buttons.show_dev_menu, dev_menu_tooltip
	    },
	    {
		_("Hide default X-Plane 11 tug"),
		&buttons.hide_xp11_tug, hide_xp11_tug_tooltip
	    },
	    { NULL, NULL, NULL }
	};

	if (bp_xp_ver < 11000)
		other[3] = (checkbox_t){ NULL, NULL, NULL };

	col1_width = measure_checkboxes_width(col1);
	col2_width = measure_checkboxes_width(col2);
	col3_width = measure_checkboxes_width(radio_out);
	col4_width = measure_checkboxes_width(sound_out);
	main_window_width = 4 * MARGIN + col1_width + col2_width +
	    MAX(col3_width, col4_width);

	l = snprintf(NULL, 0, "%s (%s)",
	    _("BetterPushback Preferences"), BP_PLUGIN_VERSION);
	prefs_title = safe_malloc(l + 1);
	snprintf(prefs_title, l + 1, "%s (%s)",
	    _("BetterPushback Preferences"), BP_PLUGIN_VERSION);
	main_win = create_widget_rel(100, 100, B_FALSE, main_window_width,
	    MAIN_WINDOW_HEIGHT, 0, prefs_title, 1, NULL,
	    xpWidgetClass_MainWindow);
	XPSetWidgetProperty(main_win, xpProperty_MainWindowHasCloseBoxes, 1);
	XPAddWidgetCallback(main_win, main_window_cb);
	free(prefs_title);

	tts = tooltip_set_new(main_win);

	layout_checkboxes(col1, MARGIN, MARGIN, tts);
	layout_checkboxes(col2, MARGIN + col1_width + MARGIN, MARGIN, tts);
	layout_checkboxes(other, MARGIN + col1_width + MARGIN,
	    MARGIN + 4.5 * BUTTON_HEIGHT, tts);
	layout_checkboxes(radio_out, 3 * MARGIN + col1_width + col2_width,
	    MARGIN, tts);
	layout_checkboxes(sound_out, 3 * MARGIN + col1_width + col2_width,
	    MARGIN + (buttons.num_radio_boxes + 1.5) * BUTTON_HEIGHT, tts);

#define LAYOUT_PUSH_BUTTON(var, x, y, w, h, label, tooltip) \
	do { \
		buttons.var = create_widget_rel(x, y, B_FALSE, w, h, 1, \
		    label, 0, main_win, xpWidgetClass_Button); \
		if (tooltip != NULL) { \
			tooltip_new(tts, x, y, w, h, _(tooltip)); \
		} \
	} while (0)


	LAYOUT_PUSH_BUTTON(save_cfg, (main_window_width - BUTTON_WIDTH) / 2,
	    MAIN_WINDOW_HEIGHT - MARGIN, BUTTON_WIDTH, BUTTON_HEIGHT,
	    _("Save preferences"), save_prefs_tooltip);

	free_checkboxes(radio_out);
	free_checkboxes(sound_out);
}

static void
destroy_main_window(void)
{
	free_strlist(buttons.radio_devs, buttons.num_radio_devs);
	buttons.radio_devs = NULL;
	buttons.num_radio_devs = 0;
	free_strlist(buttons.sound_devs, buttons.num_sound_devs);
	buttons.sound_devs = NULL;
	buttons.num_sound_devs = 0;

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

static void
gui_init(void)
{
	tooltip_init();
	create_main_window();
	buttons_update();
}

void
bp_conf_set_save_enabled(bool_t flag)
{
	ASSERT(inited);
	if (main_win == NULL)
		gui_init();
	XPSetWidgetProperty(buttons.save_cfg, xpProperty_Enabled, flag);
}

void
bp_conf_open(void)
{
	ASSERT(inited);
	if (main_win == NULL)
		gui_init();
	XPShowWidget(main_win);
}
