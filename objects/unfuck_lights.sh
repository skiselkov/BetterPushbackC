#!/bin/bash
# CDDL HEADER START
#
# This file and its contents are supplied under the terms of the
# Common Development and Distribution License ("CDDL"), version 1.0.
# You may only use this file in accordance with the terms of version
# 1.0 of the CDDL.
#
# A full copy of the text of the CDDL should have accompanied this
# source.  A copy of the CDDL is also available via the Internet at
# http://www.illumos.org/license/CDDL.
#
# CDDL HEADER END

# Copyright 2017 Saso Kiselkov. All rights reserved.

# Due to some inexplicable reason X-Plane's parametric lights are totally
# screwed and can only allow to point forward (despite having an X/Y/Z
# direction vector - go figure...). To fix this strangeness, we need to
# stick the light into a fixed animation segment (which doesn't actually
# animate at all) and use ANIM_trans and ANIM_rotate to get it to point
# in the correct direction.

if [ -z "$1" ]; then
	echo "Usage: $0 <OBJFILE>" >&2
	exit 1
fi

UNFUCKERY_TMP="/tmp/unfucker_tmp.$$.obj"

if ! gawk '
BEGIN {
	in_anim = 0;
}

/\<ANIM_begin\>/ {
	in_anim++;
}

/\<ANIM_end\>/ {
	in_anim--;
}


{
	if ($0 ~ /\<LIGHT_PARAM[[:space:]]+airplane_(landing|taxi|spot|generic)_(core|glow|flare).*unfuck\>/) {
		if (!in_anim)
			print("ANIM_begin");
		type = $2;
		x = $3;
		y = $4;
		z = $5;
		len = $7;
		rx = $8;
		ry = $9;
		rz = $10;
		idx = $11;
		sz = $12;

		print("## UNFUCKERY BEGIN ##");
		printf("ANIM_trans %s %s %s %s %s %s 0 0 no_ref\n", x, y, z,
		    x, y, z);
		# Remember, X-Plane`s Y/Z axes are flipped to Blender`s
		if (rx != 0)
			printf("ANIM_rotate 1 0 0 %s %s 0 0 no_ref\n", rx, rx);
		if (ry != 0)
			printf("ANIM_rotate 0 0 -1 %s %s 0 0 no_ref\n", ry, ry);
		if (rz != 0)
			printf("ANIM_rotate 0 1 0 %s %s 0 0 no_ref\n", rz, rz);
		printf("LIGHT_PARAM %s 0 0 0 0 0 %s %s %s\n", type, len, idx,
		    sz);
		if (!in_anim)
			print("ANIM_end");
		print("## UNFUCKERY END ##");
	} else if ($0 ~ /\<LIGHT_PARAM[[:space:]]+airplane_nav_(tail|left|right).*unfuck\>/) {
		if (!in_anim)
			print("ANIM_begin");
		type = $2;
		x = $3;
		y = $4;
		z = $5;
		rx = $7;
		ry = $8;
		rz = $9;
		sz = $10;
		focus = $11;

		print("## UNFUCKERY BEGIN ##");
		printf("ANIM_trans %s %s %s %s %s %s 0 0 no_ref\n", x, y, z,
		    x, y, z);
		# Remember, X-Plane`s Y/Z axes are flipped to Blender`s
		if (rx != 0)
			printf("ANIM_rotate 1 0 0 %s %s 0 0 no_ref\n", rx, rx);
		if (ry != 0)
			printf("ANIM_rotate 0 0 -1 %s %s 0 0 no_ref\n", ry, ry);
		if (rz != 0)
			printf("ANIM_rotate 0 1 0 %s %s 0 0 no_ref\n", rz, rz);
		printf("LIGHT_PARAM %s 0 0 0 %s %s\n", type, sz, focus);
		if (!in_anim)
			print("ANIM_end");
		print("## UNFUCKERY END ##");
	} else if ($0 ~ /\<UNFUCK_LITERAL\>/) {
		$1 = "";
		print $0;
	} else {
		print;
	}
}
' "$1" > "$UNFUCKERY_TMP"; then
	exit 1
fi

mv "$UNFUCKERY_TMP" "$1"
