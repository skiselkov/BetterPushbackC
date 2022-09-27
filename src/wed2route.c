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

#include <stddef.h>
#include <string.h>
#include <errno.h>

#if	!IBM
#include <sys/stat.h>
#endif

#include <libxml/parser.h>
#include <libxml/xpath.h>

#include <acfutils/assert.h>
#include <acfutils/helpers.h>
#include <acfutils/safe_alloc.h>
#include <acfutils/stat.h>

#include "driving.h"
#include "wed2route.h"
#include "xplane.h"

#define	MIN_NODE_DIST	3	/* meters */

typedef struct {
	unsigned long	id;
	const xmlNode	*xml_node;
	avl_node_t	avl_node;
} objmap_t;

/* The virtual vehicle spec we use for computing path segments. */
static const vehicle_t veh = {
	.wheelbase = 1, .fixed_z_off = -0.5, .max_steer = 60,
};

static int
objmap_compar(const void *a, const void *b)
{
	const objmap_t *oa = a, *ob = b;

	if (oa->id < ob->id)
		return (-1);
	if (oa->id == ob->id)
		return (0);
	return (1);
}

void
wed2route_init(void)
{
	xmlInitParser();
}

void
wed2route_fini(void)
{
	xmlCleanupParser();
}

/*
 * Read the route start heading based on the orientation of the ramp start.
 */
static double
route_read_start_hdg(const xmlNode *point_node, xmlXPathContext *xpath_ctx,
    const fpp_t *fpp)
{
	char xpath_query[128];
	xmlXPathObject *xpath_obj = NULL;
	xmlChar *lat_prop = xmlGetProp(point_node, (xmlChar *)"latitude");
	xmlChar *lon_prop = xmlGetProp(point_node, (xmlChar *)"longitude");
	double hdg = NAN;
	double lat = atof((char *)lat_prop), lon = atof((char *)lon_prop);

	ASSERT(lat_prop != NULL);
	ASSERT(lon_prop != NULL);
	snprintf(xpath_query, sizeof (xpath_query),
	    "//object[@class='WED_RampPosition']"
	    "[point[@latitude='%s'][@longitude='%s']]/point/@heading",
	    (char *)lat_prop, (char *)lon_prop);
	xpath_obj = xmlXPathEvalExpression((xmlChar *)xpath_query, xpath_ctx);
	if (xpath_obj == NULL || xpath_obj->nodesetval == NULL ||
	    xpath_obj->nodesetval->nodeNr == 0 ||
	    sscanf((char *)xpath_obj->nodesetval->nodeTab[0]->children->content,
	    "%lf", &hdg) != 1 || !is_valid_hdg(hdg)) {
		hdg = NAN;
	}
	xmlXPathFreeObject(xpath_obj);

	if (isnan(hdg)) {
		/*
		 * There is no ramp start associated with this route,
		 * try using the low control point handle and compute its
		 * heading from the lat/lon of the actual point.
		 */
		xmlChar *ctrl_lat_prop = xmlGetProp(point_node,
		    (xmlChar *)"ctrl_latitude_lo");
		xmlChar *ctrl_lon_prop = xmlGetProp(point_node,
		    (xmlChar *)"ctrl_longitude_lo");
		double ctrl_d_lat = atof((char *)ctrl_lat_prop);
		double ctrl_d_lon = atof((char *)ctrl_lon_prop);

		if (ctrl_d_lat != 0.0 && ctrl_d_lon != 0.0) {
			vect2_t pos = geo2fpp(GEO_POS2(lat, lon), fpp);
			vect2_t ctrl_pos = geo2fpp(GEO_POS2(lat + ctrl_d_lat,
			    lon + ctrl_d_lon), fpp);
			ASSERT(vect2_dist(pos, ctrl_pos) != 0);
			hdg = dir2hdg(vect2_sub(ctrl_pos, pos));
		}
		xmlFree(ctrl_lat_prop);
		xmlFree(ctrl_lon_prop);
	}
	if (isnan(hdg)) {
		/*
		 * The first point on the
		 */
	}

	xmlFree(lat_prop);
	xmlFree(lon_prop);

	return (hdg);
}

static route_t *
cons_route(const xmlNode *node, const avl_tree_t *objmap, avl_tree_t *route_tbl)
{
	char xpath_query[128];
	xmlXPathContext *xpath_ctx = xmlXPathNewContext(node->doc);
	xmlXPathObject *xpath_obj = NULL;
	xmlChar *id_prop = xmlGetProp(node, (xmlChar *)"id");
	route_t *route = NULL;
	fpp_t fpp;
	geo_pos2_t start_pos_geo = NULL_GEO_POS2;
	vect2_t start_pos = NULL_VECT2;
	double start_hdg = NAN;
	list_t segs;
	seg_t *seg;
	char *route_name = NULL;

	list_create(&segs, sizeof (seg_t), offsetof(seg_t, node));

	ASSERT(id_prop != NULL);

	snprintf(xpath_query, sizeof (xpath_query),
	    "//object[@id='%s']/hierarchy/@name", id_prop);
	xpath_obj = xmlXPathEvalExpression((xmlChar *)xpath_query, xpath_ctx);
	if (xpath_obj == NULL || xpath_obj->nodesetval->nodeNr != 1) {
		logMsg("WED2ROUTE: error processing xpath query \"%s\"",
		    xpath_query);
		goto out;
	}
	route_name = strdup((char *)
	    xpath_obj->nodesetval->nodeTab[0]->children->content);
	xmlXPathFreeObject(xpath_obj);

	snprintf(xpath_query, sizeof (xpath_query),
	    "//object[@id='%s']/children/child/@id", id_prop);
	xmlFree(id_prop);

	xpath_obj = xmlXPathEvalExpression((xmlChar *)xpath_query, xpath_ctx);
	if (xpath_obj == NULL) {
		logMsg("WED2ROUTE: error processing xpath query \"%s\" for "
		    "route %s", xpath_query, route_name);
		goto out;
	}

	for (int i = 0; i < xpath_obj->nodesetval->nodeNr; i++) {
		const xmlNode *child_id = xpath_obj->nodesetval->nodeTab[i];
		const xmlNode *point_node = NULL, *vertex_node = NULL;
		const objmap_t *vertex;
		objmap_t srch;
		double lat, lon, lat_hi, lon_hi, lat_lo, lon_lo;
		vect2_t s2e;
		double s2e_hdg;

#define	READ_PROP(var, fmt, node, propname) \
	do { \
		xmlChar *prop = xmlGetProp(node, (xmlChar *)propname); \
		if (prop == NULL || sscanf((char *)prop, fmt, var) != 1) { \
			logMsg("WED2ROUTE: missing or malformed attribute %s " \
			    "in element %s", propname, node->name);\
			if (prop != NULL) \
				xmlFree(prop); \
			goto out; \
		} \
		xmlFree(prop); \
	} while (0)

		if (sscanf((char *)child_id->children->content, "%lu",
		    &srch.id) != 1) {
			logMsg("WED2ROUTE: malformed 'id' attribute %s in "
			    "route %s", child_id->children->content,
			    route_name);
			goto out;
		}
		vertex = avl_find(objmap, &srch, NULL);
		if (vertex == NULL) {
			logMsg("WED2ROUTE: couldn't find vertex ID %lu for "
			    "route %s", srch.id, route_name);
			goto out;
		}
		vertex_node = vertex->xml_node;

		for (const xmlNode *node = vertex_node->children;
		    node != NULL && node != vertex_node->last;
		    node = node->next) {
			if (strcmp((char *)node->name, "point") == 0) {
				point_node = node;
				break;
			}
		}
		if (point_node == NULL) {
			logMsg("WED2ROUTE: vertex <object id='%lu'> doesn't "
			    "containt a <point> on route %s", srch.id,
			    route_name);
			goto out;
		}

		READ_PROP(&lat, "%lf", point_node, "latitude");
		READ_PROP(&lon, "%lf", point_node, "longitude");
		READ_PROP(&lat_hi, "%lf", point_node, "ctrl_latitude_hi");
		READ_PROP(&lon_hi, "%lf", point_node, "ctrl_longitude_hi");
		READ_PROP(&lat_lo, "%lf", point_node, "ctrl_latitude_lo");
		READ_PROP(&lon_lo, "%lf", point_node, "ctrl_longitude_lo");

#undef	READ_PROP

		if (IS_NULL_GEO_POS(start_pos_geo)) {
			start_pos_geo = GEO_POS2(lat, lon);
			start_pos = ZERO_VECT2;
			fpp = stereo_fpp_init(start_pos_geo, 0, &wgs84, B_TRUE);
			start_hdg = route_read_start_hdg(point_node, xpath_ctx,
			    &fpp);
			if (isnan(start_hdg)) {
				logMsg("WED2ROUTE: error parsing start "
				    "heading in route %s", route_name);
				goto out;
			}
		} else {
			geo_pos2_t end_pos_geo = GEO_POS2(lat, lon);
			vect2_t end_pos = geo2fpp(end_pos_geo, &fpp);
			double end_hdg;
			geo_pos2_t node_pos_geo = GEO_POS2(
			    end_pos_geo.lat + lat_lo, end_pos_geo.lon + lon_lo);
			vect2_t node_pos = geo2fpp(node_pos_geo, &fpp);

			/* skip doubled nodes */
			if (vect2_dist(end_pos, start_pos) < MIN_NODE_DIST)
				continue;

			if (vect2_dist(node_pos, end_pos) < MIN_NODE_DIST / 2) {
				end_hdg = dir2hdg(vect2_sub(start_pos,
				    end_pos));
			} else {
				end_hdg = dir2hdg(vect2_sub(node_pos, end_pos));
			}
			/*
			 * If the end_pos in the front 180-degree sector from
			 * start_pos (in the direction of start_hdg), reverse
			 * the end_hdg vector. This is because we only take the
			 * ctrl_{latitude,longitude}_lo nodes into account,
			 * which always back towards our start_pos. When backing
			 * up, this gives the correct orientation of our nose.
			 * However, when going forward (end_pos node ahead of
			 * start_pos), this would point along our tail, but the
			 * driving algorithm needs the next nose heading.
			 */
			s2e = vect2_sub(end_pos, start_pos);
			s2e_hdg = dir2hdg(s2e);
			if (fabs(rel_hdg(s2e_hdg, start_hdg)) < 90)
				end_hdg = normalize_hdg(end_hdg + 180);

			if (compute_segs(&veh, start_pos, start_hdg, end_pos,
			    end_hdg, &segs) <= 0) {
				logMsg("WED2ROUTE: failed to construct route "
				    "%s: route too erratic", route_name);
				goto out;
			}
		}
	}

	if (list_head(&segs) == NULL) {
		logMsg("WED2ROUTE: failed to construct route %s: "
		    "no points in route.", route_name);
		goto out;
	}

	route = route_alloc(NULL, NULL);
	for (seg = list_head(&segs); seg != NULL; seg = list_next(&segs, seg))
		route_seg_append(route_tbl, route, seg);

	logMsg("WED2ROUTE: successfully constructed route %s", route_name);

out:
	if (route_name != NULL)
		xmlFree(route_name);
	if (xpath_obj != NULL)
		xmlXPathFreeObject(xpath_obj);
	if (xpath_ctx != NULL)
		xmlXPathFreeContext(xpath_ctx);
	while ((seg = list_remove_head(&segs)) != NULL)
		free(seg);
	list_destroy(&segs);

	return (route);
}

static bool_t
wed2dat(const char *earthwedxml, const char *route_table_filename)
{
	avl_tree_t *objmap = NULL;
	xmlDoc *doc = NULL;
	xmlXPathContext *xpath_ctx = NULL;
	xmlXPathObject *xpath_obj = NULL;
	avl_tree_t route_table;

	logMsg("WED2ROUTE: processing %s", earthwedxml);

	route_table_create(&route_table);

	doc = xmlParseFile(earthwedxml);
	if (doc == NULL) {
		logMsg("WED2ROUTE: error parsing XML file %s", earthwedxml);
		goto errout;
	}
	xpath_ctx = xmlXPathNewContext(doc);
	if (xpath_ctx == NULL) {
		logMsg("WED2ROUTE: error creating XPath context for "
		    "document %s.", earthwedxml);
		goto errout;
	}

	/* construct an ID-to-object mapping for faster lookup */
	objmap = safe_calloc(1, sizeof (*objmap));
	avl_create(objmap, objmap_compar, sizeof (objmap_t),
	    offsetof(objmap_t, avl_node));

	xpath_obj = xmlXPathEvalExpression((xmlChar *)"//object", xpath_ctx);
	if (xpath_obj == NULL || xpath_obj->nodesetval == NULL)
		goto errout;
	for (int i = 0; i < xpath_obj->nodesetval->nodeNr; i++) {
		objmap_t *e = safe_calloc(1, sizeof (*e));
		const xmlNode *node = xpath_obj->nodesetval->nodeTab[i];
		xmlChar *prop = xmlGetProp(node, (xmlChar *)"id");
		avl_index_t where;

		e->xml_node = node;
		if (prop == NULL || sscanf((char *)prop, "%lu", &e->id) != 1 ||
		    avl_find(objmap, e, NULL) != NULL) {
			logMsg("WED2ROUTE: error parsing object ID mappings "
			    "<object> is missing 'id' attribute, or "
			    "attribute value not a number, or corresponding "
			    "object doesn't exist.");
			if (prop != NULL)
				xmlFree(prop);
			free(e);
			goto errout;
		}
		xmlFree(prop);
		if (avl_find(objmap, e, &where) != NULL) {
			logMsg("WED2ROUTE: error parsing object ID mappings: "
			    "duplicate <object> with id=%lu found", e->id);
			free(e);
			goto errout;
		}
		avl_insert(objmap, e, where);
	}
	xmlXPathFreeObject(xpath_obj);
	xpath_obj = NULL;

	/* Enumerate all pushback routes and construct them in memory. */
	xpath_obj = xmlXPathEvalExpression((xmlChar *)
	    "//object[@class='WED_StringPlacement'][string_placement"
		"[@resource='pushback']]", xpath_ctx);
	for (int i = 0; i < xpath_obj->nodesetval->nodeNr; i++) {
		const xmlNode *node = xpath_obj->nodesetval->nodeTab[i];
		route_t *route = cons_route(node, objmap, &route_table);

		if (route == NULL)
			goto errout;
	}

	xmlXPathFreeObject(xpath_obj);
	xpath_obj = NULL;

	/* Now do the same for all tug routes. */
	xpath_obj = xmlXPathEvalExpression((xmlChar *)
	    "//object[@class='WED_StringPlacement'][string_placement"
		"[@resource='pushback_tug']]", xpath_ctx);
	for (int i = 0; i < xpath_obj->nodesetval->nodeNr; i++) {
		const xmlNode *node = xpath_obj->nodesetval->nodeTab[i];
		route_t *route = cons_route(node, objmap, &route_table);

		if (route == NULL)
			goto errout;
	}

	if (!route_table_store(&route_table, route_table_filename))
		goto errout;

	xmlXPathFreeObject(xpath_obj);
	xpath_obj = NULL;

	if (objmap != NULL) {
		void *cookie = NULL;
		objmap_t *e;
		while ((e = avl_destroy_nodes(objmap, &cookie)) != NULL)
			free(e);
		avl_destroy(objmap);
		free(objmap);
	}
	xmlXPathFreeObject(xpath_obj);
	xmlXPathFreeContext(xpath_ctx);
	xmlFreeDoc(doc);
	route_table_destroy(&route_table);

	return (B_TRUE);
errout:
	if (objmap != NULL) {
		void *cookie = NULL;
		objmap_t *e;
		while ((e = avl_destroy_nodes(objmap, &cookie)) != NULL)
			free(e);
		avl_destroy(objmap);
		free(objmap);
	}
	if (xpath_obj != NULL)
		xmlXPathFreeObject(xpath_obj);
	if (xpath_ctx != NULL)
		xmlXPathFreeContext(xpath_ctx);
	if (doc != NULL)
		xmlFreeDoc(doc);
	route_table_destroy(&route_table);

	return (B_FALSE);
}

void
xlate_wedroutes(void)
{
	char *path = mkpathname(bp_xpdir, "Custom Scenery", NULL);
	DIR *d = opendir(path);
	struct dirent *de;

	if (d == NULL) {
		logMsg("Error opening %s: %s", path, strerror(errno));
		free(path);
		return;
	}
	free(path);

	while ((de = readdir(d)) != NULL) {
		char *entname = NULL, *earthwed = NULL, *routefile = NULL;
		struct stat earthwed_st, routefile_st;
		bool_t exists, isdir;

		entname = mkpathname(bp_xpdir, "Custom Scenery",
		    de->d_name, NULL);
		if (!file_exists(entname, &isdir) || !isdir)
			goto done;

		earthwed = mkpathname(bp_xpdir, "Custom Scenery",
		    de->d_name, "earth.wed.xml", NULL);
		routefile = mkpathname(bp_xpdir, "Custom Scenery",
		    de->d_name, "BetterPushback_routes.dat", NULL);
		exists = file_exists(routefile, NULL);
		if (!file_exists(earthwed, NULL))
			goto done;
		if (stat(earthwed, &earthwed_st) < 0 ||
		    (exists && stat(routefile, &routefile_st) < 0)) {
			logMsg("Cannot stat %s or %s: %s", earthwed,
			    routefile, strerror(errno));
			goto done;
		}
		if ((!exists || earthwed_st.st_mtime > routefile_st.st_mtime) &&
		    !wed2dat(earthwed, routefile)) {
			logMsg("WED2ROUTE: Error translating %s to %s",
			    earthwed, routefile);
		}
done:
		free(earthwed);
		free(routefile);
		free(entname);
	}

	closedir(d);
}
