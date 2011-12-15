/*
 *  OpenSCAD (www.openscad.org)
 *  Copyright (C) 2009-2011 Clifford Wolf <clifford@clifford.at> and
 *                          Marius Kintel <marius@kintel.net>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  As a special exception, you have permission to link this program
 *  with the CGAL library and distribute executables, as long as you
 *  follow the requirements of the GNU GPL in regard to all of the
 *  software in the executable aside from CGAL.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "dxfdim.h"
#include "value.h"
#include "function.h"
#include "builtin.h"
#include "printutils.h"
#include "context.h"
#include "polyset.h"
#include "importnode.h"

#include "mathc99.h"
#include <QDateTime>
#include <QFileInfo>
#include <sstream>

boost::unordered_map<std::string,Value> stl_dim_cache;

Value builtin_stl_dim(const Context *ctx, const std::vector<std::string> &argnames, const std::vector<Value> &args)
{
	std::string filename;
	std::string name;

	for (size_t i = 0; i < argnames.size() && i < args.size(); i++) {
		if (argnames[i] == "file")
			filename = ctx->getAbsolutePath(args[i].text);
		if (argnames[i] == "name")
			name = args[i].text;
	}

	QFileInfo fileInfo(QString::fromStdString(filename));

	std::stringstream keystream;
	// Create a key based on the filename, value name and file info
	keystream << filename << "|" << name << "|" << fileInfo.lastModified().toTime_t() << "|" << fileInfo.size();
	std::string key = keystream.str();
	if (stl_dim_cache.find(key) != stl_dim_cache.end())
		return stl_dim_cache.find(key)->second;

	// Import the STL file - note to self: we need a cache for all properties to prevent loading the
	// objects over and over again.
	import_type_e actualtype = TYPE_STL;
	ImportNode *node = new ImportNode(NULL, actualtype);
	node->filename = filename;

	// Run the actual import
	PolySet *import = node->evaluate_polyset(NULL);
	// Obtain bounds
	BoundingBox bbox = import->getBoundingBox();

	// Inject all bounds into the map so future lookups are cached
	keystream.str("");	// Note: ugly method to reset the stream to reuse it
	keystream << filename << "|min_x|" << fileInfo.lastModified().toTime_t() << "|" << fileInfo.size();
	stl_dim_cache[keystream.str()] = Value(bbox.min()(0));

	keystream.str("");
	keystream << filename << "|min_y|" << fileInfo.lastModified().toTime_t() << "|" << fileInfo.size();
	stl_dim_cache[keystream.str()] = Value(bbox.min()(1));

	keystream.str("");
	keystream << filename << "|min_z|" << fileInfo.lastModified().toTime_t() << "|" << fileInfo.size();
	stl_dim_cache[keystream.str()] = Value(bbox.min()(2));

	keystream.str("");
	keystream << filename << "|max_x|" << fileInfo.lastModified().toTime_t() << "|" << fileInfo.size();
	stl_dim_cache[keystream.str()] = Value(bbox.max()(0));

	keystream.str("");
	keystream << filename << "|max_y|" << fileInfo.lastModified().toTime_t() << "|" << fileInfo.size();
	stl_dim_cache[keystream.str()] = Value(bbox.max()(1));

	keystream.str("");
	keystream << filename << "|max_z|" << fileInfo.lastModified().toTime_t() << "|" << fileInfo.size();
	stl_dim_cache[keystream.str()] = Value(bbox.max()(2));

	// The map is now filled with all supported dimensions for the STL object, retry the lookup
	if (stl_dim_cache.find(key) != stl_dim_cache.end())
		return stl_dim_cache.find(key)->second;

	PRINTF("WARNING: Can't find dimension `%s' in `%s'!", name.c_str(), filename.c_str());
	return Value();
}

void initialize_builtin_stl_dim()
{
	Builtins::init("stl_dim", new BuiltinFunction(&builtin_stl_dim));
}

