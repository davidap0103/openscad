#include "circle.h"

namespace libsvg {

const std::string circle::name("circle");

circle::circle()
{
}

circle::circle(const circle& orig) : shape(orig)
{
	r = orig.r;
}

circle::~circle()
{
}

void
circle::set_attrs(attr_map_t& attrs)
{
	shape::set_attrs(attrs);
	this->x = atof(attrs["cx"].c_str());
	this->y = atof(attrs["cy"].c_str());
	this->r = atof(attrs["r"].c_str());
	
	path_t path;
	draw_ellipse(path, get_x(), get_y(), get_radius(), get_radius());
	path_list.push_back(path);
}

void
circle::dump()
{
	std::cout << get_name()
		<< ": x = " << this->x
		<< ": y = " << this->y
		<< ": r = " << this->r
		<< std::endl;
}
}