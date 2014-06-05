#include "line.h"

namespace libsvg {

const std::string line::name("line"); 

line::line()
{
}

line::line(const line& orig) : shape(orig)
{
}

line::~line()
{
}

}