
// example016.stl is based on example 16 and credits go to
// the original authors.
//
// Small demo by Berend Dekens to demonstrate the alignment
// of an imported STL file using the model boundaries

min_x = stl_dim(file = "example016.stl", name = "min_x");
max_x = stl_dim(file = "example016.stl", name = "max_x");
max_y = stl_dim(file = "example016.stl", name = "max_y");
translate([-max_x, max_y, 0]) import(file = "example016.stl", convexity = 12);
// Draw the origin of the XY plane - the model should touch the corner
cube([1,1,50]);