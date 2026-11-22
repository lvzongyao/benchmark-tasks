// Dimensions in millimetres. Select "outer" or "inner" before exporting.
part = "outer";
module outer() {
    union() {
        translate([0, 0, 1.5]) cube([220, 150, 3], center=true);
        translate([0, -73.5, 24]) cube([220, 3, 42], center=true);
        translate([0, 73.5, 24]) cube([220, 3, 42], center=true);
        translate([-108.5, 0, 24]) cube([3, 144, 42], center=true);
        translate([108.5, 0, 24]) cube([3, 144, 42], center=true);
        translate([0, -67.5, 41.5]) cube([4, 9, 5], center=true);
        translate([0, 67.5, 41.5]) cube([4, 9, 5], center=true);
        translate([0, -82, 49.5]) cube([24, 16, 47], center=true);
    }
}
module inner() {
    union() {
        translate([0, 0, 1.5]) cube([180, 120, 3], center=true);
        translate([0, -58.5, 19]) cube([180, 3, 32], center=true);
        translate([0, 58.5, 19]) cube([180, 3, 32], center=true);
        translate([-88.5, 0, 19]) cube([3, 114, 32], center=true);
        translate([88.5, 0, 19]) cube([3, 114, 32], center=true);
        translate([0, -64.5, 28]) cube([4, 11, 4], center=true);
        translate([0, 64.5, 28]) cube([4, 11, 4], center=true);
        translate([0, 54.5, 47]) cube([24, 11, 42], center=true);
        translate([0, 31.5, 61]) cube([24, 51, 14], center=true);
    }
}
if (part == "outer") outer();
if (part == "inner") inner();
