// Millimetres. Select "plate", "sleeve", "rod", or "coupon" before export.
part = "plate";
$fn = 96;

module axial_cylinder(length, diameter) {
    rotate([0, 90, 0]) cylinder(h=length, d=diameter, center=true);
}

module plate() {
    difference() {
        union() {
            cube([20, 120, 90], center=true);
            translate([0, 111, 31]) cube([20, 110, 22], center=true);
            translate([0, 168, 44]) cube([25, 24, 56], center=true);
            for (side = [-1, 1])
                translate([0, side * 67, -39.5]) cube([20, 18, 5], center=true);
        }
        axial_cylinder(22, 24);
    }
}

module sleeve() {
    difference() {
        axial_cylinder(16, 23.8);
        axial_cylinder(18, 10);
    }
}

module rod() {
    union() {
        axial_cylinder(100, 14);
        translate([-48, 0, 22]) cube([25, 25, 55], center=true);
    }
}

module coupon() {
    difference() {
        cube([20, 36, 36], center=true);
        axial_cylinder(22, 24);
    }
}

if (part == "plate") plate();
if (part == "sleeve") sleeve();
if (part == "rod") rod();
if (part == "coupon") coupon();
