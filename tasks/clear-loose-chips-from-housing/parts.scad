// Dimensions in millimetres. Choose a part in the Customizer.
part = "housing";

if (part == "housing") union() {
    translate([-45, -70, 0]) cube([90, 140, 5]);
    translate([-45, -70, 5]) cube([4, 140, 21]);
    translate([41, -70, 5]) cube([4, 140, 21]);
    translate([-41, -70, 5]) cube([82, 4, 21]);
    translate([30, -34, 5]) cube([5, 69, 3]);
    translate([-14, -64, 4]) cube([28, 26, 68]);
}

if (part == "catch_cup") union() {
    translate([-64, -54, 0]) cube([128, 108, 4]);
    translate([-64, -54, 4]) cube([4, 108, 31]);
    translate([60, -54, 4]) cube([4, 108, 31]);
    translate([-60, -54, 4]) cube([120, 4, 19]);
    translate([-60, 50, 4]) cube([120, 4, 31]);
}

if (part == "sweeping_tool") union() {
    translate([-12.5, -41, 0]) cube([25, 74, 6]);
    translate([-12, -34, 6]) cube([24, 25, 44]);
    translate([-12.5, 31, 0]) cube([25, 16, 11]);
}

if (part == "chip_01") union() {
    translate([-3, -3, -2]) cube([6, 6, 4]);
}
