// Millimetres. Choose product, left, right, tray, or rest in the Customizer.
part = "left";

module box(x0, x1, y0, y1, z0, z1) {
    translate([x0, y0, z0]) cube([x1-x0, y1-y0, z1-z0]);
}

module protector(side) {
    scale([1, side, 1]) union() {
        box(-41, 41, -12.5, 12.5, 0, 4);
        box(-41, 41, 8.5, 12.5, 4, 64);
        box(-41, -39.5, -6, 8.5, 4, 32);
        box(39.5, 41, -6, 8.5, 4, 32);
        box(-14, 14, -12.5, 12.5, 60, 90);
    }
}

module tray() {
    box(-47, 47, -85, 85, 0, 4);
    box(-47, -43, -85, 85, 4, 26);
    box(43, 47, -85, 85, 4, 26);
    box(-43, 43, -85, -81, 4, 26);
    box(-43, 43, 81, 85, 4, 26);
}

if (part == "product") box(-35, 35, -75, 75, 0, 40);
if (part == "left") protector(1);
if (part == "right") protector(-1);
if (part == "tray") tray();
if (part == "rest") box(-29, 29, -46, 46, 0, 15);
