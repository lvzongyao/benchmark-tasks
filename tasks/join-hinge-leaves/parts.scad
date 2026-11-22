// Millimetres. Select "base", "flap", or "pin".
part = "base";
$fn = 64;

module box(center, size) {
    translate(center) cube(size, center=true);
}

module cylinder_y(y, length, r0, r1) {
    translate([0, y, 0]) rotate([-90, 0, 0])
        cylinder(h=length, r1=r0, r2=r1, center=true);
}

module eye(y, length, bore=5.8) {
    difference() {
        cylinder_y(y, length, 13, 13);
        union() {
            cylinder_y(y, length + 1, bore, bore);
            cylinder_y(y - length/2 + .75, 1.5, bore + 1.5, bore);
            cylinder_y(y + length/2 - .75, 1.5, bore, bore + 1.5);
        }
    }
}

module base() {
    box([-15,0,-60], [130,99,8]);
    for (y = [-22.5,22.5]) {
        eye(y,10,y > 0 ? 6.3 : 5.8);
        box([-6,y,-33.5], [14,10,52]);
    }
    box([45,0,-31.5], [12,34,49]);
}

module flap() {
    eye(0,26);
    box([59,0,-2], [80,60,10]);
    box([14,0,-2], [14,24,10]);
    box([64,54,4], [20,68,12]);
    box([64,90,24], [24,28,40]);
}

module pin() {
    cylinder_y(-2,61,5,5);
    cylinder_y(30.5,4,5,4.2);
    box([0,-41,7.5], [25,18,25]);
    box([0,-47,28], [25,24,28]);
}

if (part == "base") base();
if (part == "flap") flap();
if (part == "pin") pin();
