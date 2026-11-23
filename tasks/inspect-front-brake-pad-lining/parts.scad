// Nominal bench geometry in mm. This model does not specify replacement brake parts.
part = "pad";
lining = 8.4;
gauge = 8;
module box(center, size) { translate(center) cube(size, center=true); }
module pad() {
    color("dimgray") box([0,0,lining/2], [49,120,lining]);
    color("silver") {
        box([0,0,lining+3], [54,132,6]);
        for (side=[-1,1]) box([5,side*68,lining+3], [22,22,6]);
    }
}
module rotor() {
    difference() {
        translate([0,0,21]) cylinder(r=148,h=28,$fn=64);
        translate([0,0,20]) cylinder(r=71,h=30,$fn=64);
    }
    difference() {
        cylinder(r=76,h=30,$fn=64);
        translate([0,0,-1]) cylinder(r=31,h=32,$fn=64);
    }
}
module tool() {
    color("seagreen") {
        box([0,0,0],[4,gauge,gauge]);
        box([0,-(46+gauge/2)/2,-gauge/2+3],[4,46-gauge/2,6]);
        box([0,-89,13],[4,29,18]);
        hull() {
            box([0,-46,-gauge/2+3],[4,.1,6]);
            box([0,-74,13],[4,.1,18]);
        }
        translate([0,-103,13]) rotate([0,90,0])
            difference() {
                cylinder(r=9,h=4,center=true,$fn=32);
                cylinder(r=3.2,h=5,center=true,$fn=32);
            }
    }
}
if (part=="pad") pad();
if (part=="rotor") rotor();
if (part=="gauge") tool();
