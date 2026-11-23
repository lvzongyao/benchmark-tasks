// Printable nominal panel, support and holder, dimensions in mm.
// Drill the indicated axle holes after printing if the measured fit requires it.
part = "panel";
$fn=48;
module block(center,size) { translate(center) cube(size,center=true); }
module axle(center,length,radius) {
    translate(center) rotate([90,0,0]) cylinder(r=radius,h=length,center=true);
}
module panel() {
    difference() {
        union() {
            block([-90,0,0],[180,100,8]);
            block([-135,56,3],[23,34,14]);
            block([-135,74,23],[22,23,32]);
            axle([-2,0,0],91,4);
        }
        axle([-2,0,0],102,1.6);
    }
}
module base() {
    difference() {
        union() {
            block([38,0,-34],[120,118,10]);
            for (side = [-1,1])
                block([-2,side*51,-15],[21,9,39]);
        }
        axle([-2,0,0],120,1.6);
    }
}
module support() {
    block([0,0,0],[38,28,35]);
    block([0,-43,-6],[18,74,14]);
    block([0,-74,0],[24,24,35]);
}
if (part=="panel") panel();
if (part=="base") base();
if (part=="support") support();
