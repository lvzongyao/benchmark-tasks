// Prototype part envelopes. Export one part at a time using the selector below.
$fn = 96;
module local_box(p, s) { translate(p) cube(s, center=true); }
module local_cylinder(p, d, l) { translate(p) cylinder(d=d,h=l,center=true); }
part = "plug";
module plug() {
    local_box([0,-12.5,-4], [40,25,28]);
    local_box([-4.25,-47,-4.5], [25.5,44,27]);
    local_box([-13.5,-50,12.5], [7,28,7]);
    local_box([-0.7,10,-1.5], [16.6,20,9]);
    local_box([-0.7,10,5], [16.6,20,4]);
    local_box([7.15,6.5,0.5], [0.9,7,13]);
    local_box([-4.5,-30.5,11], [25,3,2]);
}
module socket() {
    local_box([0,12,-13], [46,24,10]);
    local_box([-17,12,5], [12,24,26]);
    local_box([20,12,5], [6,24,26]);
    local_box([3,12,13.5], [28,24,9]);
    local_box([13.75,6.5,0], [6.5,5,16]);
    local_box([0,27.5,0], [46,7,36]);
    local_box([0,50,0], [26,38,36]);
}
module latch() {
    local_box([13.2,13.5,0.5], [2.4,5,11]);
    local_box([8.8,-7.5,4.5], [2.4,47,3]);
    local_box([11,13.5,4.5], [2.4,5,3]);
    local_box([15.5,-45.5,6.5], [15,27,21]);
    local_box([10,-28,8.5], [4,10,11]);
    local_box([3.5,-20,13.5], [13,12,5]);
}
module safety() {
    local_box([-6.05,-19.75,14], [4.9,8.5,7]);
    local_box([-6.5,-63,16.5], [5,78,3]);
    local_box([-5,-107.5,26], [22,23,16]);
    local_box([-5,-117.5,34.5], [22,3,1]);
}
if (part == "plug") plug();
if (part == "socket") socket();
if (part == "latch") latch();
if (part == "safety") safety();
