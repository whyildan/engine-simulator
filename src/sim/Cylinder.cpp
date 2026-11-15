#include "sim/Cylinder.h"

namespace es {

void Cylinder::init(const CylGeom& g, double phi, double Tamb) {
    double dv, wall;
    g.chamber(phi, V, dv, wall);
    T = Tamb;
    p = kPatm;
    m = p * V / (kR * T);
    mf = 0; mb = m * 0.05;
    U = m * kCv * T;
    prevPhi = phi;
    burning = false;
    egt = Tamb;
}

} // namespace es
