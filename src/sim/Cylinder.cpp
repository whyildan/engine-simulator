#include "sim/Cylinder.h"

namespace es {

void Cylinder::init(const CylGeom& g, double phi, double Tamb, const GasComp& products) {
    double dv, wall;
    g.chamber(phi, V, dv, wall);
    T = Tamb;
    p = kPatm;
    m = p * V / (kR * T);
    mf = 0; mb = m * 0.05;
    yb = yNew = products;
    U = thermo::energy(m - mb, mb, yb, T);
    prevPhi = phi;
    burning = false;
    egt = Tamb;
}

} // namespace es
