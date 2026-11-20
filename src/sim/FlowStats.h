#pragma once
// Package 3 (flow map): the gas path averaged for display. For every section of the path the forward and the reverse
// mass flow are kept apart (a pulsating flow that goes back and forth shows both - reversion at the intake valves,
// standoff in front of the throttle, compressor surge), plus the pressure and temperature of its volumes.
// Kept by the Engine every simulation step and copied to the telemetry.

namespace es {

enum FlowSeg : int {
    kFlowInlet,         // air filter -> compressor (boost; reverse = surge) or -> throttle (NA: the throttle flow)
    kFlowBov,           // blow-off valve / supercharger bypass: charge pipe -> atmosphere
    kFlowThrottle,      // throttle(s) -> plenum / runners (reverse = gas pushed back out: standoff)
    kFlowIntakeValve,   // ports -> cylinders (reverse = reversion into the port)
    kFlowExhaustValve,  // cylinders -> exhaust ports (reverse = exhaust drawn back during the overlap)
    kFlowTurbine,       // collector -> turbine wheel -> downpipe
    kFlowWastegate,     // turbine bypass
    kFlowTail,          // downpipe -> cat -> muffler -> tailpipe (net)
    kFlowSegCount
};

struct FlowState {
    double fwd[kFlowSegCount] = {}, back[kFlowSegCount] = {};   // kg/s, averaged over ~0.15 s
    // volumes along the path: pressure kPa absolute, temperature K (averaged over ~0.1 s)
    double pAmb = 101.325, tAmb = 298;
    double pCharge = 101.325;               // compressor outlet / charge pipe (boost)
    double tCompOut = 298, tCharge = 298;   // compressor outlet before / after the intercooler
    double pPlenum = 101.325, tPlenum = 298;
    double pPortIn = 101.325, tPortIn = 298;
    double pPortEx = 101.325;
    double tCyl = 800;                      // gas leaving the cylinders
    double tCollector = 800;                // after the exhaust ports (header / collector)
    double pTurbIn = 101.325, pTurbOut = 101.325;   // turbine housing / downpipe (NA: the collector, ambient)
    double tPostTurb = 800;                 // after the turbine (the wastegate flow mixed back in)
};

} // namespace es
