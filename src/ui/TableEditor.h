#pragma once
// Spreadsheet-style calibration map editor (Dear ImGui): colour-coded cells, editable axes, active
// operating point + trail, keyboard navigation, multi-cell edits (set / add / scale / interpolate /
// smooth), clipboard (tab separated, spreadsheet compatible), undo / redo, lock, compare (diff) view
// and a 3D surface view.
#include "ecu/Calibration.h"
#include <vector>

namespace es::ui {

struct OpPoint { float x = 0, y = 0; bool valid = false; };

class TableEditor {
public:
    // returns true when the table was changed this frame
    bool draw(const char* id, Table2D& t, const TableInfo& info, OpPoint op, const std::vector<OpPoint>* trail,
              const Table2D* compare, const char* compareName);
    bool drawCurve(const char* id, Curve1D& c, const TableInfo& info, float opX, bool opValid, const Curve1D* compare);
    bool show3D = false;

private:
    struct Sel { int r0 = 0, c0 = 0, r1 = 0, c1 = 0; };
    Sel norm() const;
    void pushUndo(const Table2D& t);
    bool applyOp(Table2D& t, int op, float arg);
    void copySel(const Table2D& t);
    bool paste(Table2D& t);
    // selection: rows / cols in table index space (row 0 = lowest load)
    int curR_ = 0, curC_ = 0, ancR_ = 0, ancC_ = 0;
    bool dragging_ = false;
    std::vector<Table2D> undo_, redo_;
    float setVal_ = 0, addVal_ = 1, mulPct_ = 5;
    char editBuf_[32] = {};
    bool editing_ = false, focusEdit_ = false;
    int axisEdit_ = -1;     // 0..nx-1: x axis, 100+: y axis
    char axisBuf_[24] = {};
    // curves keep their own undo of the whole curve
    std::vector<Curve1D> cUndo_, cRedo_;
    int cCur_ = 0, cAnc_ = 0;
};

} // namespace es::ui
