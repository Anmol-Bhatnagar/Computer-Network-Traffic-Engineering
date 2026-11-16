/*
 * =====================================================================================
 *  CROSS-DOMAIN FLOW CONTROL VISUAL LAB  —  Win32 Native GUI
 *  Vehicular Traffic Management  <--->  Computer Network Traffic Engineering
 *
 *  Baseline Research:
 *    [1] Varaiya, P. (2013). "Max Pressure Control of a Network of Signalized
 *        Intersections." Transportation Research Part C.
 *    [2] Zaidi, A.A., Kulcsar, B., Wymeersch, H. (2016). "Back-Pressure Traffic
 *        Signal Control with Fixed and Adaptive Routing." IEEE T-ITS.
 *    [3] Wei, H. et al. (2019). "CoLight: Learning Network-Level Cooperation for
 *        Traffic Signal Control." CIKM 2019.
 *    [4] Tassiulas & Ephremides (1992). "Stability Properties of Constrained Queueing
 *        Systems." IEEE Trans. Automatic Control.
 *
 *  JIIT Computer Networks Project — 24B15CS313
 *
 *  KEYBOARD CONTROLS:
 *    [1] Fixed-Time (Baseline)   [2] Max-Pressure   [3] CoLight   [4] Predictive
 *    [F] Sever / Repair Link B-C (simulates road incident / link failure)
 *    [L] Cycle Traffic Load  (Low → Medium → High → Low)
 *    [Space] Pause / Resume   [R] Reset Simulation
 * =====================================================================================
 */

#ifndef UNICODE
#define UNICODE
#endif

#include <windows.h>
#include <string>
#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <deque>

using namespace std;

// ── Window / Timer ────────────────────────────────────────────────────────────
const int WIN_WIDTH  = 1440;
const int WIN_HEIGHT = 900;
const int TIMER_ID   = 1001;

// ── Controller Modes ──────────────────────────────────────────────────────────
enum ControllerMode { CTRL_FIXED = 0, CTRL_MAX_PRESSURE, CTRL_COLIGHT, CTRL_PREDICTIVE };

// ── Data Structures ───────────────────────────────────────────────────────────
struct SimNode {
    int     id;
    wstring name;       // Short label (A, B …)
    wstring role;       // "Intersection" / "Gateway" / "Host"
    int     vx, vy;    // Vehicular canvas position
    int     nx, ny;    // Network canvas position
    int     vehQueue;
    int     netQueue;
    int     signalPhase; // 0 = EW green, 1 = NS green
    int     signalTimer;
};

struct SimEdge {
    int    u, v;
    double capacity;
    bool   active;
    double utilization; // 0..1 for link coloring
};

struct Particle {
    double x, y;
    int    u, v;
    int    target;
    double progress;
    bool   inTransit;
    bool   finished;
    int    age;
    COLORREF color;
};

// ── Global Sim State ──────────────────────────────────────────────────────────
ControllerMode currentMode = CTRL_MAX_PRESSURE;
double currentRho  = 0.75;   // Normalized load
double alphaCoop   = 0.40;   // CoLight cooperation weight
bool   simRunning  = true;
bool   linkSevered = false;
int    tickCount   = 0;

vector<SimNode>   gNodes;
vector<SimEdge>   gEdges;
vector<Particle>  gVehicles;
vector<Particle>  gPackets;

long long completedVehicles = 0;
long long completedPackets  = 0;
double    totalVehDelay     = 0.0;
double    totalNetDelay     = 0.0;

const int HISTORY_LEN = 100;
deque<double> vehQueueHistory;
deque<double> netQueueHistory;
deque<double> vehThroughputHist;
deque<double> netThroughputHist;

// Per-interval counts for throughput
int intervalVehCompleted = 0;
int intervalNetCompleted = 0;

// ── Colour Palette ────────────────────────────────────────────────────────────
namespace C {
    const COLORREF BG        = RGB( 10,  15,  30);
    const COLORREF PANEL     = RGB( 20,  32,  54);
    const COLORREF BORDER    = RGB( 40,  60,  90);
    const COLORREF TITLE     = RGB( 56, 189, 248);
    const COLORREF GREEN     = RGB( 34, 197,  94);
    const COLORREF YELLOW    = RGB(234, 179,   8);
    const COLORREF RED       = RGB(239,  68,  68);
    const COLORREF PURPLE    = RGB(192, 132, 252);
    const COLORREF MUTED     = RGB(100, 120, 150);
    const COLORREF WHITE     = RGB(248, 250, 252);
    const COLORREF ROAD      = RGB( 55,  65,  81);
    const COLORREF ROAD_CUT  = RGB(239,  68,  68);
    const COLORREF LINK      = RGB(  3,  82, 130);
    const COLORREF LINK_CUT  = RGB(239,  68,  68);
    const COLORREF CAR       = RGB( 56, 189, 248);
    const COLORREF PKT       = RGB( 74, 222, 128);
    const COLORREF CHART_V   = RGB( 34, 197,  94);
    const COLORREF CHART_N   = RGB( 56, 189, 248);
}

// ── Helpers ───────────────────────────────────────────────────────────────────
static HBRUSH MakeBrush(COLORREF c) { return CreateSolidBrush(c); }
static HPEN   MakePen(int style, int w, COLORREF c) { return CreatePen(style, w, c); }

static void FilledRect(HDC dc, int x, int y, int w, int h, COLORREF c) {
    HBRUSH b = MakeBrush(c);
    RECT r{x, y, x+w, y+h};
    FillRect(dc, &r, b);
    DeleteObject(b);
}

static void DrawText16(HDC dc, const wstring& s, int x, int y) {
    TextOut(dc, x, y, s.c_str(), (int)s.size());
}

static wstring fmt(double v, int dp = 1) {
    wostringstream ss;
    ss << fixed << setprecision(dp) << v;
    return ss.str();
}

// ── Simulation Init ───────────────────────────────────────────────────────────
void initSimulation() {
    gNodes.clear(); gEdges.clear();
    gVehicles.clear(); gPackets.clear();
    completedVehicles = completedPackets = 0;
    totalVehDelay = totalNetDelay = 0.0;
    vehQueueHistory.clear(); netQueueHistory.clear();
    vehThroughputHist.clear(); netThroughputHist.clear();
    intervalVehCompleted = intervalNetCompleted = 0;
    tickCount = 0;
    linkSevered = false;

    /*
     * 6-Node topology — same graph, two visual renderings:
     *
     *   Vehicular (left canvas, x:70-540, y:95-430):
     *     A — B — E
     *     |   |   |
     *     D — C — F
     *
     *   Network  (right canvas, x:730-1200, y:95-430):
     *     A — B — E
     *     |   |   |
     *     D — C — F
     *
     *  Cross-Domain Analogy:
     *    Road              ↔  Link
     *    Intersection      ↔  Router / Switch
     *    Vehicle           ↔  Packet / Flow
     *    Vehicle Queue     ↔  Packet Queue (Buffer)
     *    Traffic Signal    ↔  Scheduler / Forwarding Decision
     *    Road Capacity     ↔  Link Bandwidth
     *    Accident/Closure  ↔  Link Failure
     *    Travel Time       ↔  End-to-End Latency
     *    Vehicles/hour     ↔  Packets/second
     */

    // Vehicular positions: left half [60..560, 90..440]
    // Network positions:  right half [720..1210, 90..440]
    gNodes = {
        {0, L"A", L"Intersection / Router",    90,  110, 740,  110, 0, 0, 0, 0},
        {1, L"B", L"Gateway / Core Router",   310,  110, 960,  110, 0, 0, 0, 0},
        {2, L"C", L"Gateway / Core Router",   310,  370, 960,  370, 0, 0, 0, 0},
        {3, L"D", L"Intersection / Router",    90,  370, 740,  370, 0, 0, 0, 0},
        {4, L"E", L"District / Edge Router",  540,  110, 1190, 110, 0, 0, 0, 0},
        {5, L"F", L"Harbor / Edge Router",    540,  370, 1190, 370, 0, 0, 0, 0},
    };

    auto addBidir = [](int u, int v, double cap) {
        gEdges.push_back({u, v, cap, true, 0.0});
        gEdges.push_back({v, u, cap, true, 0.0});
    };
    addBidir(0, 1, 6.0);  // A-B
    addBidir(1, 2, 8.0);  // B-C  (severable)
    addBidir(2, 3, 6.0);  // C-D
    addBidir(3, 0, 5.0);  // D-A
    addBidir(1, 4, 7.0);  // B-E
    addBidir(2, 5, 7.0);  // C-F
    addBidir(4, 5, 6.0);  // E-F
    addBidir(0, 2, 4.0);  // A-C (diagonal bypass)
}

// ── Routing / Scheduling ──────────────────────────────────────────────────────
int selectNextHop(int u, int /*target*/, bool isNet) {
    vector<SimEdge*> cands;
    for (auto& e : gEdges)
        if (e.u == u && e.active) cands.push_back(&e);
    if (cands.empty()) return -1;

    auto Q = [&](int node) {
        return isNet ? gNodes[node].netQueue : gNodes[node].vehQueue;
    };

    switch (currentMode) {
    case CTRL_FIXED:
        return cands[rand() % cands.size()]->v;

    case CTRL_MAX_PRESSURE: {
        // Pressure_ij = Q_i − Q_j   (Varaiya 2013)
        int best = -1; double best_p = -1e9;
        for (auto* e : cands) {
            double p = (double)(Q(u) - Q(e->v));
            if (p > best_p) { best_p = p; best = e->v; }
        }
        return best != -1 ? best : cands[0]->v;
    }

    case CTRL_COLIGHT: {
        // score_ij = (Q_i − Q_j) + α * Σ_k(Q_j − Q_k)   (Wei et al. 2019)
        int best = -1; double best_s = -1e9;
        for (auto* e : cands) {
            double localP = (double)(Q(u) - Q(e->v));
            double downP  = 0.0;
            for (auto& d : gEdges)
                if (d.u == e->v && d.active)
                    downP += (double)(Q(e->v) - Q(d.v));
            double score = localP + alphaCoop * downP;
            if (score > best_s) { best_s = score; best = e->v; }
        }
        return best != -1 ? best : cands[0]->v;
    }

    case CTRL_PREDICTIVE: {
        // Anticipate +1 packet arriving at next hop
        int best = -1; double best_p = -1e9;
        for (auto* e : cands) {
            double p = (double)(Q(u) - (Q(e->v) + 1));
            if (p > best_p) { best_p = p; best = e->v; }
        }
        return best != -1 ? best : cands[0]->v;
    }
    }
    return cands[0]->v;
}

// ── Simulation Step ───────────────────────────────────────────────────────────
void simulationStep() {
    if (!simRunning) return;
    tickCount++;

    // Traffic signal phase toggle every 60 ticks
    for (auto& n : gNodes) {
        n.signalTimer++;
        if (n.signalTimer > 60) { n.signalPhase ^= 1; n.signalTimer = 0; }
    }

    // Ingress generation
    if ((double)rand() / RAND_MAX < currentRho * 0.60) {
        int src = rand() % (int)gNodes.size();
        int dst = rand() % (int)gNodes.size();
        while (dst == src) dst = rand() % (int)gNodes.size();

        COLORREF col = RGB(rand()%180+50, rand()%180+50, rand()%180+50);

        gNodes[src].vehQueue++;
        gVehicles.push_back({0,0, src,src, dst, 0.0, false, false, 0, C::CAR});

        gNodes[src].netQueue++;
        gPackets.push_back({0,0, src,src, dst, 0.0, false, false, 0, col});
    }

    // Advance particles
    auto advanceParticles = [&](vector<Particle>& particles, bool isNet) {
        for (auto& p : particles) {
            p.age++;
            if (!p.inTransit) {
                int next = selectNextHop(p.u, p.target, isNet);
                if (next != -1) {
                    p.v = next; p.inTransit = true; p.progress = 0.0;
                    if (isNet) gNodes[p.u].netQueue = max(0, gNodes[p.u].netQueue-1);
                    else       gNodes[p.u].vehQueue = max(0, gNodes[p.u].vehQueue-1);
                }
            } else {
                // Speed depends on queue at destination: backpressure slows you down
                int destQ = isNet ? gNodes[p.v].netQueue : gNodes[p.v].vehQueue;
                double speed = max(0.03, 0.09 - destQ * 0.005);
                p.progress += speed;
                if (p.progress >= 1.0) {
                    p.u = p.v; p.inTransit = false; p.progress = 0.0;
                    if (p.u == p.target) {
                        if (isNet) { completedPackets++;  totalNetDelay += p.age; intervalNetCompleted++; }
                        else       { completedVehicles++; totalVehDelay += p.age; intervalVehCompleted++; }
                        p.finished = true;
                    } else {
                        if (isNet) gNodes[p.u].netQueue++;
                        else       gNodes[p.u].vehQueue++;
                    }
                }
            }
        }
        particles.erase(
            remove_if(particles.begin(), particles.end(), [](const Particle& p){ return p.finished; }),
            particles.end()
        );
    };

    advanceParticles(gVehicles, false);
    advanceParticles(gPackets,  true);

    // Update link utilization estimates
    for (auto& e : gEdges) {
        if (!e.active) { e.utilization = 0.0; continue; }
        int inTransit = 0;
        for (auto& p : gPackets)  if (p.inTransit && p.u == e.u && p.v == e.v) inTransit++;
        for (auto& p : gVehicles) if (p.inTransit && p.u == e.u && p.v == e.v) inTransit++;
        double target = min(1.0, inTransit / max(1.0, e.capacity));
        e.utilization = 0.85 * e.utilization + 0.15 * target; // EMA
    }

    // History (every 5 ticks to reduce noise)
    if (tickCount % 5 == 0) {
        double totV = 0, totN = 0;
        for (auto& n : gNodes) { totV += n.vehQueue; totN += n.netQueue; }
        vehQueueHistory.push_back(totV);
        netQueueHistory.push_back(totN);
        vehThroughputHist.push_back((double)intervalVehCompleted);
        netThroughputHist.push_back((double)intervalNetCompleted);
        intervalVehCompleted = intervalNetCompleted = 0;
        while ((int)vehQueueHistory.size() > HISTORY_LEN)  vehQueueHistory.pop_front();
        while ((int)netQueueHistory.size() > HISTORY_LEN)  netQueueHistory.pop_front();
        while ((int)vehThroughputHist.size() > HISTORY_LEN) vehThroughputHist.pop_front();
        while ((int)netThroughputHist.size() > HISTORY_LEN) netThroughputHist.pop_front();
    }
}

// ── Font helpers (create once, delete in render) ──────────────────────────────
struct Fonts {
    HFONT title, sub, mono, label;
};
Fonts createFonts() {
    Fonts f;
    f.title = CreateFont(20,0,0,0,FW_BOLD,0,0,0,ANSI_CHARSET,0,0,CLEARTYPE_QUALITY, DEFAULT_PITCH|FF_SWISS, L"Segoe UI");
    f.sub   = CreateFont(14,0,0,0,FW_NORMAL,0,0,0,ANSI_CHARSET,0,0,CLEARTYPE_QUALITY, DEFAULT_PITCH|FF_SWISS, L"Segoe UI");
    f.mono  = CreateFont(13,0,0,0,FW_BOLD,  0,0,0,ANSI_CHARSET,0,0,CLEARTYPE_QUALITY, FIXED_PITCH|FF_MODERN, L"Consolas");
    f.label = CreateFont(12,0,0,0,FW_BOLD,  0,0,0,ANSI_CHARSET,0,0,CLEARTYPE_QUALITY, DEFAULT_PITCH|FF_SWISS, L"Segoe UI");
    return f;
}
void deleteFonts(Fonts& f) {
    DeleteObject(f.title); DeleteObject(f.sub);
    DeleteObject(f.mono);  DeleteObject(f.label);
}

// ── Strip Chart ───────────────────────────────────────────────────────────────
void drawStripChart(HDC dc, int x, int y, int w, int h,
                    const deque<double>& hist, COLORREF col,
                    const wstring& label, double maxVal = 0.0)
{
    // Background
    FilledRect(dc, x, y, w, h, RGB(12,20,38));
    HPEN bPen = MakePen(PS_SOLID, 1, C::BORDER);
    HPEN oldPen = (HPEN)SelectObject(dc, bPen);
    Rectangle(dc, x, y, x+w, y+h);
    SelectObject(dc, oldPen);
    DeleteObject(bPen);

    if (hist.size() < 2) return;

    double mxVal = maxVal;
    if (mxVal == 0.0)
        for (auto v : hist) mxVal = max(mxVal, v);
    if (mxVal < 1.0) mxVal = 1.0;

    // Draw grid line at 50%
    HPEN gridPen = MakePen(PS_DOT, 1, RGB(40,55,70));
    SelectObject(dc, gridPen);
    int midY = y + h/2;
    MoveToEx(dc, x+2, midY, NULL); LineTo(dc, x+w-2, midY);
    SelectObject(dc, oldPen);
    DeleteObject(gridPen);

    // Data polyline
    int n = (int)hist.size();
    vector<POINT> pts(n);
    for (int i = 0; i < n; i++) {
        pts[i].x = x + 2 + (int)((double)i / (n-1) * (w-4));
        double norm = hist[i] / mxVal;
        pts[i].y = y + h - 2 - (int)(norm * (h-4));
    }
    HPEN dataPen = MakePen(PS_SOLID, 2, col);
    SelectObject(dc, dataPen);
    Polyline(dc, pts.data(), n);
    SelectObject(dc, oldPen);
    DeleteObject(dataPen);

    // Label
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, col);
    TextOut(dc, x+4, y+3, label.c_str(), (int)label.size());

    // Current value
    wstring cur = fmt(hist.back(), 0);
    SetTextColor(dc, C::WHITE);
    TextOut(dc, x+w-30, y+3, cur.c_str(), (int)cur.size());
}

// ── Main Render ───────────────────────────────────────────────────────────────
void render(HDC hdc, RECT rc) {
    HDC    memDC    = CreateCompatibleDC(hdc);
    HBITMAP memBmp  = CreateCompatibleBitmap(hdc, rc.right, rc.bottom);
    HBITMAP oldBmp  = (HBITMAP)SelectObject(memDC, memBmp);

    // Dark background
    FilledRect(memDC, 0, 0, rc.right, rc.bottom, C::BG);
    SetBkMode(memDC, TRANSPARENT);

    Fonts f = createFonts();
    HFONT oldFont = (HFONT)SelectObject(memDC, f.title);

    // ── HEADER ────────────────────────────────────────────────────────────────
    FilledRect(memDC, 0, 0, WIN_WIDTH, 70, RGB(14,20,40));
    HPEN hdrLine = MakePen(PS_SOLID, 1, C::BORDER);
    HPEN oldPen  = (HPEN)SelectObject(memDC, hdrLine);
    MoveToEx(memDC, 0, 70, NULL); LineTo(memDC, WIN_WIDTH, 70);
    SelectObject(memDC, oldPen); DeleteObject(hdrLine);

    SelectObject(memDC, f.title);
    SetTextColor(memDC, C::TITLE);
    TextOut(memDC, 20, 10, L"CROSS-DOMAIN FLOW CONTROL  |  Vehicular Traffic  ⟺  Computer Networks", 70);

    SelectObject(memDC, f.sub);
    SetTextColor(memDC, C::MUTED);
    TextOut(memDC, 20, 36,
        L"Research Grounding: Varaiya (2013) Max-Pressure  ·  Zaidi et al. Back-Pressure (IEEE T-ITS)  ·  Wei et al. CoLight (CIKM 2019)",
        128);

    // Controller status badge
    wstring modeName;
    COLORREF modeCol;
    wstring hypothesis;
    switch (currentMode) {
        case CTRL_FIXED:
            modeName = L"[1] Fixed-Time  (Baseline)";
            modeCol  = C::MUTED;
            hypothesis = L"H0: Static scheduling — no adaptation. Use as baseline for throughput comparison.";
            break;
        case CTRL_MAX_PRESSURE:
            modeName = L"[2] Max-Pressure  Pressure = Q_i - Q_j";
            modeCol  = C::GREEN;
            hypothesis = L"H1: Max-Pressure gradient transfer improves flow in BOTH domains by prioritizing congested upstream queues.";
            break;
        case CTRL_COLIGHT:
            modeName = L"[3] CoLight  score = (Q_i-Q_j) + α·Σ(Q_j-Q_k)";
            modeCol  = C::PURPLE;
            hypothesis = L"H3: Sharing neighbour queue info allows cooperative controllers to outperform purely local Max-Pressure.";
            break;
        case CTRL_PREDICTIVE:
            modeName = L"[4] Predictive  Pressure = Q_i - (Q_j+1)";
            modeCol  = C::YELLOW;
            hypothesis = L"H4: Anticipating future queue growth avoids hotspots before congestion forms.";
            break;
    }

    int bx = WIN_WIDTH - 400;
    FilledRect(memDC, bx, 8, 390, 22, RGB(20,30,55));
    SelectObject(memDC, f.label);
    SetTextColor(memDC, modeCol);
    TextOut(memDC, bx+6, 10, modeName.c_str(), (int)modeName.size());

    SelectObject(memDC, f.sub);
    SetTextColor(memDC, RGB(180,190,210));
    int rhoInt = (int)(currentRho * 100);
    wstring loadStr = L"ρ = " + fmt(currentRho,2) + L"  (" + (currentRho < 0.6 ? L"Low" : (currentRho < 1.0 ? L"Medium" : L"High")) + L" Load)";
    TextOut(memDC, bx+6, 34, loadStr.c_str(), (int)loadStr.size());

    // ── SECTION HEADERS ──────────────────────────────────────────────────────
    const int TOP = 76;
    const int SIM_H = 390; // simulation canvas height
    const int MID_X = 625; // vertical divider x

    SelectObject(memDC, f.title);
    SetTextColor(memDC, C::GREEN);
    TextOut(memDC, 20, TOP+4, L"VEHICULAR TRAFFIC  (Urban Road Grid)", 36);

    SetTextColor(memDC, C::TITLE);
    TextOut(memDC, MID_X+20, TOP+4, L"COMPUTER NETWORK  (Router Mesh)", 31);

    // Divider
    HPEN divPen = MakePen(PS_DOT, 1, C::BORDER);
    SelectObject(memDC, divPen);
    MoveToEx(memDC, MID_X, TOP, NULL); LineTo(memDC, MID_X, TOP+SIM_H+2);
    SelectObject(memDC, oldPen); DeleteObject(divPen);

    // ── 1. VEHICULAR WORLD ────────────────────────────────────────────────────

    // Roads
    HPEN oldP2;
    for (const auto& e : gEdges) {
        if (!e.active) {
            HPEN rp = MakePen(PS_SOLID, 10, C::ROAD_CUT);
            oldP2 = (HPEN)SelectObject(memDC, rp);
            MoveToEx(memDC, gNodes[e.u].vx, gNodes[e.u].vy, NULL);
            LineTo(memDC,   gNodes[e.v].vx, gNodes[e.v].vy);
            SelectObject(memDC, oldP2); DeleteObject(rp);
        } else {
            // Utilization-tinted road
            int r = (int)(55 + e.utilization * 184);
            int g = (int)(65 - e.utilization * 65);
            int b = (int)(81 - e.utilization * 81);
            HPEN rp = MakePen(PS_SOLID, 10, RGB(r,g,b));
            oldP2 = (HPEN)SelectObject(memDC, rp);
            MoveToEx(memDC, gNodes[e.u].vx, gNodes[e.u].vy, NULL);
            LineTo(memDC,   gNodes[e.v].vx, gNodes[e.v].vy);
            SelectObject(memDC, oldP2); DeleteObject(rp);
        }
    }

    // Intersections
    for (const auto& n : gNodes) {
        // Traffic signal colour by queue depth
        COLORREF sigCol = (n.vehQueue > 6) ? C::RED : (n.vehQueue > 2 ? C::YELLOW : C::GREEN);

        // Outer ring (road asphalt)
        HBRUSH outerBr = MakeBrush(RGB(30,42,60));
        HPEN   outerPn = MakePen(PS_SOLID, 2, sigCol);
        HBRUSH oldBr   = (HBRUSH)SelectObject(memDC, outerBr);
        HPEN   oldPn   = (HPEN)SelectObject(memDC, outerPn);
        Ellipse(memDC, n.vx-18, n.vy-18, n.vx+18, n.vy+18);

        // Signal light
        HBRUSH sigBr = MakeBrush(sigCol);
        SelectObject(memDC, sigBr);
        Ellipse(memDC, n.vx-7, n.vy-7, n.vx+7, n.vy+7);
        DeleteObject(sigBr);

        SelectObject(memDC, oldBr); SelectObject(memDC, oldPn);
        DeleteObject(outerBr); DeleteObject(outerPn);

        // Node label
        SelectObject(memDC, f.label);
        SetTextColor(memDC, C::WHITE);
        wstring nodeLabel = n.name + L"  Q:" + to_wstring(n.vehQueue);
        TextOut(memDC, n.vx-20, n.vy-34, nodeLabel.c_str(), (int)nodeLabel.size());
    }

    // Moving cars (small rectangles)
    for (const auto& p : gVehicles) {
        if (!p.inTransit) continue;
        int ux = gNodes[p.u].vx, uy = gNodes[p.u].vy;
        int vx2 = gNodes[p.v].vx, vy2 = gNodes[p.v].vy;
        double cx = ux + (vx2 - ux) * p.progress;
        double cy = uy + (vy2 - uy) * p.progress;
        int carW = 10, carH = 5;
        HBRUSH carBr = MakeBrush(p.color);
        RECT cr{(LONG)(cx-carW/2),(LONG)(cy-carH/2),(LONG)(cx+carW/2),(LONG)(cy+carH/2)};
        FillRect(memDC, &cr, carBr);
        DeleteObject(carBr);
    }

    // ── 2. NETWORK WORLD ──────────────────────────────────────────────────────

    // Links
    for (const auto& e : gEdges) {
        COLORREF lc;
        int lw = 3;
        if (!e.active) {
            lc = C::LINK_CUT; lw = 4;
        } else {
            // Green→Red with utilization
            int r2 = (int)(e.utilization * 230);
            int g2 = (int)((1.0 - e.utilization) * 180) + 20;
            lc = RGB(r2, g2, 50);
        }
        HPEN lp = MakePen(PS_SOLID, lw, lc);
        oldP2 = (HPEN)SelectObject(memDC, lp);
        MoveToEx(memDC, gNodes[e.u].nx, gNodes[e.u].ny, NULL);
        LineTo(memDC,   gNodes[e.v].nx, gNodes[e.v].ny);
        SelectObject(memDC, oldP2); DeleteObject(lp);
    }

    // Routers
    for (const auto& n : gNodes) {
        int bufPct = min(10, n.netQueue);
        COLORREF bufCol = (n.netQueue > 6) ? C::RED : (n.netQueue > 2 ? C::YELLOW : C::TITLE);

        // Outer circle
        HBRUSH rBr = MakeBrush(RGB(10,20,40));
        HPEN   rPn = MakePen(PS_SOLID, 2, C::TITLE);
        HBRUSH oldBr2 = (HBRUSH)SelectObject(memDC, rBr);
        HPEN   oldPn2 = (HPEN)SelectObject(memDC, rPn);
        Ellipse(memDC, n.nx-22, n.ny-22, n.nx+22, n.ny+22);

        // Core
        HBRUSH coreBr = MakeBrush(bufCol);
        SelectObject(memDC, coreBr);
        Ellipse(memDC, n.nx-10, n.ny-10, n.nx+10, n.ny+10);
        DeleteObject(coreBr);

        SelectObject(memDC, oldBr2); SelectObject(memDC, oldPn2);
        DeleteObject(rBr); DeleteObject(rPn);

        // Buffer bar (below node)
        int barW = min(44, n.netQueue * 6);
        FilledRect(memDC, n.nx-22, n.ny+26, 44, 6, RGB(20,30,50));
        FilledRect(memDC, n.nx-22, n.ny+26, barW, 6, bufCol);

        // Label
        SelectObject(memDC, f.label);
        SetTextColor(memDC, C::MUTED);
        wstring rLabel = n.name + L"  Buf:" + to_wstring(n.netQueue);
        TextOut(memDC, n.nx-22, n.ny-38, rLabel.c_str(), (int)rLabel.size());
    }

    // Flying packets (small circles)
    for (const auto& p : gPackets) {
        if (!p.inTransit) continue;
        double cx = gNodes[p.u].nx + (gNodes[p.v].nx - gNodes[p.u].nx) * p.progress;
        double cy = gNodes[p.u].ny + (gNodes[p.v].ny - gNodes[p.u].ny) * p.progress;
        HBRUSH pBr = MakeBrush(p.color);
        HPEN   pPn = MakePen(PS_SOLID, 1, RGB(255,255,255));
        HBRUSH oldBrP = (HBRUSH)SelectObject(memDC, pBr);
        HPEN   oldPnP = (HPEN)SelectObject(memDC, pPn);
        Ellipse(memDC, (int)cx-4, (int)cy-4, (int)cx+4, (int)cy+4);
        SelectObject(memDC, oldBrP); SelectObject(memDC, oldPnP);
        DeleteObject(pBr); DeleteObject(pPn);
    }

    // ── 3. BOTTOM PANEL ───────────────────────────────────────────────────────
    const int PNL_Y = TOP + SIM_H + 6;
    const int PNL_H = WIN_HEIGHT - PNL_Y - 4;

    FilledRect(memDC, 0, PNL_Y, WIN_WIDTH, PNL_H, C::PANEL);
    HPEN pBorderPen = MakePen(PS_SOLID, 1, C::BORDER);
    SelectObject(memDC, pBorderPen);
    MoveToEx(memDC, 0, PNL_Y, NULL); LineTo(memDC, WIN_WIDTH, PNL_Y);
    SelectObject(memDC, oldPen); DeleteObject(pBorderPen);

    // Strip charts — Queue Length
    int chartY = PNL_Y + 8;
    int chartH = 70;
    double maxQ = 0;
    for (auto v : vehQueueHistory) maxQ = max(maxQ, v);
    for (auto v : netQueueHistory) maxQ = max(maxQ, v);
    if (maxQ < 1) maxQ = 20;

    drawStripChart(memDC, 10,         chartY, 300, chartH, vehQueueHistory,    C::CHART_V, L"Queue Len (Vehicular)", maxQ);
    drawStripChart(memDC, 320,        chartY, 300, chartH, netQueueHistory,     C::CHART_N, L"Queue Len (Network)",   maxQ);
    drawStripChart(memDC, 630,        chartY, 300, chartH, vehThroughputHist,   C::CHART_V, L"Throughput (Vehicles/s)", 0.0);
    drawStripChart(memDC, 940,        chartY, 300, chartH, netThroughputHist,   C::CHART_N, L"Throughput (Packets/s)",  0.0);

    // ── Live metrics row ──────────────────────────────────────────────────────
    int MY = chartY + chartH + 8;
    SelectObject(memDC, f.mono);

    double vAvg = completedVehicles > 0 ? (totalVehDelay / completedVehicles) : 0;
    double nAvg = completedPackets  > 0 ? (totalNetDelay  / completedPackets)  : 0;
    nAvg *= 1.6; // scale to ms

    // Vehicular metrics
    SetTextColor(memDC, C::CHART_V);
    TextOut(memDC, 10,  MY, L"VEHICULAR:", 10);
    SetTextColor(memDC, C::WHITE);
    wstring vm = L"Trips=" + to_wstring(completedVehicles) +
                 L"  Mean Delay=" + fmt(vAvg,1) + L" s" +
                 L"  Vehicles in-transit=" + to_wstring(gVehicles.size());
    TextOut(memDC, 110, MY, vm.c_str(), (int)vm.size());

    // Network metrics
    SetTextColor(memDC, C::CHART_N);
    TextOut(memDC, 10, MY+20, L"NETWORK: ", 9);
    SetTextColor(memDC, C::WHITE);
    wstring nm = L"Pkts=" + to_wstring(completedPackets) +
                 L"  Mean Latency=" + fmt(nAvg,1) + L" ms" +
                 L"  Packets in-flight=" + to_wstring(gPackets.size());
    TextOut(memDC, 110, MY+20, nm.c_str(), (int)nm.size());

    // Active hypothesis
    SelectObject(memDC, f.sub);
    SetTextColor(memDC, C::PURPLE);
    TextOut(memDC, 10, MY+45, hypothesis.c_str(), (int)hypothesis.size());

    // Link status
    if (linkSevered) {
        SetTextColor(memDC, C::RED);
        TextOut(memDC, 10, MY+65, L"⚠  LINK B-C SEVERED  (Road Closure / Link Failure)  —  Press [F] to restore", 74);
    }

    // Analogy table (right side of bottom panel)
    const int TX = 820, TY = PNL_Y + 8;
    SelectObject(memDC, f.label);
    SetTextColor(memDC, C::TITLE);
    TextOut(memDC, TX, TY, L"Cross-Domain Analogy Map", 24);

    struct Row { const wchar_t* veh; const wchar_t* net; };
    Row rows[] = {
        {L"Road",             L"Network Link"},
        {L"Intersection",     L"Router / Switch"},
        {L"Vehicle",          L"Packet / Flow"},
        {L"Vehicle Queue",    L"Packet Buffer"},
        {L"Traffic Signal",   L"Scheduler"},
        {L"Road Capacity",    L"Link Bandwidth"},
        {L"Road Closure",     L"Link Failure"},
        {L"Travel Time",      L"End-to-End Latency"},
        {L"Max-Pressure ctrl",L"Back-Pressure routing"},
    };
    int nRows = sizeof(rows)/sizeof(rows[0]);
    SelectObject(memDC, f.sub);
    for (int i = 0; i < nRows; i++) {
        int ry = TY + 18 + i*14;
        SetTextColor(memDC, C::CHART_V);
        TextOut(memDC, TX,      ry, rows[i].veh, (int)wcslen(rows[i].veh));
        SetTextColor(memDC, C::MUTED);
        TextOut(memDC, TX+150,  ry, L"↔", 1);
        SetTextColor(memDC, C::CHART_N);
        TextOut(memDC, TX+170,  ry, rows[i].net, (int)wcslen(rows[i].net));
    }

    // Keyboard controls
    SelectObject(memDC, f.sub);
    SetTextColor(memDC, C::MUTED);
    TextOut(memDC, 10, WIN_HEIGHT-28,
        L"[1] Fixed  [2] Max-Pressure  [3] CoLight  [4] Predictive  |  [F] Sever/Repair Link B-C  [L] Cycle Load  [Space] Pause  [R] Reset",
        128);

    // Blit
    BitBlt(hdc, 0, 0, rc.right, rc.bottom, memDC, 0, 0, SRCCOPY);

    SelectObject(memDC, oldBmp);
    SelectObject(memDC, oldFont);
    DeleteObject(memBmp);
    DeleteDC(memDC);
    deleteFonts(f);
}

// ── Window Procedure ──────────────────────────────────────────────────────────
LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_CREATE:
        srand((unsigned)time(NULL));
        initSimulation();
        SetTimer(hwnd, TIMER_ID, 30, NULL); // ~33 FPS
        break;

    case WM_TIMER:
        if (!simRunning) break;
        simulationStep();
        InvalidateRect(hwnd, NULL, FALSE);
        break;

    case WM_KEYDOWN:
        switch (wParam) {
        case '1': currentMode = CTRL_FIXED;         break;
        case '2': currentMode = CTRL_MAX_PRESSURE;  break;
        case '3': currentMode = CTRL_COLIGHT;       break;
        case '4': currentMode = CTRL_PREDICTIVE;    break;
        case 'F': case 'f':
            linkSevered = !linkSevered;
            for (auto& e : gEdges)
                if ((e.u==1&&e.v==2)||(e.u==2&&e.v==1))
                    e.active = !linkSevered;
            break;
        case 'L': case 'l':
            if      (currentRho < 0.55) currentRho = 0.75;
            else if (currentRho < 0.90) currentRho = 1.20;
            else                         currentRho = 0.35;
            break;
        case VK_SPACE:
            simRunning = !simRunning;
            break;
        case 'R': case 'r':
            initSimulation();
            break;
        }
        InvalidateRect(hwnd, NULL, FALSE);
        break;

    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hwnd, &ps);
        RECT rc; GetClientRect(hwnd, &rc);
        render(hdc, rc);
        EndPaint(hwnd, &ps);
        break;
    }

    case WM_DESTROY:
        KillTimer(hwnd, TIMER_ID);
        PostQuitMessage(0);
        break;

    default:
        return DefWindowProc(hwnd, msg, wParam, lParam);
    }
    return 0;
}

// ── Entry Point ───────────────────────────────────────────────────────────────
int WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR, int nCmdShow) {
    const wchar_t CLASS[] = L"CrossDomainFlowSim";
    WNDCLASS wc{};
    wc.lpfnWndProc   = WndProc;
    wc.hInstance     = hInst;
    wc.lpszClassName = CLASS;
    wc.hCursor       = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
    RegisterClass(&wc);

    HWND hwnd = CreateWindowEx(
        0, CLASS,
        L"Cross-Domain Flow Simulator  |  Vehicular Traffic ⟺ Computer Networks  |  JIIT 24B15CS313",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, WIN_WIDTH, WIN_HEIGHT,
        NULL, NULL, hInst, NULL
    );
    if (!hwnd) return 0;

    ShowWindow(hwnd, nCmdShow);
    UpdateWindow(hwnd);

    MSG msg{};
    while (GetMessage(&msg, NULL, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
    return (int)msg.wParam;
}
