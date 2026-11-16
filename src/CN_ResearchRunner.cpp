/*
 * =====================================================================================
 *  CN_ResearchRunner.cpp  —  Headless Simulation Research Engine
 *
 *  Runs the cross-domain flow simulator programmatically (no GUI) across:
 *    - 4 Controllers : Fixed, Max-Pressure, CoLight, Predictive
 *    - 3 Load Levels : Low (ρ=0.35), Medium (ρ=0.75), High (ρ=1.20)
 *    - 2 Topologies  : Normal (all links up), Failure (link B-C severed)
 *
 *  Total scenarios: 4 × 3 × 2 = 24 runs, each 2000 simulation ticks.
 *
 *  Output:
 *    research_logs/raw_ticks.csv       — per-tick telemetry for every run
 *    research_logs/summary.csv         — aggregated metrics per scenario
 *    research_logs/summary.json        — same data as JSON
 *    research_logs/hypothesis_eval.txt — human-readable hypothesis evaluation
 *
 *  Research Questions:
 *    H1: Does Max-Pressure outperform Fixed-Time in throughput and delay?
 *    H2: Does cooperative CoLight outperform local Max-Pressure under high load?
 *    H3: Under link failure, do pressure-based controllers recover faster?
 *    H4: Does predictive anticipation reduce peak queue length?
 * =====================================================================================
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <string>
#include <numeric>
#include <filesystem>
#include <map>

using namespace std;
namespace fs = std::filesystem;

// ── Controller Modes ──────────────────────────────────────────────────────────
enum ControllerMode { CTRL_FIXED=0, CTRL_MAX_PRESSURE, CTRL_COLIGHT, CTRL_PREDICTIVE };
const char* ctrlName[] = { "Fixed-Time", "Max-Pressure", "CoLight", "Predictive" };

// ── Simulation Structures (identical physics to the GUI version) ──────────────
struct SimNode {
    int id;
    string name;
    int vehQueue = 0;
    int netQueue = 0;
};

struct SimEdge {
    int    u, v;
    double capacity;
    bool   active = true;
    double utilization = 0.0;
};

struct Particle {
    int    u, v, target;
    double progress = 0.0;
    bool   inTransit = false;
    bool   finished  = false;
    int    age       = 0;
};

// ── Per-tick record ───────────────────────────────────────────────────────────
struct TickRecord {
    int    tick;
    int    totalVehQueue;
    int    totalNetQueue;
    int    vehiclesInTransit;
    int    packetsInFlight;
    int    vehCompletedThisTick;
    int    netCompletedThisTick;
};

// ── Scenario Summary ──────────────────────────────────────────────────────────
struct ScenarioResult {
    string controller;
    double rho;
    bool   linkFailure;
    int    ticks;

    // Throughput
    long long totalVehCompleted;
    long long totalNetCompleted;
    double    vehThroughputPerTick; // completed / tick
    double    netThroughputPerTick;

    // Delay
    double meanVehDelay;   // ticks
    double meanNetDelay;   // ticks
    double meanVehDelayMs; // scaled

    // Queue
    double meanVehQueue;
    double meanNetQueue;
    double peakVehQueue;
    double peakNetQueue;

    // Stability (stddev of queue length over last 500 ticks)
    double vehQueueStddev;
    double netQueueStddev;

    // Recovery (ticks until queue drops below 10 after failure, 9999 if never)
    int    recoveryTicks;

    // Fairness (Jain index across nodes: closer to 1 = more fair)
    double jainFairnessVeh;
    double jainFairnessNet;
};

// ── Global Sim State (reset per scenario) ─────────────────────────────────────
vector<SimNode>  gNodes;
vector<SimEdge>  gEdges;
vector<Particle> gVehicles;
vector<Particle> gPackets;

ControllerMode gMode;
double gRho;
double gAlpha = 0.40;

long long gVehCompleted = 0, gNetCompleted = 0;
double gTotalVehDelay = 0.0, gTotalNetDelay = 0.0;
vector<double> nodeVehServed, nodeNetServed; // per-node throughput for Jain

void initSim(bool linkFailure) {
    gNodes.clear(); gEdges.clear();
    gVehicles.clear(); gPackets.clear();
    gVehCompleted = gNetCompleted = 0;
    gTotalVehDelay = gTotalNetDelay = 0.0;
    nodeVehServed.assign(6, 0.0);
    nodeNetServed.assign(6, 0.0);

    gNodes = {
        {0,"A"}, {1,"B"}, {2,"C"}, {3,"D"}, {4,"E"}, {5,"F"}
    };

    auto addBidir = [&](int u, int v, double cap) {
        gEdges.push_back({u, v, cap, true,  0.0});
        gEdges.push_back({v, u, cap, true,  0.0});
    };
    addBidir(0,1,6.0); addBidir(1,2,8.0); addBidir(2,3,6.0);
    addBidir(3,0,5.0); addBidir(1,4,7.0); addBidir(2,5,7.0);
    addBidir(4,5,6.0); addBidir(0,2,4.0);

    if (linkFailure) {
        for (auto& e : gEdges)
            if ((e.u==1&&e.v==2)||(e.u==2&&e.v==1))
                e.active = false;
    }
}

auto Q = [](int node, bool isNet) -> int& {
    return isNet ? gNodes[node].netQueue : gNodes[node].vehQueue;
};

int selectNextHop(int u, bool isNet) {
    vector<SimEdge*> cands;
    for (auto& e : gEdges)
        if (e.u == u && e.active) cands.push_back(&e);
    if (cands.empty()) return -1;

    switch (gMode) {
    case CTRL_FIXED:
        return cands[rand() % cands.size()]->v;

    case CTRL_MAX_PRESSURE: {
        int best=-1; double bp=-1e9;
        for (auto* e : cands) {
            double p = (double)(Q(u,isNet) - Q(e->v,isNet));
            if (p > bp) { bp=p; best=e->v; }
        }
        return best!=-1 ? best : cands[0]->v;
    }

    case CTRL_COLIGHT: {
        int best=-1; double bs=-1e9;
        for (auto* e : cands) {
            double lp = (double)(Q(u,isNet) - Q(e->v,isNet));
            double dp = 0.0;
            for (auto& d : gEdges)
                if (d.u==e->v && d.active)
                    dp += (double)(Q(e->v,isNet) - Q(d.v,isNet));
            double s = lp + gAlpha*dp;
            if (s > bs) { bs=s; best=e->v; }
        }
        return best!=-1 ? best : cands[0]->v;
    }

    case CTRL_PREDICTIVE: {
        int best=-1; double bp=-1e9;
        for (auto* e : cands) {
            double p = (double)(Q(u,isNet) - (Q(e->v,isNet)+1));
            if (p > bp) { bp=p; best=e->v; }
        }
        return best!=-1 ? best : cands[0]->v;
    }
    }
    return cands[0]->v;
}

TickRecord stepSim() {
    TickRecord rec{};

    // Ingress
    if ((double)rand()/RAND_MAX < gRho * 0.60) {
        int src = rand() % (int)gNodes.size();
        int dst = rand() % (int)gNodes.size();
        while (dst==src) dst = rand() % (int)gNodes.size();
        gNodes[src].vehQueue++;
        gVehicles.push_back({src,src,dst, 0.0, false, false, 0});
        gNodes[src].netQueue++;
        gPackets.push_back({src,src,dst, 0.0, false, false, 0});
    }

    auto advance = [&](vector<Particle>& parts, bool isNet) {
        int completed = 0;
        for (auto& p : parts) {
            p.age++;
            if (!p.inTransit) {
                int next = selectNextHop(p.u, isNet);
                if (next != -1) {
                    p.v=next; p.inTransit=true; p.progress=0.0;
                    int& qi = Q(p.u, isNet);
                    qi = max(0, qi-1);
                }
            } else {
                int destQ = Q(p.v, isNet);
                double speed = max(0.03, 0.09 - destQ*0.005);
                p.progress += speed;
                if (p.progress >= 1.0) {
                    p.u = p.v; p.inTransit = false; p.progress = 0.0;
                    if (p.u == p.target) {
                        if (isNet) {
                            gNetCompleted++;
                            gTotalNetDelay += p.age;
                            nodeNetServed[p.u]++;
                        } else {
                            gVehCompleted++;
                            gTotalVehDelay += p.age;
                            nodeVehServed[p.u]++;
                        }
                        completed++;
                        p.finished = true;
                    } else {
                        Q(p.u, isNet)++;
                    }
                }
            }
        }
        parts.erase(remove_if(parts.begin(), parts.end(),
            [](const Particle& p){ return p.finished; }), parts.end());
        return completed;
    };

    rec.vehCompletedThisTick = advance(gVehicles, false);
    rec.netCompletedThisTick = advance(gPackets,  true);

    // Collect stats
    int totV=0, totN=0;
    for (auto& n : gNodes) { totV+=n.vehQueue; totN+=n.netQueue; }
    rec.totalVehQueue    = totV;
    rec.totalNetQueue    = totN;
    rec.vehiclesInTransit= (int)gVehicles.size();
    rec.packetsInFlight  = (int)gPackets.size();
    return rec;
}

double jainIndex(const vector<double>& x) {
    double sum=0, sumSq=0;
    int n = (int)x.size();
    for (auto v : x) { sum+=v; sumSq+=v*v; }
    if (sumSq < 1e-9) return 1.0;
    return (sum*sum) / (n * sumSq);
}

double stddev(const vector<double>& v) {
    if (v.size() < 2) return 0.0;
    double mean = accumulate(v.begin(),v.end(),0.0)/v.size();
    double sq = 0;
    for (auto x : v) sq += (x-mean)*(x-mean);
    return sqrt(sq/(v.size()-1));
}

// ── Run one full scenario ─────────────────────────────────────────────────────
pair<ScenarioResult, vector<TickRecord>>
runScenario(ControllerMode mode, double rho, bool linkFail, int ticks,
            ofstream& rawLog, bool writeHeader)
{
    gMode = mode; gRho = rho;
    initSim(linkFail);
    srand(42); // reproducible seed

    vector<TickRecord> history;
    history.reserve(ticks);

    // For stddev computation over last 500 ticks
    vector<double> lateVQ, lateNQ;
    int recoveryTick = 9999;
    bool recovered = false;

    if (writeHeader) {
        rawLog << "controller,rho,link_failure,tick,"
               << "veh_queue,net_queue,veh_transit,net_flight,"
               << "veh_completed_tick,net_completed_tick\n";
    }

    for (int t = 1; t <= ticks; t++) {
        TickRecord r = stepSim();
        r.tick = t;
        history.push_back(r);

        // Write raw tick
        rawLog << ctrlName[mode] << "," << rho << "," << (linkFail?"yes":"no") << ","
               << t << "," << r.totalVehQueue << "," << r.totalNetQueue << ","
               << r.vehiclesInTransit << "," << r.packetsInFlight << ","
               << r.vehCompletedThisTick << "," << r.netCompletedThisTick << "\n";

        if (t > ticks-500) {
            lateVQ.push_back(r.totalVehQueue);
            lateNQ.push_back(r.totalNetQueue);
        }

        if (linkFail && !recovered && r.totalNetQueue < 10 && t > 100) {
            recoveryTick = t;
            recovered = true;
        }
    }

    // Aggregate
    double totVQ=0,totNQ=0, peakVQ=0, peakNQ=0;
    for (auto& r : history) {
        totVQ += r.totalVehQueue; totNQ += r.totalNetQueue;
        peakVQ = max(peakVQ, (double)r.totalVehQueue);
        peakNQ = max(peakNQ, (double)r.totalNetQueue);
    }

    ScenarioResult res;
    res.controller         = ctrlName[mode];
    res.rho                = rho;
    res.linkFailure        = linkFail;
    res.ticks              = ticks;
    res.totalVehCompleted  = gVehCompleted;
    res.totalNetCompleted  = gNetCompleted;
    res.vehThroughputPerTick = (double)gVehCompleted / ticks;
    res.netThroughputPerTick = (double)gNetCompleted  / ticks;
    res.meanVehDelay       = gVehCompleted>0 ? gTotalVehDelay/gVehCompleted : 0;
    res.meanNetDelay       = gNetCompleted>0  ? gTotalNetDelay/gNetCompleted  : 0;
    res.meanVehDelayMs     = res.meanVehDelay * 1.6;
    res.meanVehQueue       = totVQ / ticks;
    res.meanNetQueue       = totNQ / ticks;
    res.peakVehQueue       = peakVQ;
    res.peakNetQueue       = peakNQ;
    res.vehQueueStddev     = stddev(lateVQ);
    res.netQueueStddev     = stddev(lateNQ);
    res.recoveryTicks      = recoveryTick;
    res.jainFairnessVeh    = jainIndex(nodeVehServed);
    res.jainFairnessNet    = jainIndex(nodeNetServed);

    return {res, history};
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main() {
    fs::create_directories("research_logs");

    // ── Controllers × Loads × Failure modes ──────────────────────────────────
    vector<ControllerMode> modes    = {CTRL_FIXED, CTRL_MAX_PRESSURE, CTRL_COLIGHT, CTRL_PREDICTIVE};
    vector<double>         loads    = {0.35, 0.75, 1.20};
    vector<bool>           failures = {false, true};
    const int TICKS = 2000;

    ofstream rawLog("research_logs/raw_ticks.csv");
    ofstream sumCSV("research_logs/summary.csv");
    ofstream sumJSON("research_logs/summary.json");
    ofstream hypEval("research_logs/hypothesis_eval.txt");

    cout << "\n╔══════════════════════════════════════════════════════════╗\n";
    cout <<   "║  CROSS-DOMAIN FLOW SIMULATOR  —  Research Run Engine    ║\n";
    cout <<   "║  JIIT 24B15CS313  |  " << 4*3*2 << " scenarios × " << TICKS << " ticks each  ║\n";
    cout <<   "╚══════════════════════════════════════════════════════════╝\n\n";

    // CSV header
    sumCSV << "controller,rho,link_failure,ticks,"
           << "veh_completed,net_completed,"
           << "veh_throughput_per_tick,net_throughput_per_tick,"
           << "mean_veh_delay_ticks,mean_net_delay_ticks,mean_veh_delay_ms,"
           << "mean_veh_queue,mean_net_queue,"
           << "peak_veh_queue,peak_net_queue,"
           << "veh_queue_stddev,net_queue_stddev,"
           << "recovery_ticks,jain_fairness_veh,jain_fairness_net\n";

    vector<ScenarioResult> allResults;
    bool firstRaw = true;
    int scenarioIdx = 0;

    sumJSON << "[\n";

    for (ControllerMode mode : modes) {
        for (double rho : loads) {
            for (bool fail : failures) {
                scenarioIdx++;
                string loadLabel = rho < 0.5 ? "Low" : (rho < 1.0 ? "Medium" : "High");
                cout << "[" << setw(2) << scenarioIdx << "/24]  "
                     << setw(14) << left << ctrlName[mode]
                     << "  rho=" << fixed << setprecision(2) << rho
                     << "  " << (fail ? "FAILURE" : "Normal ")
                     << "  ...  ";
                cout.flush();

                auto [res, hist] = runScenario(mode, rho, fail, TICKS, rawLog, firstRaw);
                firstRaw = false;
                allResults.push_back(res);

                cout << "  VehCompleted=" << setw(5) << res.totalVehCompleted
                     << "  MeanDelay=" << fixed << setprecision(1) << res.meanVehDelay
                     << "  MeanQ=" << fixed << setprecision(1) << res.meanVehQueue
                     << "\n";

                // Summary CSV row
                sumCSV << res.controller << "," << res.rho << "," << (res.linkFailure?"yes":"no") << ","
                       << res.ticks << "," << res.totalVehCompleted << "," << res.totalNetCompleted << ","
                       << fixed << setprecision(4)
                       << res.vehThroughputPerTick << "," << res.netThroughputPerTick << ","
                       << res.meanVehDelay << "," << res.meanNetDelay << "," << res.meanVehDelayMs << ","
                       << res.meanVehQueue << "," << res.meanNetQueue << ","
                       << res.peakVehQueue << "," << res.peakNetQueue << ","
                       << res.vehQueueStddev << "," << res.netQueueStddev << ","
                       << res.recoveryTicks << ","
                       << setprecision(4) << res.jainFairnessVeh << "," << res.jainFairnessNet << "\n";

                // JSON entry
                if (scenarioIdx > 1) sumJSON << ",\n";
                sumJSON << "  {\n"
                        << "    \"controller\": \"" << res.controller << "\",\n"
                        << "    \"rho\": " << rho << ",\n"
                        << "    \"link_failure\": " << (fail?"true":"false") << ",\n"
                        << "    \"veh_completed\": " << res.totalVehCompleted << ",\n"
                        << "    \"net_completed\": " << res.totalNetCompleted << ",\n"
                        << "    \"veh_throughput_per_tick\": " << fixed << setprecision(5) << res.vehThroughputPerTick << ",\n"
                        << "    \"net_throughput_per_tick\": " << fixed << setprecision(5) << res.netThroughputPerTick << ",\n"
                        << "    \"mean_veh_delay_ticks\": " << fixed << setprecision(2) << res.meanVehDelay << ",\n"
                        << "    \"mean_net_latency_ms\": " << fixed << setprecision(2) << res.meanVehDelayMs << ",\n"
                        << "    \"mean_veh_queue\": " << fixed << setprecision(2) << res.meanVehQueue << ",\n"
                        << "    \"mean_net_queue\": " << fixed << setprecision(2) << res.meanNetQueue << ",\n"
                        << "    \"peak_veh_queue\": " << (int)res.peakVehQueue << ",\n"
                        << "    \"peak_net_queue\": " << (int)res.peakNetQueue << ",\n"
                        << "    \"veh_queue_stddev\": " << fixed << setprecision(3) << res.vehQueueStddev << ",\n"
                        << "    \"net_queue_stddev\": " << fixed << setprecision(3) << res.netQueueStddev << ",\n"
                        << "    \"recovery_ticks\": " << res.recoveryTicks << ",\n"
                        << "    \"jain_fairness_veh\": " << fixed << setprecision(4) << res.jainFairnessVeh << ",\n"
                        << "    \"jain_fairness_net\": " << fixed << setprecision(4) << res.jainFairnessNet << "\n"
                        << "  }";
            }
        }
    }

    sumJSON << "\n]\n";
    rawLog.close(); sumCSV.close(); sumJSON.close();

    // ── Hypothesis Evaluation ─────────────────────────────────────────────────
    cout << "\n\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
    cout << "  HYPOTHESIS EVALUATION\n";
    cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";

    // Helper: find result by controller name, rho, failure
    auto findResult = [&](const string& ctrl, double rho, bool fail) -> const ScenarioResult* {
        for (auto& r : allResults)
            if (r.controller==ctrl && fabs(r.rho-rho)<0.01 && r.linkFailure==fail)
                return &r;
        return nullptr;
    };

    auto pct = [](double a, double b) -> double {
        return b>0 ? (a-b)/b*100.0 : 0.0;
    };

    auto printHyp = [&](const string& title, const string& text) {
        cout << "  ┌─ " << title << "\n";
        cout << "  │  " << text << "\n";
        hypEval << "── " << title << " ──\n" << text << "\n\n";
    };

    hypEval << "CROSS-DOMAIN FLOW SIMULATOR  —  Hypothesis Evaluation Report\n";
    hypEval << "JIIT Computer Networks 24B15CS313\n";
    hypEval << "Generated: " << __DATE__ << " " << __TIME__ << "\n";
    hypEval << "Simulation: 2000 ticks per scenario, 24 scenarios total\n\n";
    hypEval << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";

    // H1: Max-Pressure vs Fixed
    {
        cout << "H1: Max-Pressure vs Fixed-Time (all load levels)\n\n";
        hypEval << "H1: Max-Pressure vs Fixed-Time\n"
                << "    Hypothesis: Max-Pressure gradient transfer improves throughput\n"
                << "    and reduces delay in BOTH vehicular and network domains.\n\n";

        for (double rho : loads) {
            auto* f  = findResult("Fixed-Time",   rho, false);
            auto* mp = findResult("Max-Pressure",  rho, false);
            if (!f || !mp) continue;
            string loadLabel = rho<0.5 ? "Low" : (rho<1.0 ? "Medium" : "High");

            double vehGain = pct(mp->vehThroughputPerTick, f->vehThroughputPerTick);
            double netGain = pct(mp->netThroughputPerTick, f->netThroughputPerTick);
            double delayRed= pct(f->meanVehDelay, mp->meanVehDelay);
            double qRed    = pct(f->meanVehQueue, mp->meanVehQueue);

            ostringstream ss;
            ss << fixed << setprecision(1);
            ss << "    Load=" << loadLabel << " (rho=" << rho << "):\n"
               << "      Vehicular throughput:  Fixed=" << f->vehThroughputPerTick*1000 << " veh/1000t"
               << "  MaxP=" << mp->vehThroughputPerTick*1000 << " veh/1000t"
               << "  GAIN=" << (vehGain>=0?"+":"") << vehGain << "%\n"
               << "      Network throughput:    Fixed=" << f->netThroughputPerTick*1000 << " pkt/1000t"
               << "  MaxP=" << mp->netThroughputPerTick*1000 << " pkt/1000t"
               << "  GAIN=" << (netGain>=0?"+":"") << netGain << "%\n"
               << "      Mean vehicle delay:    Fixed=" << f->meanVehDelay << "t"
               << "  MaxP=" << mp->meanVehDelay << "t"
               << "  REDUCTION=" << (delayRed>=0?"+":"") << delayRed << "%\n"
               << "      Mean vehicle queue:    Fixed=" << f->meanVehQueue
               << "  MaxP=" << mp->meanVehQueue
               << "  REDUCTION=" << (qRed>=0?"+":"") << qRed << "%\n";

            cout << ss.str();
            hypEval << ss.str();
        }
    }

    cout << "\nH2: CoLight vs Max-Pressure (cooperative benefit)\n\n";
    hypEval << "\nH2: CoLight (cooperative) vs Max-Pressure (local)\n"
            << "    Hypothesis: Sharing neighbour queue info further improves flow.\n\n";
    {
        for (double rho : loads) {
            auto* mp  = findResult("Max-Pressure", rho, false);
            auto* col = findResult("CoLight",      rho, false);
            if (!mp || !col) continue;
            string loadLabel = rho<0.5 ? "Low" : (rho<1.0 ? "Medium" : "High");

            double vg = pct(col->vehThroughputPerTick, mp->vehThroughputPerTick);
            double ng = pct(col->netThroughputPerTick, mp->netThroughputPerTick);
            double jv = col->jainFairnessVeh - mp->jainFairnessVeh;
            double jn = col->jainFairnessNet - mp->jainFairnessNet;

            ostringstream ss;
            ss << fixed << setprecision(1);
            ss << "    Load=" << loadLabel << ":\n"
               << "      Veh throughput delta:  " << (vg>=0?"+":"") << vg << "%\n"
               << "      Net throughput delta:  " << (ng>=0?"+":"") << ng << "%\n"
               << "      Jain Fairness (Veh):   MaxP=" << fixed << setprecision(3) << mp->jainFairnessVeh
               << "  CoLight=" << col->jainFairnessVeh
               << "  DELTA=" << (jv>=0?"+":"") << jv << "\n"
               << "      Jain Fairness (Net):   MaxP=" << mp->jainFairnessNet
               << "  CoLight=" << col->jainFairnessNet
               << "  DELTA=" << (jn>=0?"+":"") << jn << "\n";

            cout << ss.str();
            hypEval << ss.str();
        }
    }

    cout << "\nH3: Failure Recovery — pressure-based vs fixed (link B-C severed)\n\n";
    hypEval << "\nH3: Link Failure Recovery\n"
            << "    Hypothesis: Pressure-based controllers recover faster from link failure.\n\n";
    {
        for (double rho : {0.75, 1.20}) {
            string loadLabel = rho<1.0 ? "Medium" : "High";
            ostringstream ss;
            ss << fixed << setprecision(1);
            ss << "    Load=" << loadLabel << " (link B-C severed):\n";
            for (auto* ctrl : {"Fixed-Time","Max-Pressure","CoLight","Predictive"}) {
                auto* r = findResult(ctrl, rho, true);
                if (!r) continue;
                ss << "      " << setw(14) << left << ctrl
                   << "  PeakQ=" << setw(5) << (int)r->peakNetQueue
                   << "  MeanQ=" << setw(6) << r->meanNetQueue
                   << "  Recovery=" << (r->recoveryTicks==9999 ? "did not recover" : to_string(r->recoveryTicks)+" ticks")
                   << "  Throughput=" << r->netThroughputPerTick*1000 << " pkt/1000t\n";
            }
            cout << ss.str();
            hypEval << ss.str();
        }
    }

    cout << "\nH4: Predictive vs Max-Pressure — peak queue reduction\n\n";
    hypEval << "\nH4: Predictive vs Max-Pressure\n"
            << "    Hypothesis: Predictive anticipation reduces peak queue buildup.\n\n";
    {
        for (double rho : loads) {
            auto* mp  = findResult("Max-Pressure", rho, false);
            auto* pr  = findResult("Predictive",   rho, false);
            if (!mp || !pr) continue;
            string loadLabel = rho<0.5 ? "Low" : (rho<1.0 ? "Medium" : "High");

            double peakV = pct(pr->peakVehQueue, mp->peakVehQueue);
            double peakN = pct(pr->peakNetQueue, mp->peakNetQueue);
            double stdV  = pct(pr->vehQueueStddev, mp->vehQueueStddev);

            ostringstream ss;
            ss << fixed << setprecision(1);
            ss << "    Load=" << loadLabel << ":\n"
               << "      Peak Veh Queue:  MaxP=" << (int)mp->peakVehQueue
               << "  Pred=" << (int)pr->peakVehQueue
               << "  DELTA=" << (peakV>=0?"+":"") << peakV << "%\n"
               << "      Peak Net Queue:  MaxP=" << (int)mp->peakNetQueue
               << "  Pred=" << (int)pr->peakNetQueue
               << "  DELTA=" << (peakN>=0?"+":"") << peakN << "%\n"
               << "      Queue Stability (stddev): MaxP=" << fixed << setprecision(2) << mp->vehQueueStddev
               << "  Pred=" << pr->vehQueueStddev
               << "  DELTA=" << (stdV>=0?"+":"") << stdV << "%\n";

            cout << ss.str();
            hypEval << ss.str();
        }
    }

    // Overall winner table
    cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
    cout << "  OVERALL RANKING  (High Load, No Failure, Net Throughput)\n";
    cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";

    hypEval << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
    hypEval << "OVERALL RANKING (High Load, Normal Topology)\n";
    hypEval << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";

    struct RankEntry { string name; double netTP; double vehDelay; double jain; };
    vector<RankEntry> rank;
    for (auto& r : allResults)
        if (fabs(r.rho-1.20)<0.01 && !r.linkFailure)
            rank.push_back({r.controller, r.netThroughputPerTick, r.meanVehDelay, r.jainFairnessNet});
    sort(rank.begin(), rank.end(), [](auto& a, auto& b){ return a.netTP > b.netTP; });

    int pos=1;
    for (auto& e : rank) {
        ostringstream ss;
        ss << "  #" << pos++ << "  " << setw(14) << left << e.name
           << "  Net Throughput=" << fixed << setprecision(4) << e.netTP*1000 << "/1000t"
           << "  Mean Veh Delay=" << setprecision(1) << e.vehDelay << "t"
           << "  Jain Fairness=" << setprecision(3) << e.jain << "\n";
        cout << ss.str();
        hypEval << ss.str();
    }

    cout << "\n\nOutput files written to research_logs/:\n";
    cout << "  raw_ticks.csv       — " << 24*TICKS << " rows of per-tick telemetry\n";
    cout << "  summary.csv         — 24 aggregated scenario rows\n";
    cout << "  summary.json        — same as JSON for dashboards\n";
    cout << "  hypothesis_eval.txt — full hypothesis evaluation log\n\n";

    hypEval << "\nFiles: raw_ticks.csv (48000 rows), summary.csv (24 rows), summary.json\n";
    hypEval.close();

    return 0;
}
