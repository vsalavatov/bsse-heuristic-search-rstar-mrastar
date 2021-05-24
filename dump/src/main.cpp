#include "open_dump.h"
#include "map.h"
#include "scenario.h"
#include "astar.h"
#include "rstar.h"
#include "mrastar.h"
#include "rss_check.h"

#include <cxxopts.hpp>
#include <exception>
#include <fstream>
#include <chrono>



int main(int argc, char* argv[]) {
    
    cxxopts::Options options{"dump", "Dump heuristic search algorithm statistics"};
    options.add_options()
        ("out", "Output file path", cxxopts::value<std::filesystem::path>())
        ("map", "Map file path", cxxopts::value<std::filesystem::path>())
        ("full", "Log all actions (for visualization)", cxxopts::value<bool>()->default_value("false"))
        ("astar", "Use A* algorithm")
        ("rstar", "Use R* algorithm")
        ("mrastar", "Use MRA* algorithm")
        ("seed", "R*/MRA* random seed", cxxopts::value<std::size_t>()->default_value("239"))
        ("astar_weight", "A* heuristic weight", cxxopts::value<double>())
        ("rstar_delta", "R* delta", cxxopts::value<double>())
        ("rstar_k", "R* k", cxxopts::value<std::size_t>())
        ("rstar_weight", "R* weight", cxxopts::value<double>())
        ("rstar_ti_factor", "R* threshold inflation factor", cxxopts::value<double>()->default_value("5.0"))
        ("rstar_smart_iters_coef", "R* smart iters coef", cxxopts::value<std::size_t>()->default_value("8"))
        ("rstar_range_low_coef", "R* range low coef", cxxopts::value<double>()->default_value("0"))
        ("mrastar_cell_sizes", "MRA* cell sizes", cxxopts::value<std::vector<std::size_t>>())
        ("mrastar_weight", "MRA* weight", cxxopts::value<double>())
        ("mrastar_suboptimality_coef", "MRA* suboptimality coef", cxxopts::value<double>())
        ("mrastar_choose_queue_method", "MRA* meta algorithm (round_robin/meta_astar/dts)", cxxopts::value<std::string>()->default_value("round_robin"))
        ("mrastar_thompson_history_coef", "MRA* history coef in DTS", cxxopts::value<double>()->default_value("10"))
        ("start", "Start position row and column", cxxopts::value<std::string>())
        ("goal", "Goal position row and column", cxxopts::value<std::string>())
        ("heuristic", "Heuristic function (euclidean/octile)", cxxopts::value<std::string>()->default_value("octile"))
        ("dist", "Distance function (euclidean/octile)", cxxopts::value<std::string>()->default_value("euclidean"))
        ;

    cxxopts::ParseResult parseResult;
    parseResult = options.parse(argc, argv);

    auto parsePosition = [&](const std::string& name) -> heuristicsearch::Position {
        heuristicsearch::Position pos{};
        std::istringstream is{parseResult[name].as<std::string>()};
        is >> pos.row >> pos.col;
        return pos;
    };
    auto parseMetric = [&](const std::string& name) -> heuristicsearch::Metric {
        auto metric = parseResult[name].as<std::string>();
        if (metric == "euclidean") {
            return heuristicsearch::EuclideanDistance;
        }
        if (metric == "octile") {
            return heuristicsearch::OctileDistance;
        }
        throw std::invalid_argument("unknown metric: " + metric);
    };

    assert(parseResult["out"].count());
    auto out = parseResult["out"].as<std::filesystem::path>();
    assert(parseResult["map"].count());
    auto mapPath = parseResult["map"].as<std::filesystem::path>();
    auto map = heuristicsearch::Map::fromMovingAI(mapPath);
    assert(parseResult["start"].count());
    auto startPos = parsePosition("start");
    assert(parseResult["goal"].count());
    auto goalPos = parsePosition("goal");
    auto heuristic = parseMetric("heuristic");
    auto dist = parseMetric("dist");

    std::ofstream os{out};
    DumpInfo::setOutputStream(&os);
    DumpInfo::logAll(parseResult["full"].as<bool>());

    os << std::filesystem::absolute(mapPath) << "\n"
       << "start " << startPos.row << " " << startPos.col << "\n"
       << "goal " << goalPos.row << " " << goalPos.col << "\n";

    std::optional<heuristicsearch::HeuristicAlgoResult> optres;
    
    // std::cerr << getCurrentRSS() << ' ' << getPeakRSS() << '\n';
    // fill the memory so we can measure actual memory consumption through the difference of peak values
    // this thing sometimes doesn't work :(

    /*
    constexpr auto PAGE_SIZE = 4096;
    auto memdiff = getPeakRSS() - getCurrentRSS();
    if (memdiff > PAGE_SIZE) 
        memdiff -= PAGE_SIZE;
    auto memfill = reinterpret_cast<char*>(malloc(memdiff));
    for (size_t i = 0; i < memdiff; i++)
        memfill[i] = i % 256;
    memdiff = getPeakRSS() - getCurrentRSS();
    if (memdiff > 16 * PAGE_SIZE) {
        std::cerr << "Failed to set current rss to peak rss. Diff: " << memdiff << '\n';
        return 1;
    }*/ 

    const size_t rssStart = getPeakRSS();
    using namespace std::chrono;
    const auto timeStart = high_resolution_clock::now();
    if (parseResult.count("rstar")) {
        assert(parseResult["rstar_delta"].count());
        auto delta = parseResult["rstar_delta"].as<double>();
        assert(parseResult["rstar_k"].count());
        auto k = parseResult["rstar_k"].as<std::size_t>();
        assert(parseResult["rstar_weight"].count());
        auto weight = parseResult["rstar_weight"].as<double>();
        auto thresholdInflationFactor = parseResult["rstar_ti_factor"].as<double>();
        auto smartItersCoef = parseResult["rstar_smart_iters_coef"].as<std::size_t>();
        auto rangeLowCoef = parseResult["rstar_range_low_coef"].as<double>();
        auto randomSeed = parseResult["seed"].as<std::size_t>();

        auto rstar = RStar<OpenSetDump<heuristicsearch::Position, std::pair<int, double>>>{
            delta, k, weight, thresholdInflationFactor, smartItersCoef, rangeLowCoef, randomSeed
        };
        optres = rstar(map, startPos, goalPos, heuristic, dist);
    } else if (parseResult.count("mrastar")) {
        assert(parseResult["mrastar_cell_sizes"].count());
        auto cellSizes = parseResult["mrastar_cell_sizes"].as<std::vector<std::size_t>>();
        assert(parseResult["mrastar_weight"].count());
        auto weight = parseResult["mrastar_weight"].as<double>();
        assert(parseResult["mrastar_suboptimality_coef"].count());
        auto suboptimalityCoef = parseResult["mrastar_suboptimality_coef"].as<double>();
        auto chooseQueueMethod = parseResult["mrastar_choose_queue_method"].as<std::string>();
        auto thompsonHistoryCoef = parseResult["mrastar_thompson_history_coef"].as<double>();
        auto randomSeed = parseResult["seed"].as<std::size_t>();

        if (chooseQueueMethod == "dts") {
            auto mrastar = MRAStar<DynamicThompsonSamplingMethod, OpenSetDump<heuristicsearch::Position, double>>{
                std::set<int>{cellSizes.begin(), cellSizes.end()},
                weight,
                suboptimalityCoef,
                DynamicThompsonSamplingMethod{thompsonHistoryCoef, randomSeed}
            };
            optres = mrastar(map, startPos, goalPos, heuristic, dist);
        } else if (chooseQueueMethod == "meta_astar") {
            auto mrastar = MRAStar<MetaAStarMethod, OpenSetDump<heuristicsearch::Position, double>>{
                std::set<int>{cellSizes.begin(), cellSizes.end()},
                weight,
                suboptimalityCoef
            };
            optres = mrastar(map, startPos, goalPos, heuristic, dist);
        } else {
            auto mrastar = MRAStar<RoundRobinMethod, OpenSetDump<heuristicsearch::Position, double>>{
                std::set<int>{cellSizes.begin(), cellSizes.end()},
                weight,
                suboptimalityCoef
            };
            optres = mrastar(map, startPos, goalPos, heuristic, dist);
        }
    } else {
        auto weight = parseResult["astar_weight"].as<double>();
        optres = heuristicsearch::AStar<OpenSetDump<heuristicsearch::Position, double>>(
            map, startPos, goalPos, 
            [&heuristic, weight](const auto &a, const auto &b) { return weight * heuristic(a, b); }, 
            dist
        );
    }
    const size_t rssFinish = getPeakRSS();
    const auto timeFinish = high_resolution_clock::now();
    os << "memory " << (rssFinish - rssStart) << '\n';
    os << "time " << std::setprecision(6) << duration_cast<milliseconds>(timeFinish - timeStart).count() / 1000.0 << '\n';

    if (!optres.has_value()) {
        os << "no path\n";
    } else {
        auto result = *optres;
        os << "expansions " << result.expansions << '\n';
        os << "distance " << std::setprecision(10) << result.distance << '\n';
        os << "path " << result.path.size() << "\n";        
        for (const auto& p : result.path) {
            os << p.row << " " << p.col << "\n";
        }
    }

    os.flush();
    os.close();

    return 0;
}
