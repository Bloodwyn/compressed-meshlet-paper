/*
Copyright (c) 2023 Bastian Kuth

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <MILP.h>
#include <OptimalStrips.h>

#include <algorithm>
#include <cassert>
#include <deque>
#include <memory>
#include <unordered_map>

#ifdef HAS_GUROBI
#include <Gurobi.h>
#else
#include <SCIP.h>
#endif

namespace optimal_strips {
    using Edge = std::pair<int, int>;  // and Edge consits of two vertex indices

    struct EdgeHash {
        std::size_t operator()(const Edge& edge) const
        {
            const auto hasher = std::hash<int>();
            return hasher(edge.first) ^ hasher(edge.second);  // as an edge is not directed, xor is sufficient
        }
    };

    // maps an edge to the two triangles that connect to it. If border, rtiangle is -1
    using EdgeMap = std::unordered_map<Edge, std::pair<TriangleId, TriangleId>, EdgeHash>;

    EdgeMap ComputeEdgeMap(std::span<const int> indices)
    {
        EdgeMap result;
        for (int t = 0; t < indices.size() / 3; ++t) {
            for (int corner = 0; corner < 3; ++corner) {
                const auto nextCorner = (corner + 1) % 3;

                const Edge edge       = {indices[3 * t + corner], indices[3 * t + nextCorner]};
                const Edge uniqueEdge = std::minmax(edge.first, edge.second);
                const auto& [it, _]   = result.emplace(uniqueEdge, std::make_pair(-1, -1));

                auto& [left, right] = it->second;

                // edge runs counter-clockwise around primitive
                //
                // * <--- *
                // |      ^
                // | prim |
                // v      |
                // * ---> *
                //
                // Primitive is on left side of edge.
                // If edge was flipped to create unique edge, primitive is on right side of edge

                if (edge.first < edge.second) {
                    left = t;
                } else {
                    right = t;
                }
            }
        }

        return result;
    }

    // helper function to quickly navigate the dual graph
    int EdgeMapOther(const std::pair<TriangleId, TriangleId>& pair, TriangleId cmp)
    {
        if (pair.first == cmp) {
            return pair.second;
        }
        return pair.first;
    }

    class StripOptimizer {
    public:
    protected:
        std::span<const int>                           indices_;
        EdgeMap                                        edgeMap_;
        std::vector<Edge>                              dualEdgeIdToEdge_;
        std::unordered_map<Edge, int, EdgeMap::hasher> edgeToDualEdgeId_;

        std::unique_ptr<milp::MILPSolverBase>                  solver_;
        std::vector<milp::Variable>                            x_;
        std::vector<std::pair<milp::Variable, milp::Variable>> y_;

    public:
        StripOptimizer(std::span<const int> indices, std::unique_ptr<milp::MILPSolverBase>&& optimizer)
            : solver_(std::move(optimizer)), indices_(indices), edgeMap_(ComputeEdgeMap(indices))
        {
            for (const auto& [key, triangles] : edgeMap_) {
                const auto& [left, right] = triangles;

                if (left >= 0 && right >= 0) {
                    edgeToDualEdgeId_.emplace(key, static_cast<int>(dualEdgeIdToEdge_.size()));
                    dualEdgeIdToEdge_.emplace_back(key);
                }
            }
        }

        std::vector<TriangleStrip> operator()()
        {
            CreateVariables();
            CreateConstraints();
            solver_->SetObjective(true);
            std::vector<bool> edgeIsInStrip = Run();
            return ExtractStrips(edgeIsInStrip);
        }

    protected:
        void CreateVariables()
        {
            x_.reserve(dualEdgeIdToEdge_.size());
            for (int i = 0; i < dualEdgeIdToEdge_.size(); i++) {
                x_.emplace_back(solver_->AddBinaryVariable(1.0));
            }

            y_.reserve(dualEdgeIdToEdge_.size());
            for (int i = 0; i < dualEdgeIdToEdge_.size(); i++) {
                y_.emplace_back(std::make_pair(solver_->AddVariable(0.0, std::numeric_limits<double>::max(), 0.0),
                                               solver_->AddVariable(0.0, std::numeric_limits<double>::max(), 0.0)));
            }
        }

        void CreateConstraints()
        {
            const double F       = 1.;
            const double epsilon = 0.0001;

            for (int t = 0; t < indices_.size() / 3; ++t) {
                milp::LinearExpression sumX;
                milp::LinearExpression sumY;

                int numberOfDualEdges = 0;
                for (int i = 0; i < 3; i++) {
                    Edge uniqueEdge = std::minmax(indices_[3 * t + i], indices_[3 * t + ((i + 1) % 3)]);

                    auto itr = edgeToDualEdgeId_.find(uniqueEdge);
                    if (itr == edgeToDualEdgeId_.end()) {
                        continue;
                    }
                    ++numberOfDualEdges;

                    sumX += x_[edgeToDualEdgeId_.at(uniqueEdge)];

                    auto faces   = edgeMap_.at(uniqueEdge);
                    auto edgeIdx = edgeToDualEdgeId_.at(uniqueEdge);
                    if (faces.first == t) {
                        sumY += y_[edgeIdx].first;
                    } else if (faces.second == t) {
                        sumY += y_[edgeIdx].second;
                    }
                }
                if (numberOfDualEdges == 3) {
                    // anti-fork constraint
                    solver_->AddConstraint(sumX, milp::Comparison::LESS_EQUAL, 2.0);
                }
                // anti-cycle constraint per triangle
                solver_->AddConstraint(sumY, milp::Comparison::LESS_EQUAL, F - epsilon);
            }

            for (int eIdx = 0; eIdx < dualEdgeIdToEdge_.size(); eIdx++) {
                milp::LinearExpression e;
                e += milp::LinearExpression(-F, x_[eIdx]);
                e += y_[eIdx].first;
                e += y_[eIdx].second;
                // anti-cycle constraint per edge
                solver_->AddConstraint(e, milp::Comparison::EQUAL, 0.0);
            }
        }

        std::vector<bool> Run()
        {
            solver_->Optimize();
            std::vector<bool> edgeIsInStrip;
            for (const auto& v : x_) {
                // solvers sometimes output booleans 0 <= b <= 1...
                edgeIsInStrip.emplace_back(solver_->GetSolutionVariableValue(v) > 0.5);
            }
            return edgeIsInStrip;
        }

        void Visit(TriangleId               t,
                   bool                     front,
                   std::deque<TriangleId>&  strip,
                   std::vector<bool>&       visitedTriangles,
                   const std::vector<bool>& edgeIsInStrip)
        {
            TriangleId oldEnd = -1;
            if (strip.size() != 1) {
                if (front) {
                    oldEnd = *std::next(strip.begin());
                } else {
                    oldEnd = *std::prev(std::prev(strip.end()));
                }
            }

            visitedTriangles[t] = true;
            int nVisited        = 0;
            for (int i = 0; i < 3; ++i) {
                const auto uniqueEdge = std::minmax(indices_[3 * t + i], indices_[3 * t + ((i + 1) % 3)]);
                auto       other      = EdgeMapOther(edgeMap_.at(uniqueEdge), t);

                auto itr = edgeToDualEdgeId_.find(uniqueEdge);
                if (itr == edgeToDualEdgeId_.end()) {
                    continue;
                }
                auto dualEdgeId = itr->second;
                if (!edgeIsInStrip[dualEdgeId]) {
                    continue;
                }
                if (other != oldEnd) {
                    assert(nVisited == 0);
                    if (visitedTriangles[other]) {
                        auto side = (front ? strip.back() : strip.front());
                        assert(side == other);
                        throw std::exception("Strip contains circles!");
                    }
                    if (front) {
                        strip.push_front(other);
                    } else {
                        strip.push_back(other);
                    }
                    Visit(other, front, strip, visitedTriangles, edgeIsInStrip);
                    if (oldEnd == -1) {
                        return;
                    }
                    ++nVisited;
                }
            }
        }

        std::vector<TriangleStrip> ExtractStrips(const std::vector<bool>& edgeIsInStrip)
        {
            std::vector<std::deque<TriangleId>> triangleStrips;
            std::vector<bool>                   visitedTriangles(indices_.size() / 3, false);

            for (int f = 0; f < indices_.size() / 3; ++f) {
                if (visitedTriangles[f]) {
                    continue;
                }
                std::deque<TriangleId> strip;
                strip.push_back(f);
                Visit(f, false, strip, visitedTriangles, edgeIsInStrip);
                Visit(f, true, strip, visitedTriangles, edgeIsInStrip);
                triangleStrips.emplace_back(std::move(strip));
            }

            std::vector<TriangleStrip> ret;
            ret.reserve(triangleStrips.size());

            for (const auto& s : triangleStrips) {
                ret.emplace_back(s.begin(), s.end());
            }
            return ret;
        }
    };

    std::vector<TriangleStrip> CreateTriangleStrips(std::span<const int> triangles)
    {
#ifdef HAS_GUROBI
        return StripOptimizer(triangles, std::make_unique<gurobi::GUROBISolver>())();
#else
        return StripOptimizer(triangles, std::make_unique<scip::SCIPSolver>())();
#endif
    }
}  // namespace optimal_strips
