/*
Copyright (c) 2023 Bastian Kuth

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once
#include <MILP.h>
#include <gurobi_c++.h>

namespace gurobi {
    class GUROBISolver : public milp::MILPSolverBase {
        friend class StripOptimizerLazyGurobi;

    private:
        GRBEnv                    env_;
        std::unique_ptr<GRBModel> model_;
        std::vector<GRBVar>       variables_;
        GRBLinExpr                objective_;

    public:
        GUROBISolver();

        var    AddVariable(double min, double max, double objFactor) override;
        var    AddIntegerVariable(double min, double max, double objFactor) override;
        var    AddBinaryVariable(double objFactor) override;
        void   AddConstraint(const milp::LinearExpression& lhs, milp::Comparison cmp, double rhs) override;
        void   SetObjective(bool maximize) override;
        void   Optimize() override;
        double GetSolutionVariableValue(var v);

    private:
        var AddVariableImpl(double min, double max, double objFactor, char type);
    };
}  // namespace gurobi
