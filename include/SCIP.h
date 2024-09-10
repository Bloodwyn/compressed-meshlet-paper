/*
Copyright (c) 2023 Bastian Kuth

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <MILP.h>
#include <scip/scip.h>

namespace scip {
    class SCIPSolver : public milp::MILPSolverBase {
    private:
        SCIP*                  scip_;
        std::vector<SCIP_VAR*> variables_;

    public:
        SCIPSolver();
        ~SCIPSolver() override;
        var    AddVariable(double min, double max, double objFactor) override;
        var    AddIntegerVariable(double min, double max, double objFactor) override;
        var    AddBinaryVariable(double objFactor) override;
        void   AddConstraint(const milp::LinearExpression& lhs, milp::Comparison cmp, double rhs) override;
        void   SetObjective(bool maximize) override;
        void   Optimize() override;
        double GetSolutionVariableValue(var v);
        var    AddVariableImpl(double min, double max, double objFactor, SCIP_VARTYPE type);
        void   SCIPthrow(SCIP_RETCODE code);
    };
}  // namespace compression
