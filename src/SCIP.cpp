/*
Copyright (c) 2023 Bastian Kuth

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <SCIP.h>
#include <scip/scipdefplugins.h>

namespace scip {
    SCIPSolver::SCIPSolver()
    {
        SCIPthrow(SCIPcreate(&scip_));
        SCIPthrow(SCIPincludeDefaultPlugins(scip_));
        SCIPmessagehdlrSetQuiet(SCIPgetMessagehdlr(scip_), TRUE);
        SCIPthrow(SCIPcreateProbBasic(scip_, ""));
    }

    SCIPSolver::~SCIPSolver()
    {
        SCIPfree(&scip_);
    }

    milp::Variable SCIPSolver::AddVariable(double min, double max, double objFactor)
    {
        return AddVariableImpl(min, max, objFactor, SCIP_VARTYPE_CONTINUOUS);
    }

    milp::Variable SCIPSolver::AddIntegerVariable(double min, double max, double objFactor)
    {
        return AddVariableImpl(min, max, objFactor, SCIP_VARTYPE_INTEGER);
    }

    milp::Variable SCIPSolver::AddBinaryVariable(double objFactor)
    {
        return AddVariableImpl(0.0, 1.0, objFactor, SCIP_VARTYPE_BINARY);
    }

    void SCIPSolver::AddConstraint(const milp::LinearExpression& lhs, milp::Comparison cmp, double rhs)
    {
        SCIP_CONS* constraint = nullptr;

        // =
        double     min        = rhs;
        double     max        = rhs;
        if (cmp == milp::Comparison::GREATER_EQUAL) {  // >=
            max = SCIPinfinity(scip_);
        } else if (cmp == milp::Comparison::LESS_EQUAL) {  // <=
            min = -SCIPinfinity(scip_);
        }

        SCIPthrow(SCIPcreateConsBasicLinear(scip_, &constraint, "", 0, nullptr, nullptr, min, max));
        for (const auto [v, factor] : lhs.GetExpression()) {
            SCIPthrow(SCIPaddCoefLinear(scip_, constraint, variables_[v], factor));
        }
        SCIPthrow(SCIPaddCons(scip_, constraint));
    };

    void SCIPSolver::SetObjective(bool maximize)
    {
        SCIPthrow(SCIPsetObjsense(scip_, maximize ? SCIP_OBJSENSE_MAXIMIZE : SCIP_OBJSENSE_MINIMIZE));
    }

    void SCIPSolver::Optimize()
    {
        SCIPthrow(SCIPsolve(scip_));
    }

    double SCIPSolver::GetSolutionVariableValue(var v)
    {
        auto sol = SCIPgetBestSol(scip_);
        return SCIPgetSolVal(scip_, sol, variables_[v]);
    }

    milp::Variable SCIPSolver::AddVariableImpl(double min, double max, double objFactor, SCIP_VARTYPE type)
    {
        variables_.emplace_back();
        SCIPthrow(SCIPcreateVarBasic(scip_, &variables_.back(), "", min, max, objFactor, type));
        SCIPthrow(SCIPaddVar(scip_, variables_.back()));
        return variables_.size() - 1;
    }

    void SCIPSolver::SCIPthrow(SCIP_RETCODE code)
    {
        if (code != SCIP_OKAY) {
            throw std::exception("SCIP failed");
        }
    }
}  // namespace scip
