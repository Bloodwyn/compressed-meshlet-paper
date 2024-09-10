/*
Copyright (c) 2023 Bastian Kuth

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "Gurobi.h"

namespace gurobi {
    GUROBISolver::GUROBISolver() : env_(true)
    {
        env_.set(GRB_IntParam_OutputFlag, 0);
        env_.set(GRB_IntParam_LogToConsole, 0);
        env_.set(GRB_IntParam_Threads, 1);
        env_.start();

        model_ = std::make_unique<GRBModel>(env_);
        model_->set(GRB_IntParam_LogToConsole, 0);
    }

    milp::Variable GUROBISolver::AddVariable(double min, double max, double objFactor)
    {
        return AddVariableImpl(min, max, objFactor, GRB_CONTINUOUS);
    }

    milp::Variable GUROBISolver::AddIntegerVariable(double min, double max, double objFactor)
    {
        return AddVariableImpl(min, max, objFactor, GRB_INTEGER);
    }

    milp::Variable GUROBISolver::AddBinaryVariable(double objFactor)
    {
        return AddVariableImpl(0., 1., objFactor, GRB_BINARY);
    }

    void GUROBISolver::AddConstraint(const milp::LinearExpression& lhs, milp::Comparison cmp, double rhs)
    {
        GRBLinExpr constraint;
        for (const auto& [v, factor] : lhs.GetExpression()) {
            constraint += factor * variables_[v];
        }
        char sense = GRB_EQUAL;
        if (cmp == milp::Comparison::GREATER_EQUAL) {
            sense = GRB_GREATER_EQUAL;
        } else if (cmp == milp::Comparison::LESS_EQUAL) {
            sense = GRB_LESS_EQUAL;
        }
        model_->addConstr(constraint, sense, rhs);
    };

    void GUROBISolver::SetObjective(bool maximize)
    {
        model_->setObjective(objective_, maximize ? GRB_MAXIMIZE : GRB_MINIMIZE);
    }

    void GUROBISolver::Optimize()
    {
        model_->optimize();
        int optimstatus = model_->get(GRB_IntAttr_Status);
        if (optimstatus != GRB_OPTIMAL) {
            throw std::exception("Could not find optimal solution!");
        }
    }

    double GUROBISolver::GetSolutionVariableValue(var v)
    {
        return variables_[v].get(GRB_DoubleAttr_X);
    }

    milp::Variable GUROBISolver::AddVariableImpl(double min, double max, double objFactor, char type)
    {
        variables_.emplace_back(model_->addVar(min, max, 0., type));
        if (objFactor != 0.) {
            objective_ += objFactor * variables_.back();
        }
        return variables_.size() - 1;
    }
}  // namespace gurobi