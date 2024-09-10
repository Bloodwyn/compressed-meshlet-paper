/*
Copyright (c) 2023 Bastian Kuth

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once
#include <unordered_map>

namespace milp {
    using Variable = std::int32_t;
    enum class Comparison { EQUAL, GREATER_EQUAL, LESS_EQUAL };

    class LinearExpression {
    private:
        std::unordered_map<Variable, double> expression_;

    public:
        LinearExpression() = default;
        LinearExpression(Variable v);
        LinearExpression(double factor, Variable v);
        LinearExpression&                           operator+=(const LinearExpression& other);
        const std::unordered_map<Variable, double>& GetExpression() const;
    };

    class MILPSolverBase {
    public:
        using var = std::int32_t;

        virtual ~MILPSolverBase()                                                             = default;
        virtual var    AddVariable(double min, double max, double objFactor)                  = 0;
        virtual var    AddIntegerVariable(double min, double max, double objFactor)           = 0;
        virtual var    AddBinaryVariable(double objFactor)                                    = 0;
        virtual void   AddConstraint(const LinearExpression& lhs, Comparison cmp, double rhs) = 0;
        virtual void   SetObjective(bool maximize)                                            = 0;
        virtual void   Optimize()                                                             = 0;
        virtual double GetSolutionVariableValue(var)                                          = 0;
    };
}  // namespace milp