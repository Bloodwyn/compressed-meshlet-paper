/*
Copyright (c) 2023 Bastian Kuth

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <MILP.h>

namespace milp {
    LinearExpression::LinearExpression(Variable v)
    {
        expression_.emplace(v, 1.0);
    }

    LinearExpression::LinearExpression(double factor, Variable v)
    {
        expression_.emplace(v, factor);
    }

    LinearExpression& LinearExpression::operator+=(const LinearExpression& other)
    {
        for (const auto& e : other.expression_) {
            auto [itr, suc] = expression_.emplace(e);
            if (!suc) {
                itr->second += e.second;  // e.g. 2x + 3x = 5x
            }
        }
        return *this;
    }

    const std::unordered_map<Variable, double>& LinearExpression::GetExpression() const
    {
        return expression_;
    }
}  // namespace milp