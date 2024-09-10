/*
Copyright (c) 2023 Bastian Kuth

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <OptimalStrips.h>

#include <fstream>
#include <iostream>

int main()
{
    static const int indices[] = {
        0,  1,  2,  0,  3,  1,  2,  1,  4,  2,  4,  5,  4,  6,  5,  4,  7,  6,  8,  7,  4,  1,  8,  4,  9,  8,  1,  3,
        9,  1,  9,  10, 8,  3,  11, 9,  9,  11, 10, 10, 11, 12, 3,  13, 11, 13, 12, 11, 14, 13, 3,  14, 3,  0,  15, 14,
        0,  16, 15, 0,  14, 17, 13, 13, 18, 12, 13, 17, 18, 18, 19, 12, 15, 20, 14, 20, 17, 14, 21, 18, 17, 21, 19, 18,
        19, 21, 22, 23, 22, 21, 20, 24, 17, 25, 21, 17, 25, 17, 24, 26, 20, 15, 27, 15, 16, 26, 15, 27, 28, 27, 16, 26,
        27, 28, 29, 26, 28, 30, 26, 29, 30, 31, 26, 26, 31, 32, 26, 32, 20, 20, 32, 33, 33, 24, 20, 34, 24, 33, 25, 24,
        34, 35, 21, 25, 35, 25, 34, 35, 23, 21, 35, 34, 36, 36, 34, 37, 38, 23, 35, 39, 23, 38, 40, 35, 36, 40, 36, 37,
        41, 40, 37, 42, 38, 35, 42, 35, 40, 43, 39, 38, 43, 38, 42, 39, 43, 44, 45, 40, 41, 45, 42, 40, 45, 41, 46, 46,
        41, 47, 48, 43, 42, 48, 42, 45, 48, 49, 43, 50, 45, 46, 51, 48, 45, 51, 49, 48, 52, 46, 47, 50, 46, 52, 52, 47,
        53, 52, 53, 54, 55, 52, 54, 56, 45, 50, 56, 51, 45, 43, 57, 44, 49, 57, 43, 58, 56, 50, 51, 59, 49, 59, 57, 49,
        56, 60, 51, 60, 56, 58, 61, 50, 52, 58, 50, 61, 60, 58, 61, 51, 62, 59, 60, 62, 51};
    static const int triangleCount = sizeof(indices) / sizeof(int) / 3;

    float positions[] = {
        -0.018724, -0.547756, -0.171450, -0.493603, -0.153312, -0.633638, -0.102638, -0.397046, -0.287279, -0.525062,
        -0.279559, -0.620039, -0.419381, -0.567800, -0.434716, -0.426210, -0.322745, -0.378523, -0.201855, -0.367964,
        -0.314367, -0.223810, -0.182653, -0.298800, -0.199972, -0.189119, -0.094796, -0.262013, 0.002658,  -0.349853,
        0.093996,  -0.430733, 0.127582,  -0.555152, 0.015896,  -0.185885, -0.092194, -0.146174, -0.162770, -0.091021,
        0.137440,  -0.273461, -0.068836, -0.035299, -0.212919, .005226,   -0.134004, .087667,   0.130539,  -0.145652,
        0.065539,  -0.063524, 0.236408,  -0.400095, 0.164297,  -0.476381, 0.236267,  -0.535171, 0.402891,  -0.555584,
        0.376801,  -0.404859, 0.336367,  -0.271120, 0.235853,  -0.273980, 0.244601,  -0.167007, 0.177370,  -0.036208,
        0.030321,  .089560,   0.147565,  .068598,   0.239760,  .066307,   -0.099708, .179149,   -0.204720, .192734,
        0.104900,  .187947,   0.177594,  .229913,   -0.025930, .272236,   -0.154790, .338312,   -0.300979, .360302,
        0.050467,  .365086,   0.161554,  .371356,   0.254986,  .297181,   -0.078220, .403209,   -0.155537, .471899,
        0.113098,  .487474,   -0.058746, .510765,   0.250904,  .482753,   0.352940,  .370912,   0.358836,  .481216,
        0.385340,  .601315,   0.027031,  .501824,   -0.274399, .493706,   0.063328,  .550040,   -0.161576, .576624,
        0.018037,  .602629,   0.135963,  .610810,   -0.060785, .622164};
    static const int positionCount = sizeof(positions) / sizeof(float) / 2;

    const auto strips = optimal_strips::CreateTriangleStrips(std::span(indices, 3 * triangleCount));

    std::ofstream obj("demo.obj");

    // write positions
    for (int i = 0; i < positionCount; ++i) {
        obj << "v " << positions[2 * i + 0] << ' ' << positions[2 * i + 1] << " 0\n";
    }

    // write triangles
    obj << "o meshlet\n";
    for (int i = 0; i < triangleCount; ++i) {
        // compute the triangle center for visualizing the strip later
        float avgx = 0.f;
        float avgy = 0.f;
        obj << "f ";
        for (int j = 0; j < 3; ++j) {
            int index = indices[3 * i + j];
            obj << index + 1 << ' ';  // objs start counting at 1
            avgx += positions[2 * index + 0];
            avgy += positions[2 * index + 1];
        }
        obj << "\n";
        // write triangle center
        obj << "v " << avgx / 3.f << ' ' << avgy / 3.f << " 0\n";
    }

    // write strips
    obj << "o strips\n";
    for (const auto& strip : strips) {        
        for (int i = 1; i < strip.size(); ++i) {            
            obj << "l " << strip[i - 1] + 1 + positionCount << ' ' << strip[i] + 1 + positionCount << '\n';
        }
    }

    obj.close();
    return 0;
}