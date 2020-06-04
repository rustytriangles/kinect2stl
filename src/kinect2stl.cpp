// kinect2stl © 2020 RustyTriangles LLC

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

#include "libfreenect.h"
#include "libfreenect_sync.h"

using Vertex = std::array<float,3>;

struct Triangle {
    Vertex v0;
    Vertex v1;
    Vertex v2;
};

std::ostream& operator<<(std::ostream& str, const Vertex& v) {
    str << v[0] << " " << v[1] << " " << v[2];
    return str;
}

Vertex operator-(const Vertex& a, const Vertex& b) {
    const Vertex result = { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
    return result;
}

Vertex cross(const Vertex& a, const Vertex& b) {
    const Vertex result = { a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0] };
    return result;
}

Vertex normalized(const Vertex& a) {
    double len = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
    double scale = 1. / sqrt(len);
    const Vertex result = { static_cast<float>(a[0] * scale), static_cast<float>(a[1] * scale), static_cast<float>(a[2] * scale) };
    return result;
}

Vertex compute_normal(const Triangle& tri) {
    Vertex d1 = tri.v1 - tri.v0;
    Vertex d2 = tri.v2 - tri.v0;
    Vertex n = normalized(cross(d1, d2));
    return n;
}

float getDistanceAt(const unsigned short *depth,
                    const int x, const int y,
                    unsigned short minval, unsigned short maxval) {
    const int width = 640;
    const float num = 250.f;
    const float scale = num / static_cast<float>(maxval - minval);
    unsigned short raw = depth[y*width + x];
    return static_cast<float>(maxval - raw) * scale;
}

int main(int argc, char* argv[]) {

    char* rgb = nullptr;
    uint32_t ts;
    int ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB);
    if (ret < 0) {
        std::cerr << "No kinect found" << std::endl;
        exit(1);
    }

    unsigned short *depth = nullptr;
    ret = freenect_sync_get_depth((void**)&depth, &ts, 0, FREENECT_DEPTH_11BIT);
    if (ret < 0) {
        std::cerr << "Could not get depth" << std::endl;
        exit(1);
    }

    const int width = 640;
    const int height = 480;

    // determine Z scale
    unsigned short minval = 65535;
    unsigned short maxval = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            minval = std::min(minval, depth[y*width + x]);
            maxval = std::max(maxval, depth[y*width + x]);
        }
    }
    std::cout << "depth range [" << minval << ", " << maxval << "]" << std::endl;

    const float scale = 25.f / static_cast<float>(maxval - minval);

    // generate triangles
    std::vector<Triangle> tris;

    // the front
    for (int y = 0; y < height - 1; ++y) {
        const float y0 = static_cast<float>(y);
        const float y1 = y0 + 1.f;
        for (int x = 0; x < width - 1; ++x) {
            const float x0 = static_cast<float>(x);
            const float x1 = x0 + 1.f;
            const float v00 = getDistanceAt(depth, x  , y  , minval, maxval);
            const float v10 = getDistanceAt(depth, x+1, y  , minval, maxval);
            const float v01 = getDistanceAt(depth, x  , y+1, minval, maxval);
            const float v11 = getDistanceAt(depth, x+1, y+1, minval, maxval);

            Triangle t0;
            t0.v0 = {x0, y0, v00};
            t0.v1 = {x1, y0, v10};
            t0.v2 = {x1, y1, v11};
            tris.push_back(t0);

            Triangle t1;
            t1.v0 = {x0, y0, v00};
            t1.v1 = {x1, y1, v11};
            t1.v2 = {x0, y1, v01};
            tris.push_back(t1);
        }
    }

    const float back = -3.f;
    // the bottom
    for (int x = 0; x < width - 1; ++x) {
        const int yi = 0;
        const float yf = static_cast<float>(yi);
        const float x0 = static_cast<float>(x);
        const float x1 = x0 + 1.f;
        const float v0 = getDistanceAt(depth, x  , yi, minval, maxval);
        const float v1 = getDistanceAt(depth, x+1, yi, minval, maxval);

        Triangle t0;
        t0.v0 = {x0, yf, back};
        t0.v1 = {x1, yf, back};
        t0.v2 = {x1, yf, v1};
        tris.push_back(t0);

        Triangle t1;
        t1.v0 = {x0, yf, back};
        t1.v1 = {x1, yf, v1};
        t1.v2 = {x0, yf, v0};
        tris.push_back(t1);
    }
    // the right side
    for (int y = 0; y < height - 1; ++y) {
        const int xi = width-1;
        const float xf = static_cast<float>(xi);
        const float y0 = static_cast<float>(y);
        const float y1 = y0 + 1.f;
        const float v0 = getDistanceAt(depth, xi, y  , minval, maxval);
        const float v1 = getDistanceAt(depth, xi, y+1, minval, maxval);

        Triangle t0;
        t0.v0 = {xf, y0, back};
        t0.v1 = {xf, y1, back};
        t0.v2 = {xf, y1, v1};
        tris.push_back(t0);

        Triangle t1;
        t1.v0 = {xf, y0, back};
        t1.v1 = {xf, y1, v1};
        t1.v2 = {xf, y0, v0};
        tris.push_back(t1);
    }
    // the top
    for (int x = width-1; x > 0; --x) {
        const int yi = height - 1;
        const float yf = static_cast<float>(yi);
        const float x0 = static_cast<float>(x);
        const float x1 = x0 - 1.f;
        const float v0 = getDistanceAt(depth, x  , yi, minval, maxval);
        const float v1 = getDistanceAt(depth, x-1, yi, minval, maxval);

        Triangle t0;
        t0.v0 = {x0, yf, back};
        t0.v1 = {x1, yf, back};
        t0.v2 = {x1, yf, v1};
        tris.push_back(t0);

        Triangle t1;
        t1.v0 = {x0, yf, back};
        t1.v1 = {x1, yf, v1};
        t1.v2 = {x0, yf, v0};
        tris.push_back(t1);
    }
    // the left side
    for (int y = height - 1; y > 0; --y) {
        const int xi = 0;
        const float xf = static_cast<float>(xi);
        const float y0 = static_cast<float>(y);
        const float y1 = y0 - 1.f;
        const float v0 = getDistanceAt(depth, xi, y  , minval, maxval);
        const float v1 = getDistanceAt(depth, xi, y-1, minval, maxval);

        Triangle t0;
        t0.v0 = {xf, y0, back};
        t0.v1 = {xf, y1, back};
        t0.v2 = {xf, y1, v1};
        tris.push_back(t0);

        Triangle t1;
        t1.v0 = {xf, y0, back};
        t1.v1 = {xf, y1, v1};
        t1.v2 = {xf, y0, v0};
        tris.push_back(t1);
    }
    // the back
    for (int y = 0; y < height - 1; ++y) {
        const float y0 = static_cast<float>(y);
        const float y1 = y0 + 1.f;
        for (int x = 0; x < width - 1; ++x) {
            const float x0 = static_cast<float>(x);
            const float x1 = x0 + 1.f;

            Triangle t0;
            t0.v0 = {x0, y0, back};
            t0.v1 = {x1, y1, back};
            t0.v2 = {x1, y0, back};
            tris.push_back(t0);

            Triangle t1;
            t1.v0 = {x0, y0, back};
            t1.v1 = {x0, y1, back};
            t1.v2 = {x1, y1, back};
            tris.push_back(t1);
        }
    }

    // write triangles to STL file
    const bool ascii = false;
    if (ascii) {
        std::ofstream ofs("kinect.stl", std::ofstream::out);
        ofs << "solid kinect" << std::endl;
        for (const Triangle& t : tris) {
            ofs << "facet normal" << compute_normal(t) << std::endl;
            ofs << "outer loop" << std::endl;
            ofs << "  vertex " << t.v0 << std::endl;
            ofs << "  vertex " << t.v1 << std::endl;
            ofs << "  vertex " << t.v2 << std::endl;
            ofs << "endloop" << std::endl;
        }
        ofs << "endsolid kinect" << std::endl;
        ofs.close();
    } else {
        std::ofstream ofs("kinect.stl", std::ofstream::out | std::ofstream::binary);
        const std::vector<char> x(80, 'x');
        ofs.write(&x[0], 80);

        uint32_t numTri = tris.size();
        ofs.write((char*)&numTri, sizeof(uint32_t));

        unsigned short attr_count = 0;
        for (const Triangle& t : tris) {
            Vertex n = compute_normal(t);
            ofs.write((char*)&n[0], 3*sizeof(float));
            ofs.write((char*)&t.v0[0], 3*sizeof(float));
            ofs.write((char*)&t.v1[0], 3*sizeof(float));
            ofs.write((char*)&t.v2[0], 3*sizeof(float));
            ofs.write((char*)&attr_count, sizeof(unsigned short));
        }
        ofs.close();
    }
}
