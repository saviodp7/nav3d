#pragma once

#include <cstdlib>

namespace nav3d_util {

class LineIterator {
  public:
    LineIterator(int x0, int y0, int x1, int y1)
        : x0_(x0), y0_(y0), x1_(x1), y1_(y1), x_(x0), y_(y0), deltax_(std::abs(x1 - x0)), deltay_(std::abs(y1 - y0)),
          curpixel_(0) {
        if (x1_ >= x0_) {
            xinc1_ = 1;
            xinc2_ = 1;
        } else {
            xinc1_ = -1;
            xinc2_ = -1;
        }

        if (y1_ >= y0_) {
            yinc1_ = 1;
            yinc2_ = 1;
        } else {
            yinc1_ = -1;
            yinc2_ = -1;
        }

        if (deltax_ >= deltay_) {
            xinc1_ = 0;
            yinc2_ = 0;
            den_ = deltax_;
            num_ = deltax_ / 2;
            numadd_ = deltay_;
            numpixels_ = deltax_;
        } else {
            xinc2_ = 0;
            yinc1_ = 0;
            den_ = deltay_;
            num_ = deltay_ / 2;
            numadd_ = deltax_;
            numpixels_ = deltay_;
        }
    }

    bool isValid() const { return curpixel_ <= numpixels_; }

    void advance() {
        num_ += numadd_;
        if (num_ >= den_) {
            num_ -= den_;
            x_ += xinc1_;
            y_ += yinc1_;
        }
        x_ += xinc2_;
        y_ += yinc2_;
        ++curpixel_;
    }

    int getX() const { return x_; }
    int getY() const { return y_; }

    int getX0() const { return x0_; }
    int getY0() const { return y0_; }

    int getX1() const { return x1_; }
    int getY1() const { return y1_; }

  private:
    int x0_;
    int y0_;
    int x1_;
    int y1_;

    int x_;
    int y_;

    int deltax_;
    int deltay_;

    int curpixel_;

    int xinc1_{0};
    int xinc2_{0};
    int yinc1_{0};
    int yinc2_{0};
    int den_{0};
    int num_{0};
    int numadd_{0};
    int numpixels_{0};
};

} // namespace nav3d_util
