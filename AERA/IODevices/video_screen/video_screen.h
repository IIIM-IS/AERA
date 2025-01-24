//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2022-2025 Jeff Thompson
//_/_/ Copyright (c) 2022-2025 Icelandic Institute for Intelligent Machines
//_/_/ http://www.iiim.is
//_/_/
//_/_/ --- Open-Source BSD License, with CADIA Clause v 1.0 ---
//_/_/
//_/_/ Redistribution and use in source and binary forms, with or without
//_/_/ modification, is permitted provided that the following conditions
//_/_/ are met:
//_/_/ - Redistributions of source code must retain the above copyright
//_/_/   and collaboration notice, this list of conditions and the
//_/_/   following disclaimer.
//_/_/ - Redistributions in binary form must reproduce the above copyright
//_/_/   notice, this list of conditions and the following disclaimer 
//_/_/   in the documentation and/or other materials provided with 
//_/_/   the distribution.
//_/_/
//_/_/ - Neither the name of its copyright holders nor the names of its
//_/_/   contributors may be used to endorse or promote products
//_/_/   derived from this software without specific prior 
//_/_/   written permission.
//_/_/   
//_/_/ - CADIA Clause: The license granted in and to the software 
//_/_/   under this agreement is a limited-use license. 
//_/_/   The software may not be used in furtherance of:
//_/_/    (i)   intentionally causing bodily injury or severe emotional 
//_/_/          distress to any person;
//_/_/    (ii)  invading the personal privacy or violating the human 
//_/_/          rights of any person; or
//_/_/    (iii) committing or preparing for any act of war.
//_/_/
//_/_/ THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
//_/_/ CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
//_/_/ INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
//_/_/ MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
//_/_/ DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
//_/_/ CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//_/_/ SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
//_/_/ BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
//_/_/ SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
//_/_/ INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
//_/_/ WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
//_/_/ NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//_/_/ OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
//_/_/ OF SUCH DAMAGE.
//_/_/ 
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#ifndef video_screen_h
#define video_screen_h

#include "../r_code/object.h"

namespace video_screen {

class VideoScreen {
public:
  /**
   * Create a VideoScreen of the given dimensions.
   * \param width The screen width.
   * \param height The screen height.
   */
  VideoScreen(int width, int height);

  ~VideoScreen() {
    delete screen_;
    if (fovea_patterns_)
      delete fovea_patterns_;
  }

  /**
   * Get the fovea_patterns_ from objects and initialize the screen.
   * \param objects The objects array from load(), used to call find_object.
   * \return True for success, false if can't initialize.
   */
  bool load(const std::vector<r_code::Code *> *objects);

  /**
   * Move the eye (and the fovea with it) by the given delta X and Y. Set the output
   * values to that actual delta moved, which can be less if hitting the edge screen,
   * which allows for the full fovea to still be within the screen boundaries.
   * \param delta_x The requested move delta X.
   * \param delta_y The requested move delta Y.
   * \param actual_delta_x Set this to the actual delta X moved.
   * \param actual_delta_y Set this to the actual delta Y moved.
   */
  void move_eye(int delta_x, int delta_y, int& actual_delta_x, int& actual_delta_y);

  /**
   * Get the fovea pattern at fovea_x_ and fovea_y_.
   */
  r_code::Code* get_fovea_pattern() const;

private:
  /**
   * Set all screen values to false.
   */
  void clear_screen() {
    for (size_t i = 0; i < width_ * height_; ++i)
      screen_[i] = false;
  }

  /**
   * Get the screen value at x and y.
   * \param x The x value from 0 to width - 1.
   * \param y The y value from 0 to height - 1.
   * \return The screen value.
   */
  bool get_screen(int x, int y) const {
    if (x < 0 || x >= width_ || y < 0 || y >= height_)
      return false;

    return screen_[y * width_ + x];
  }

  /**
   * Set the screen at x and y to the value.
   * \param x The x value from 0 to width - 1.
   * \param y The y value from 0 to height - 1.
   * \param value The screen value.
   */
  void set_screen(int x, int y, bool value) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_)
      return;

    screen_[y * width_ + x] = value;
  }

  /**
   * Draw a vertical line centered on x,y extending 1 pixels up and down (3 pixels).
   */
  void draw_paddle(int x, int y) {
    set_screen(x, y - 1, true);
    set_screen(x, y, true);
    set_screen(x, y + 1, true);
  }

  /**
   * Draw a ball centered on x,y.
   */
  void draw_ball(int x, int y) {
    // A ball is just a dot for now.
    set_screen(x, y, true);
  }

  int width_;
  int height_;
  bool* screen_;
  int fovea_x_;
  int fovea_y_;
  r_code::Code** fovea_patterns_;
};

}
#endif
