/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards ACI-1339863,
   ACI-1339881 and ACI-1339840
   ======================================================================================= */
//
// Image.h
//

#ifndef GVT_RENDER_DATA_SCENE_IMAGE_H
#define GVT_RENDER_DATA_SCENE_IMAGE_H

#include <gvt/render/data/scene/ColorAccumulator.h>

#include <string>
#include <gvt/core/math.h>

namespace gvt {
namespace render {
namespace data {
namespace scene {
/// image buffer
/** image buffer used to accumulate the final image
*/
class Image {
public:
  // clang-format off
  enum ImageFormat {
    PPM
  };
  // clang-format on

  int GetBufferSize() {return bufferSize;}

  void SetBufferBackground(int pixel, bool isBackground)
  {
    int offset = (pixel) / 8;
    int maskBit = (pixel % 8);
    if(isBackground)
    { 
      //calculate mask bit
      unsigned char mask = 0x1<<maskBit;
      rgb[backGroundBitSizeStart +offset ] |= mask; 
    }
    else
    {
      unsigned char mask = 0x1<<maskBit;
      mask = ~mask;
      rgb[backGroundBitSizeStart +offset ] &= mask; 
    }

  }

  bool GetBackGroundPixel(int pixel, unsigned char * buffer = nullptr, int offset = 0)
  {
    if(buffer == nullptr) buffer = rgb;

    int buffoffset = (pixel) / 8;
    int maskBit = (pixel % 8);
    unsigned char mask = 0x1<<maskBit;

    unsigned char val = (unsigned char ) (buffer[backGroundBitSizeStart + buffoffset + offset] & mask);

    return  val > 0? true: false;
  }

  Image(int w, int h, std::string fn = "gvt_image", glm::vec3 backgroundColor = glm::vec3(0,0,0),ImageFormat f = PPM)
      : width(w), height(h), filename(fn), format(f),backgroundColor(backgroundColor) {

    int imageSize = 3 * width * height;
    int backGroundBitSize = (width * height + 8 -1)/8;

    //int backGroundBitSize = width * height;
    int size =  imageSize + backGroundBitSize;
    bufferSize = size;
    rgb = new unsigned char[size];
    for (int i = 0; i < imageSize; i+=3) 
    {
      rgb[i + 0] = (unsigned char)(backgroundColor[0] * 256.f);
      rgb[i + 1] = (unsigned char)(backgroundColor[1] * 256.f);
      rgb[i + 2] = (unsigned char)(backgroundColor[2] * 256.f);
    };

    // intially set all as background
    for(int i = imageSize;i<size;i++) rgb[i] = (unsigned char) 0xFF;

    backGroundBitSizeStart = imageSize;
  }

  void Add(int pixel, float *buf) {
    int index = 3 * pixel;
    // GVT-31 uh - oh not rly sure what do here...


    rgb[index + 0] = (unsigned char)(buf[0] * 256.f);
    rgb[index + 1] = (unsigned char)(buf[1] * 256.f);
    rgb[index + 2] = (unsigned char)(buf[2] * 256.f);
  }

  void Add(int pixel, ColorAccumulator &ca) {
    int index = 3 * pixel;

    if(ca.rgba[3] == 0) return;  // ignore un-intialized color accumulators

    if(ca.rgba[0] >=0)
    {
      rgb[index + 0] = (unsigned char)(ca.rgba[0]  * 255.f);
      rgb[index + 1] = (unsigned char)(ca.rgba[1]  * 255.f);
      rgb[index + 2] = (unsigned char)(ca.rgba[2]  * 255.f);
      if (rgb[index + 0] > 255.f) rgb[index + 0] = 255;
      if (rgb[index + 1] > 255.f) rgb[index + 1] = 255;
      if (rgb[index + 2] > 255.f) rgb[index + 2] = 255;
      SetBufferBackground(pixel, false);
    }
  }

  void Add(int pixel, ColorAccumulator &ca, float w) {
    int index = 3 * pixel;

    if(ca.rgba[3] == 0) return;  // ignore un-intialized color accumulators

    if(ca.rgba[0] >=0)
    {
      rgb[index + 0] = ((unsigned char)(ca.rgba[0] * 255.f) * w);
      rgb[index + 1] = ((unsigned char)(ca.rgba[1] * 255.f) * w);
      rgb[index + 2] = ((unsigned char)(ca.rgba[2] * 255.f) * w);
      SetBufferBackground(pixel, false);
    }
  }

  unsigned char *GetBuffer() { return rgb; }

  void Write();
  void clear() 
  {
    for (int i = 0; i < backGroundBitSizeStart; i+=3) 
    {
      rgb[i + 0] = (unsigned char)(backgroundColor[0] * 256.f);
      rgb[i + 1] = (unsigned char)(backgroundColor[1] * 256.f);
      rgb[i + 2] = (unsigned char)(backgroundColor[2] * 256.f);
    };

    // intially set all as background
    for(int i = backGroundBitSizeStart;i<bufferSize;i++) rgb[i] = (unsigned char) 0xFF;
  }

  ~Image() { delete[] rgb; }
  int backGroundBitSizeStart;
private:
  int width, height;
  std::string filename;
  ImageFormat format;
  unsigned char *rgb;
  int bufferSize;
  glm::vec3 backgroundColor;
  
};
}
}
}
}

#endif // GVT_RENDER_DATA_SCENE_IMAGE_H
